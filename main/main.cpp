/**
 * ESP32-P4-EYE - Camera capture + Pedestrian Detection -> UART + LCD preview
 *
 * Pipeline :
 *   1. BSP init (I2C, PSRAM, camera power, XCLK, MIPI-CSI, LCD)
 *   2. V4L2 capture  1920x1080 RGB565
 *   3. Resize -> 224x224 RGB888 pour le modele PedestrianDetect (esp-dl)
 *   4. Inference -> nombre de personnes + confiance
 *   5. Resize -> 240x240 RGB565 pour preview LCD (ST7789)
 *   6. Envoi UART : "state,count,confidence\r\n"
 */

#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

/* ESP-IDF */
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

/* BSP ESP32-P4-EYE */
#include "bsp/esp32_p4_eye.h"
#include "bsp/display.h"
#include "esp_video_device.h"

/* LVGL */
#include "lvgl.h"

/* V4L2 */
#include "linux/videodev2.h"

/* esp-dl : detection de pietons */
#include "pedestrian_detect.hpp"
#include "dl_image_define.hpp"
#include "dl_detect_define.hpp"

/* ---------- configuration ---------- */
#define UART_PORT       UART_NUM_0
#define BAUD_RATE       115200

#define CAM_WIDTH       1920
#define CAM_HEIGHT      1080
#define CAM_PIX_FMT     V4L2_PIX_FMT_RGB565

#define MODEL_W         224
#define MODEL_H         224

#define NUM_BUFFERS     2
#define DETECT_PERIOD_MS 200

#define LCD_W           BSP_LCD_H_RES   /* 240 */
#define LCD_H           BSP_LCD_V_RES   /* 240 */

static const char *TAG = "cam-detect";

/* ---------- helpers ---------- */

/**
 * Convertit un pixel RGB565 (little-endian) en RGB888.
 */
static inline void rgb565_to_rgb888(uint16_t px, uint8_t *r, uint8_t *g, uint8_t *b)
{
    *r = (px >> 8) & 0xF8;
    *g = (px >> 3) & 0xFC;
    *b = (px << 3) & 0xF8;
}

/**
 * Redimensionne un buffer RGB565 (src_w x src_h) vers un buffer RGB888
 * (dst_w x dst_h) par nearest-neighbour.  Le buffer dst doit etre pre-alloue.
 */
static void resize_rgb565_to_rgb888(const uint16_t *src, int src_w, int src_h,
                                    uint8_t *dst, int dst_w, int dst_h)
{
    for (int y = 0; y < dst_h; y++) {
        int sy = y * src_h / dst_h;
        for (int x = 0; x < dst_w; x++) {
            int sx = x * src_w / dst_w;
            uint16_t px = src[sy * src_w + sx];
            int idx = (y * dst_w + x) * 3;
            rgb565_to_rgb888(px, &dst[idx], &dst[idx + 1], &dst[idx + 2]);
        }
    }
}

/**
 * Redimensionne un buffer RGB565 (src_w x src_h) vers un buffer RGB565
 * (dst_w x dst_h) par nearest-neighbour, pour le preview LCD.
 */
static void resize_rgb565(const uint16_t *src, int src_w, int src_h,
                          uint16_t *dst, int dst_w, int dst_h)
{
    for (int y = 0; y < dst_h; y++) {
        int sy = y * src_h / dst_h;
        for (int x = 0; x < dst_w; x++) {
            int sx = x * src_w / dst_w;
            dst[y * dst_w + x] = src[sy * src_w + sx];
        }
    }
}

/**
 * Dessine un rectangle (bounding box) dans un buffer RGB565.
 * Couleur verte = 0x07E0 en RGB565.
 */
static void draw_rect_rgb565(uint16_t *buf, int buf_w, int buf_h,
                             int x0, int y0, int x1, int y1, uint16_t color)
{
    /* Clamp */
    if (x0 < 0) x0 = 0;
    if (y0 < 0) y0 = 0;
    if (x1 >= buf_w) x1 = buf_w - 1;
    if (y1 >= buf_h) y1 = buf_h - 1;
    if (x0 > x1 || y0 > y1) return;

    /* Lignes horizontales (haut et bas) */
    for (int x = x0; x <= x1; x++) {
        buf[y0 * buf_w + x] = color;
        buf[y1 * buf_w + x] = color;
    }
    /* Lignes verticales (gauche et droite) */
    for (int y = y0; y <= y1; y++) {
        buf[y * buf_w + x0] = color;
        buf[y * buf_w + x1] = color;
    }
}

/* ---------- V4L2 helpers ---------- */

static int cam_set_format(int fd)
{
    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type                = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width       = CAM_WIDTH;
    fmt.fmt.pix.height      = CAM_HEIGHT;
    fmt.fmt.pix.pixelformat = CAM_PIX_FMT;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        ESP_LOGE(TAG, "VIDIOC_S_FMT failed");
        return -1;
    }
    ESP_LOGI(TAG, "Format set: %ux%u", fmt.fmt.pix.width, fmt.fmt.pix.height);
    return 0;
}

static int cam_request_buffers(int fd)
{
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count  = NUM_BUFFERS;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed");
        return -1;
    }
    return 0;
}

static int cam_queue_buffers(int fd)
{
    for (int i = 0; i < NUM_BUFFERS; i++) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index  = i;
        if (ioctl(fd, VIDIOC_QBUF, &buf) < 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF[%d] failed", i);
            return -1;
        }
    }
    return 0;
}

static int cam_stream_on(int fd)
{
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) < 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMON failed");
        return -1;
    }
    return 0;
}

/* ---------- main ---------- */

extern "C" void app_main(void)
{
    /* -- 1. UART ------------------------------------------ */
    uart_driver_install(UART_PORT, 1024, 0, 0, NULL, 0);
    uart_set_baudrate(UART_PORT, BAUD_RATE);
    vTaskDelay(pdMS_TO_TICKS(500));

    const char *banner = "\r\n[ESP32-P4-EYE] Camera + PedestrianDetect started\r\n";
    uart_write_bytes(UART_PORT, banner, strlen(banner));
    uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(50));

    /* -- 2. BSP : init camera (I2C, power, XCLK, MIPI-CSI) -- */
    ESP_LOGI(TAG, "Initializing BSP camera...");
    bsp_camera_cfg_t cam_cfg = {};
    ESP_ERROR_CHECK(bsp_camera_start(&cam_cfg));
    ESP_LOGI(TAG, "BSP camera started");

    /* -- 2b. BSP : init LCD display (ST7789 240x240) ------ */
    ESP_LOGI(TAG, "Initializing display...");
    lv_display_t *lvgl_disp = bsp_display_start();
    assert(lvgl_disp != NULL);
    bsp_display_brightness_set(80);
    ESP_LOGI(TAG, "Display started (%dx%d)", LCD_W, LCD_H);

    /* Creer un canvas LVGL pour le preview camera */
    uint16_t *lcd_buf = (uint16_t *)heap_caps_malloc(LCD_W * LCD_H * 2, MALLOC_CAP_SPIRAM);
    assert(lcd_buf != NULL);

    lv_obj_t *canvas = NULL;
    lv_obj_t *lbl_status = NULL;
    bsp_display_lock(0);
    {
        canvas = lv_canvas_create(lv_screen_active());
        lv_canvas_set_buffer(canvas, lcd_buf, LCD_W, LCD_H, LV_COLOR_FORMAT_RGB565);
        lv_obj_center(canvas);

        /* Label d'etat en bas */
        lbl_status = lv_label_create(lv_screen_active());
        lv_obj_align(lbl_status, LV_ALIGN_BOTTOM_MID, 0, -4);
        lv_obj_set_style_text_color(lbl_status, lv_color_white(), 0);
        lv_obj_set_style_bg_color(lbl_status, lv_color_black(), 0);
        lv_obj_set_style_bg_opa(lbl_status, LV_OPA_60, 0);
        lv_label_set_text(lbl_status, "Starting...");
    }
    bsp_display_unlock();

    /* -- 3. Ouvrir le device V4L2 ------------------------- */
    int cam_fd = open(BSP_CAMERA_DEVICE, O_RDONLY);
    if (cam_fd < 0) {
        ESP_LOGE(TAG, "Cannot open %s", BSP_CAMERA_DEVICE);
        return;
    }
    ESP_LOGI(TAG, "Opened %s (fd=%d)", BSP_CAMERA_DEVICE, cam_fd);

    /* -- 4. Configurer capture V4L2 ----------------------- */
    if (cam_set_format(cam_fd) < 0)       return;
    if (cam_request_buffers(cam_fd) < 0)  return;
    if (cam_queue_buffers(cam_fd) < 0)    return;
    if (cam_stream_on(cam_fd) < 0)        return;

    /* -- 5. Allouer le buffer de redimensionnement (PSRAM) -- */
    size_t rgb_buf_size = MODEL_W * MODEL_H * 3;
    uint8_t *rgb_buf = (uint8_t *)heap_caps_malloc(rgb_buf_size, MALLOC_CAP_SPIRAM);
    if (!rgb_buf) {
        ESP_LOGE(TAG, "Failed to allocate RGB buffer in PSRAM (%u bytes)",
                 (unsigned)rgb_buf_size);
        return;
    }

    /* -- 6. Creer le detecteur de pietons (esp-dl) -------- */
    ESP_LOGI(TAG, "Loading PedestrianDetect model...");
    PedestrianDetect *detector = new PedestrianDetect();
    ESP_LOGI(TAG, "Model loaded");

    /* -- 7. Boucle de capture + detection ----------------- */
    while (true) {
        /* Dequeue un frame rempli */
        struct v4l2_buffer vbuf;
        memset(&vbuf, 0, sizeof(vbuf));
        vbuf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vbuf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(cam_fd, VIDIOC_DQBUF, &vbuf) < 0) {
            ESP_LOGE(TAG, "VIDIOC_DQBUF failed");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        const uint16_t *frame = (const uint16_t *)vbuf.m.userptr;
        int64_t t_start = esp_timer_get_time();

        /* Resize 1920x1080 RGB565 -> 224x224 RGB888 */
        resize_rgb565_to_rgb888(frame, CAM_WIDTH, CAM_HEIGHT,
                                rgb_buf, MODEL_W, MODEL_H);
        int64_t t_resize = esp_timer_get_time();

        /* Inference */
        dl::image::img_t img = {};
        img.data     = rgb_buf;
        img.width    = MODEL_W;
        img.height   = MODEL_H;
        img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;

        auto &results = detector->run(img);
        int64_t t_infer = esp_timer_get_time();
        int people_count = (int)results.size();

        /* Meilleure confiance parmi les detections */
        int confidence = 0;
        for (const auto &r : results) {
            int score_pct = (int)(r.score * 100.0f);
            if (score_pct > confidence) confidence = score_pct;
        }

        const char *state = (people_count > 0) ? "occupied" : "available";

        /* -- Preview LCD : resize 1920x1080 -> 240x240 --- */
        resize_rgb565(frame, CAM_WIDTH, CAM_HEIGHT, lcd_buf, LCD_W, LCD_H);

        /* Dessiner les bounding boxes sur le preview LCD */
        /* Les boxes du modele sont en coords 224x224, on les mappe vers 240x240 */
        for (const auto &r : results) {
            if (r.box.size() >= 4) {
                int bx0 = r.box[0] * LCD_W / MODEL_W;
                int by0 = r.box[1] * LCD_H / MODEL_H;
                int bx1 = r.box[2] * LCD_W / MODEL_W;
                int by1 = r.box[3] * LCD_H / MODEL_H;
                /* Vert = 0x07E0 en RGB565 */
                draw_rect_rgb565(lcd_buf, LCD_W, LCD_H, bx0, by0, bx1, by1, 0x07E0);
            }
        }

        /* Mettre a jour le canvas + label LVGL */
        bsp_display_lock(0);
        {
            lv_obj_invalidate(canvas);
            char status_txt[48];
            snprintf(status_txt, sizeof(status_txt), "%s | %d pers | %d%%",
                     state, people_count, confidence);
            lv_label_set_text(lbl_status, status_txt);
        }
        bsp_display_unlock();

        /* Re-queue le buffer */
        if (ioctl(cam_fd, VIDIOC_QBUF, &vbuf) < 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF failed");
        }

        /* -- Envoi UART ----------------------------------- */
        char msg[64];
        int len = snprintf(msg, sizeof(msg), "%s,%d,%d\r\n",
                           state, people_count, confidence);
        if (len > 0) {
            uart_write_bytes(UART_PORT, msg, len);
            uart_wait_tx_done(UART_PORT, pdMS_TO_TICKS(50));
        }

        int64_t t_end = esp_timer_get_time();
        int resize_ms = (int)((t_resize - t_start) / 1000);
        int infer_ms  = (int)((t_infer - t_resize) / 1000);
        int total_ms  = (int)((t_end - t_start) / 1000);

        ESP_LOGI(TAG, "Detect: %s | people=%d | conf=%d%% | resize=%dms infer=%dms total=%dms",
                 state, people_count, confidence, resize_ms, infer_ms, total_ms);

        vTaskDelay(pdMS_TO_TICKS(DETECT_PERIOD_MS));
    }

    /* Cleanup (jamais atteint) */
    delete detector;
    heap_caps_free(rgb_buf);
    heap_caps_free(lcd_buf);
    close(cam_fd);
}
