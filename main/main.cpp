/**
 * ESP32-P4-EYE - Camera capture + Pedestrian Detection -> UART output
 *
 * Pipeline :
 *   1. BSP init (I2C, PSRAM, camera power, XCLK, MIPI-CSI)
 *   2. V4L2 capture  640x480 RGB565
 *   3. Resize -> 224x224 RGB888 pour le modele PedestrianDetect (esp-dl)
 *   4. Inference -> nombre de personnes + confiance
 *   5. Envoi UART : "state,count,confidence\r\n"
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

/* BSP ESP32-P4-EYE */
#include "bsp/esp32_p4_eye.h"

/* V4L2 */
#include "linux/videodev2.h"

/* esp-dl : detection de pietons */
#include "dl_detect_pedestrian_detect.hpp"
#include "dl_image.hpp"

/* ---------- configuration ---------- */
#define UART_PORT       UART_NUM_0
#define BAUD_RATE       115200

#define CAM_WIDTH       640
#define CAM_HEIGHT      480
#define CAM_PIX_FMT     V4L2_PIX_FMT_RGB565

#define MODEL_W         224
#define MODEL_H         224

#define NUM_BUFFERS     2
#define DETECT_PERIOD_MS 5000

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

        /* Resize 640x480 RGB565 -> 224x224 RGB888 */
        resize_rgb565_to_rgb888(frame, CAM_WIDTH, CAM_HEIGHT,
                                rgb_buf, MODEL_W, MODEL_H);

        /* Inference */
        dl::image::img_t img = {};
        img.data     = rgb_buf;
        img.width    = MODEL_W;
        img.height   = MODEL_H;
        img.pix_type = dl::image::DL_IMAGE_PIX_TYPE_RGB888;

        auto &results = detector->run(img);
        int people_count = (int)results.size();

        /* Meilleure confiance parmi les detections */
        int confidence = 0;
        for (auto &r : results) {
            int score_pct = (int)(r.score * 100.0f);
            if (score_pct > confidence) confidence = score_pct;
        }

        const char *state = (people_count > 0) ? "occupied" : "available";

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

        ESP_LOGI(TAG, "Detect: %s | people=%d | conf=%d%%",
                 state, people_count, confidence);

        vTaskDelay(pdMS_TO_TICKS(DETECT_PERIOD_MS));
    }

    /* Cleanup (jamais atteint) */
    delete detector;
    heap_caps_free(rgb_buf);
    close(cam_fd);
}
