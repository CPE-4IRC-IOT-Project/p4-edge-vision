#include <algorithm>
#include <inttypes.h>

#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "driver/ppa.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_cache_private.h"
#include "bsp/esp-bsp.h"
#include "lvgl.h"

#include "app_video.h"
#include "app_pedestrian_detect.h"
#include "app_vision_event.h"
#include "app_drawing_utils.h"

extern "C" {
#include "app_video_utils.h"
}

static const char *TAG = "app_vision_event";

static int s_video_fd = -1;
static uint32_t s_frame_count = 0;
static lv_obj_t *s_canvas = nullptr;
static lv_obj_t *s_mode_label = nullptr;
static uint8_t *s_preview_buffers[EXAMPLE_CAM_BUF_NUM] = {nullptr};
static size_t s_preview_buffer_size = 0;

static constexpr uint32_t WARMUP_FRAMES = 30;
static constexpr uint32_t INFERENCE_STRIDE = 3;
static constexpr uint32_t HEARTBEAT_INTERVAL_FRAMES = 120;
static constexpr float DETECT_SCORE_THRESHOLD = 0.5f;
static constexpr int PREVIEW_SCALE_LEVEL = 1;  // x1 (widest view available in current crop pipeline)

static esp_err_t init_display_preview(void)
{
    ESP_LOGI(TAG, "Initialize display preview");
    if (bsp_display_start() == NULL) {
        ESP_LOGE(TAG, "bsp_display_start failed");
        return ESP_FAIL;
    }
    ESP_RETURN_ON_ERROR(bsp_display_backlight_on(), TAG, "backlight on failed");

    if (!bsp_display_lock(0)) {
        ESP_LOGE(TAG, "display lock failed");
        return ESP_FAIL;
    }
    s_canvas = lv_canvas_create(lv_scr_act());
    lv_obj_set_size(s_canvas, BSP_LCD_H_RES, BSP_LCD_V_RES);
    lv_obj_set_pos(s_canvas, 0, 0);
    lv_obj_clear_flag(s_canvas, LV_OBJ_FLAG_SCROLLABLE);

    s_mode_label = lv_label_create(lv_scr_act());
    lv_label_set_text(s_mode_label, "MODE: PEDESTRIAN");
    lv_obj_align(s_mode_label, LV_ALIGN_TOP_MID, 0, 6);
    lv_obj_set_style_bg_opa(s_mode_label, LV_OPA_50, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_mode_label, lv_color_hex(0x101010), LV_PART_MAIN);
    lv_obj_set_style_text_color(s_mode_label, lv_color_hex(0xFFFFFF), LV_PART_MAIN);
    lv_obj_set_style_pad_hor(s_mode_label, 8, LV_PART_MAIN);
    lv_obj_set_style_pad_ver(s_mode_label, 2, LV_PART_MAIN);
    bsp_display_unlock();

    return ESP_OK;
}

static esp_err_t init_preview_buffers(void)
{
    size_t cache_line_size = 0;
    ESP_RETURN_ON_ERROR(esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &cache_line_size), TAG, "cache alignment failed");

    s_preview_buffer_size = BSP_LCD_H_RES * BSP_LCD_V_RES * 2;
    for (int i = 0; i < EXAMPLE_CAM_BUF_NUM; i++) {
        s_preview_buffers[i] = static_cast<uint8_t *>(
            heap_caps_aligned_calloc(cache_line_size, 1, s_preview_buffer_size, MALLOC_CAP_SPIRAM)
        );
        if (s_preview_buffers[i] == nullptr) {
            ESP_LOGE(TAG, "preview buffer alloc failed at index %d", i);
            return ESP_ERR_NO_MEM;
        }
    }

    return ESP_OK;
}

static void camera_frame_cb(uint8_t *camera_buf, uint8_t camera_buf_index,
                            uint32_t width, uint32_t height, size_t camera_buf_len)
{
    (void)camera_buf_len;
    s_frame_count++;

    uint8_t *preview_buf = s_preview_buffers[camera_buf_index % EXAMPLE_CAM_BUF_NUM];
    if (preview_buf == nullptr) {
        return;
    }

    esp_err_t ret = app_image_process_video_frame(
        camera_buf,
        width,
        height,
        PREVIEW_SCALE_LEVEL,
        PPA_SRM_ROTATION_ANGLE_0,
        preview_buf,
        s_preview_buffer_size
    );
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "frame processing failed: 0x%x", ret);
        return;
    }

    int ped_count = 0;
    float best_score = 0.0f;

    if (s_frame_count <= WARMUP_FRAMES) {
        // Warm-up only; show preview without running inference.
    } else if ((s_frame_count % INFERENCE_STRIDE) == 0) {
        auto results = app_pedestrian_detect(
            reinterpret_cast<uint16_t *>(preview_buf),
            BSP_LCD_H_RES,
            BSP_LCD_V_RES
        );

        for (const auto &res : results) {
            if (res.score < DETECT_SCORE_THRESHOLD || res.box.size() < 4) {
                continue;
            }

            ped_count++;
            best_score = std::max(best_score, res.score);
            draw_rectangle_rgb(
                reinterpret_cast<uint16_t *>(preview_buf),
                BSP_LCD_H_RES,
                BSP_LCD_V_RES,
                res.box[0], res.box[1], res.box[2], res.box[3],
                0, 0,
                255, 0, 0,
                3,
                false
            );
        }

        if (ped_count > 0) {
            ESP_LOGI(TAG, "frame=%" PRIu32 " pedestrians=%d best_score=%.2f",
                     s_frame_count, ped_count, best_score);
        } else if ((s_frame_count % HEARTBEAT_INTERVAL_FRAMES) == 0) {
            ESP_LOGI(TAG, "frame=%" PRIu32 " no pedestrian", s_frame_count);
        }
    }

    swap_rgb565_bytes(reinterpret_cast<uint16_t *>(preview_buf), BSP_LCD_H_RES * BSP_LCD_V_RES);

    if (s_canvas && bsp_display_lock(0)) {
        lv_canvas_set_buffer(s_canvas, preview_buf, BSP_LCD_H_RES, BSP_LCD_V_RES, LV_IMG_CF_TRUE_COLOR);
        lv_refr_now(NULL);
        bsp_display_unlock();
    }
}

extern "C" esp_err_t app_vision_event_start(i2c_master_bus_handle_t i2c_handle)
{
    ESP_LOGI(TAG, "Initialize pedestrian model");
    PedestrianDetect *detector = get_pedestrian_detect();
    if (detector == nullptr) {
        ESP_LOGE(TAG, "Pedestrian model init failed");
        return ESP_FAIL;
    }

    ESP_RETURN_ON_ERROR(init_display_preview(), TAG, "display preview init failed");
    ESP_RETURN_ON_ERROR(app_video_utils_init(), TAG, "video utils init failed");
    ESP_RETURN_ON_ERROR(init_preview_buffers(), TAG, "preview buffers init failed");

    ESP_LOGI(TAG, "Initialize camera");
    ESP_RETURN_ON_ERROR(app_video_main(i2c_handle), TAG, "app_video_main failed");

    s_video_fd = app_video_open((char *)EXAMPLE_CAM_DEV_PATH, APP_VIDEO_FMT);
    if (s_video_fd < 0) {
        ESP_LOGE(TAG, "app_video_open failed");
        return ESP_FAIL;
    }

    ESP_RETURN_ON_ERROR(app_video_set_bufs(s_video_fd, EXAMPLE_CAM_BUF_NUM, NULL), TAG, "app_video_set_bufs failed");
    ESP_RETURN_ON_ERROR(app_video_register_frame_operation_cb(camera_frame_cb), TAG, "register frame cb failed");
    ESP_RETURN_ON_ERROR(app_video_stream_task_start(s_video_fd, 1), TAG, "stream task start failed");

    ESP_LOGI(TAG, "Event-only pipeline started");
    return ESP_OK;
}
