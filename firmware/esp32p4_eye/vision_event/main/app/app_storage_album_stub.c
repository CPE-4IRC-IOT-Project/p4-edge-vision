#include <string.h>
#include "esp_log.h"
#include "app_storage.h"
#include "app_album.h"

static const char *TAG = "app_storage_album_stub";

typedef struct {
    bool valid;
    bool gyroscope_on;
    bool od_on;
    uint8_t resolution_idx; /* 0:720P, 1:1080P, 2:480P */
    bool flash_on;
} stub_settings_t;

static stub_settings_t s_settings = {
    .valid = false,
    .gyroscope_on = false,
    .od_on = true,
    .resolution_idx = 1,
    .flash_on = true,
};

static bool s_interval_valid = false;
static uint16_t s_interval_time = 0;
static uint16_t s_magnification = 0;

static bool s_camera_valid = false;
static uint32_t s_contrast = DEFAULT_CONTRAST_PERCENT;
static uint32_t s_saturation = DEFAULT_SATURATION_PERCENT;
static uint32_t s_brightness = DEFAULT_BRIGHTNESS_PERCENT;
static uint32_t s_hue = DEFAULT_HUE_PERCENT;

static bool s_interval_state_active = false;
static uint32_t s_interval_state_next_wake = 0;

static uint16_t s_photo_count = 0;
static bool s_gyroscope_setting = false;
static bool s_coco_od_enabled = true;
static int s_album_image_count = 0;
static int s_album_current_index = 0;

esp_err_t app_storage_init(void)
{
    ESP_LOGW(TAG, "Storage disabled: using in-memory stubs");
    return ESP_OK;
}

esp_err_t app_storage_save_picture(const uint8_t *data, size_t len)
{
    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    /* Intentionally no persistent write when storage is disabled. */
    return ESP_OK;
}

esp_err_t app_storage_save_settings(settings_info_t *settings, uint16_t interval_time, uint16_t magnification)
{
    if (settings == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    s_settings.gyroscope_on = (settings->gyroscope && strcmp(settings->gyroscope, "On") == 0);
    s_settings.od_on = (settings->od && strcmp(settings->od, "On") == 0);
    s_settings.flash_on = (settings->flash && strcmp(settings->flash, "On") == 0);

    if (settings->resolution && strcmp(settings->resolution, "1080P") == 0) {
        s_settings.resolution_idx = 1;
    } else if (settings->resolution && strcmp(settings->resolution, "480P") == 0) {
        s_settings.resolution_idx = 2;
    } else {
        s_settings.resolution_idx = 0;
    }

    s_settings.valid = true;
    s_interval_time = interval_time;
    s_magnification = magnification;
    s_interval_valid = true;
    return ESP_OK;
}

esp_err_t app_storage_load_settings(settings_info_t *settings, uint16_t *interval_time, uint16_t *magnification)
{
    if (settings == NULL || interval_time == NULL || magnification == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_settings.valid) {
        settings->gyroscope = s_settings.gyroscope_on ? "On" : "Off";
        settings->od = s_settings.od_on ? "On" : "Off";
        settings->flash = s_settings.flash_on ? "On" : "Off";
        settings->resolution = (s_settings.resolution_idx == 1) ? "1080P" :
                               (s_settings.resolution_idx == 2) ? "480P" : "720P";
    }

    if (s_interval_valid) {
        *interval_time = s_interval_time;
        *magnification = s_magnification;
    }

    return ESP_OK;
}

esp_err_t app_storage_save_interval_state(bool is_active, uint32_t next_wake_time)
{
    s_interval_state_active = is_active;
    s_interval_state_next_wake = next_wake_time;
    return ESP_OK;
}

esp_err_t app_storage_get_interval_state(bool *is_active, uint32_t *next_wake_time)
{
    if (is_active == NULL || next_wake_time == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *is_active = s_interval_state_active;
    *next_wake_time = s_interval_state_next_wake;
    return ESP_OK;
}

esp_err_t app_storage_save_camera_settings(uint32_t contrast, uint32_t saturation,
                                           uint32_t brightness, uint32_t hue)
{
    s_contrast = contrast;
    s_saturation = saturation;
    s_brightness = brightness;
    s_hue = hue;
    s_camera_valid = true;
    return ESP_OK;
}

esp_err_t app_storage_load_camera_settings(uint32_t *contrast, uint32_t *saturation,
                                           uint32_t *brightness, uint32_t *hue)
{
    if (contrast == NULL || saturation == NULL || brightness == NULL || hue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (s_camera_valid) {
        *contrast = s_contrast;
        *saturation = s_saturation;
        *brightness = s_brightness;
        *hue = s_hue;
    }
    return ESP_OK;
}

esp_err_t app_storage_save_photo_count(uint16_t count)
{
    s_photo_count = count;
    return ESP_OK;
}

esp_err_t app_storage_get_photo_count(uint16_t *count)
{
    if (count == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *count = s_photo_count;
    return ESP_OK;
}

esp_err_t app_storage_save_gyroscope_setting(bool enabled)
{
    s_gyroscope_setting = enabled;
    return ESP_OK;
}

esp_err_t app_storage_load_gyroscope_setting(bool *enabled)
{
    if (enabled == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *enabled = s_gyroscope_setting;
    return ESP_OK;
}

esp_err_t app_album_init(lv_obj_t *parent)
{
    (void)parent;
    return ESP_OK;
}

esp_err_t app_album_next_image(void)
{
    if (s_album_image_count > 0) {
        s_album_current_index = (s_album_current_index + 1) % s_album_image_count;
    }
    return ESP_OK;
}

esp_err_t app_album_prev_image(void)
{
    if (s_album_image_count > 0) {
        s_album_current_index = (s_album_current_index - 1 + s_album_image_count) % s_album_image_count;
    }
    return ESP_OK;
}

esp_err_t app_album_refresh(void)
{
    return ESP_OK;
}

void app_album_deinit(void)
{
}

int app_album_get_image_count(void)
{
    return s_album_image_count;
}

int app_album_get_current_index(void)
{
    return s_album_current_index;
}

esp_err_t app_album_delete_current_image(void)
{
    if (s_album_image_count > 0) {
        s_album_image_count--;
        if (s_album_current_index >= s_album_image_count) {
            s_album_current_index = (s_album_image_count > 0) ? (s_album_image_count - 1) : 0;
        }
    }
    return ESP_OK;
}

bool app_album_can_store_new_image(void)
{
    return true;
}

void app_album_photo_saved(void)
{
    s_album_image_count++;
}

bool app_video_stream_can_store_new_mp4(float estimated_size_mb)
{
    (void)estimated_size_mb;
    return true;
}

void app_album_enable_coco_od(bool enable)
{
    s_coco_od_enabled = enable;
    (void)s_coco_od_enabled;
}
