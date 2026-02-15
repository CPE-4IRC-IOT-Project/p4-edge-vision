/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp/esp-bsp.h"
#include "app_vision_event.h"

static const char *TAG = "main";

static void vision_event_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Initialize I2C");
    i2c_master_bus_handle_t i2c_handle = NULL;
    ESP_ERROR_CHECK(bsp_i2c_init());
    bsp_get_i2c_bus_handle(&i2c_handle);

    ESP_LOGI(TAG, "Start event-only pedestrian detection");
    ESP_ERROR_CHECK(app_vision_event_start(i2c_handle));

    ESP_LOGI(TAG, "Vision event mode running");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    // Initialize NVS
    ESP_LOGI(TAG, "Initialize NVS");
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Keep flashlight available even in headless mode
    ESP_LOGI(TAG, "Initialize the flashlight");
    ESP_ERROR_CHECK(bsp_flashlight_init());

    BaseType_t task_ok = xTaskCreatePinnedToCore(
        vision_event_task,
        "vision_event",
        8 * 1024,
        NULL,
        5,
        NULL,
        1
    );
    ESP_ERROR_CHECK(task_ok == pdPASS ? ESP_OK : ESP_FAIL);
}
