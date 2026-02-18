/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "bsp/esp-bsp.h"
#include "app_vision_event.h"
#include "protocol_uart_v1.h"

static const char *TAG = "main";

#define UART_TEST_PORT   UART_NUM_1
#define UART_TEST_TX_PIN 37
#define UART_TEST_RX_PIN UART_PIN_NO_CHANGE
#define UART_TEST_BAUD   115200
#define UART_V1_NODE_ID  0x01

#define UART_COUNTER_NVS_NAMESPACE "uart_v1"
#define UART_COUNTER_NVS_KEY       "counter"
#define UART_COUNTER_SAVE_INTERVAL 64U

static uint32_t s_uart_counter = 0;
static uint32_t s_frames_since_save = 0;

static void uart_init_tx_only(void)
{
    const uart_config_t cfg = {
        .baud_rate = UART_TEST_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_TEST_PORT, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_TEST_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(
        UART_TEST_PORT,
        UART_TEST_TX_PIN,
        UART_TEST_RX_PIN,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));
}

static void uart_counter_restore(void)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(UART_COUNTER_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        s_uart_counter = 0;
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "counter restore open failed: %s", esp_err_to_name(err));
        s_uart_counter = 0;
        return;
    }

    err = nvs_get_u32(handle, UART_COUNTER_NVS_KEY, &s_uart_counter);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        s_uart_counter = 0;
    } else if (err != ESP_OK) {
        ESP_LOGW(TAG, "counter restore read failed: %s", esp_err_to_name(err));
        s_uart_counter = 0;
    } else {
        ESP_LOGI(TAG, "counter restored=%lu", (unsigned long)s_uart_counter);
    }

    nvs_close(handle);
}

static void uart_counter_store(void)
{
    nvs_handle_t handle = 0;
    esp_err_t err = nvs_open(UART_COUNTER_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "counter save open failed: %s", esp_err_to_name(err));
        return;
    }

    err = nvs_set_u32(handle, UART_COUNTER_NVS_KEY, s_uart_counter);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "counter save failed: %s", esp_err_to_name(err));
    }

    nvs_close(handle);
    s_frames_since_save = 0;
}

static void uart_counter_flush_shutdown(void)
{
    uart_counter_store();
}

void app_vision_event_uart_send_payload_v1(uint8_t msg_type,
                                           uint8_t flags,
                                           uint8_t luma,
                                           uint8_t occupied,
                                           uint8_t stable_count,
                                           uint8_t raw_count)
{
    vision_uart_payload_v1_t payload = {
        .ver = UART_V1_VERSION,
        .msg_type = msg_type,
        .node_id = UART_V1_NODE_ID,
        .flags = flags,
        .luma = luma,
        .occupied = occupied,
        .stable_count = stable_count,
        .raw_count = raw_count,
        .counter = ++s_uart_counter,
        .uptime_s = (uint32_t)(esp_timer_get_time() / 1000000ULL),
    };

    uint8_t frame[UART_V1_FRAME_LEN];
    size_t frame_len = build_uart_frame_v1(&payload, frame);
    int bytes_written = uart_write_bytes(UART_TEST_PORT, (const char *)frame, frame_len);
    if (bytes_written != (int)frame_len) {
        ESP_LOGW(TAG, "uart write short: %d/%u", bytes_written, (unsigned)frame_len);
    }
    uart_wait_tx_done(UART_TEST_PORT, pdMS_TO_TICKS(20));

    s_frames_since_save++;
    if (s_frames_since_save >= UART_COUNTER_SAVE_INTERVAL) {
        uart_counter_store();
    }

    ESP_LOGI(TAG, "[UART_TX] t=%lu ctr=%lu occ=%u l=%u sc=%u rc=%u",
             (unsigned long)payload.uptime_s,
             (unsigned long)payload.counter,
             payload.occupied,
             payload.luma,
             payload.stable_count,
             payload.raw_count);
}

static void vision_event_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Initialize UART1 TX-only test");
    uart_init_tx_only();
    uart_counter_restore();

    ESP_LOGI(TAG, "Initialize I2C");
    i2c_master_bus_handle_t i2c_handle = NULL;
    ESP_ERROR_CHECK(bsp_i2c_init());
    bsp_get_i2c_bus_handle(&i2c_handle);

    ESP_LOGI(TAG, "Start event-only pedestrian detection");
    ESP_ERROR_CHECK(app_vision_event_start(i2c_handle));

    ESP_LOGI(TAG, "Vision event mode running");
    while (1) {
        // HELLO_FROM_ESP32 disabled: UART now forwards app_vision_event heartbeat lines.
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
    ESP_ERROR_CHECK(esp_register_shutdown_handler(uart_counter_flush_shutdown));

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
