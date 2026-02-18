/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"

#include "bsp/esp-bsp.h"
#include "app_vision_event.h"

static const char *TAG = "main";

#define UART_TEST_PORT   UART_NUM_1
#define UART_TEST_TX_PIN 37
#define UART_TEST_RX_PIN UART_PIN_NO_CHANGE
#define UART_TEST_BAUD   115200

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

void app_vision_event_uart_send_line(const char *line)
{
    if (line == NULL) {
        return;
    }

    uart_write_bytes(UART_TEST_PORT, line, strlen(line));
    uart_write_bytes(UART_TEST_PORT, "\n", 1);
}

static void vision_event_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG, "Initialize UART1 TX-only test");
    uart_init_tx_only();

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
