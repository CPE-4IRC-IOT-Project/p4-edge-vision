/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/rtc_io.h"

#include "bsp/esp-bsp.h"
#include "app_vision_event.h"
#include "app_ble_temp_client.h"
#include "protocol_uart_v1.h"

static const char *TAG = "main";
#define HCI_UART_RX_ONLY_TEST_MODE 0
#define HCI_UART_SANITY_TEST_MODE 0
#define C6_UART_TEMP_RX_MODE 1

#define C6_UART_PORT ((uart_port_t)CONFIG_BT_NIMBLE_TRANSPORT_UART_PORT)
#define C6_UART_TX_PIN CONFIG_BT_NIMBLE_UART_TX_PIN
#define C6_UART_RX_PIN CONFIG_BT_NIMBLE_UART_RX_PIN
#define C6_UART_BAUD CONFIG_BT_NIMBLE_HCI_UART_BAUDRATE

#define UART_TEST_PORT   UART_NUM_1
#define UART_TEST_TX_PIN 37
#define UART_TEST_RX_PIN UART_PIN_NO_CHANGE
#define UART_TEST_BAUD   115200
#define UART_V1_NODE_ID  0x01

#define UART_COUNTER_NVS_NAMESPACE "uart_v1"
#define UART_COUNTER_NVS_KEY       "counter"
#define UART_COUNTER_SAVE_INTERVAL 64U
#define UART_TEMP_TX_PERIOD_MS     120000U

static uint32_t s_uart_counter = 0;
static uint32_t s_frames_since_save = 0;
static portMUX_TYPE s_temp_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_latest_temp_valid = false;
static float s_latest_temp_c = 0.0f;
static uint32_t s_latest_temp_ms = 0;

#if HCI_UART_SANITY_TEST_MODE
static void hci_uart_sanity_test(void)
{
    const uart_port_t hci_uart_port = (uart_port_t)CONFIG_BT_NIMBLE_TRANSPORT_UART_PORT;
    const int hci_uart_tx_pin = CONFIG_BT_NIMBLE_UART_TX_PIN;
    const int hci_uart_rx_pin = CONFIG_BT_NIMBLE_UART_RX_PIN;
    const int hci_uart_baud = CONFIG_BT_NIMBLE_HCI_UART_BAUDRATE;

    const uart_config_t cfg = {
        .baud_rate = hci_uart_baud,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_LOGI(TAG,
             "HCI UART sanity test on UART%d (TX=%d RX=%d baud=%d)",
             (int)hci_uart_port, hci_uart_tx_pin, hci_uart_rx_pin, hci_uart_baud);

    esp_err_t err = uart_driver_install(hci_uart_port, 256, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "HCI test uart_driver_install failed: %s", esp_err_to_name(err));
        return;
    }

    err = uart_param_config(hci_uart_port, &cfg);
    if (err == ESP_OK) {
        err = uart_set_pin(
            hci_uart_port,
            hci_uart_tx_pin,
            hci_uart_rx_pin,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE
        );
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "HCI test UART config failed: %s", esp_err_to_name(err));
        uart_driver_delete(hci_uart_port);
        return;
    }

    uart_flush_input(hci_uart_port);

    uint8_t rx[256] = {0};
    int rx_len = 0;
    const int64_t deadline_us = esp_timer_get_time() + 2500000; // 2.5 s timeout
    while (esp_timer_get_time() < deadline_us && rx_len < (int)sizeof(rx)) {
        int n = uart_read_bytes(
            hci_uart_port,
            rx + rx_len,
            sizeof(rx) - rx_len,
            pdMS_TO_TICKS(20)
        );
        if (n > 0) {
            rx_len += n;
        }
    }

    if (rx_len == 0) {
        ESP_LOGE(TAG, "HCI UART test failed: no bytes received from C6");
    } else {
        ESP_LOGI(TAG, "HCI UART test received %d bytes", rx_len);
        for (int i = 0; i < rx_len; ++i) {
            ESP_LOGI(TAG, "HCI RX[%d]=0x%02X", i, rx[i]);
        }
    }

    uart_driver_delete(hci_uart_port);
}
#endif

static void c6_uart_rx_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = C6_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(C6_UART_PORT, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(C6_UART_PORT, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(C6_UART_PORT,
                                 C6_UART_TX_PIN,
                                 C6_UART_RX_PIN,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG,
             "C6 UART RX ready on UART%d (RX=%d TX=%d baud=%d)",
             (int)C6_UART_PORT,
             C6_UART_RX_PIN,
             C6_UART_TX_PIN,
             C6_UART_BAUD);
}

static void temp_cache_update(float temp_c)
{
    taskENTER_CRITICAL(&s_temp_lock);
    s_latest_temp_c = temp_c;
    s_latest_temp_ms = esp_log_timestamp();
    s_latest_temp_valid = true;
    taskEXIT_CRITICAL(&s_temp_lock);
}

static bool temp_cache_get_latest(float *out_temp_c, uint32_t *out_age_ms)
{
    bool valid = false;
    float temp_c = 0.0f;
    uint32_t age_ms = 0;

    taskENTER_CRITICAL(&s_temp_lock);
    valid = s_latest_temp_valid;
    if (valid) {
        temp_c = s_latest_temp_c;
        age_ms = esp_log_timestamp() - s_latest_temp_ms;
    }
    taskEXIT_CRITICAL(&s_temp_lock);

    if (!valid) {
        return false;
    }

    if (out_temp_c != NULL) {
        *out_temp_c = temp_c;
    }
    if (out_age_ms != NULL) {
        *out_age_ms = age_ms;
    }
    return true;
}

static bool app_get_latest_temp(float *out_temp_c, uint32_t *out_age_ms)
{
#if C6_UART_TEMP_RX_MODE
    return temp_cache_get_latest(out_temp_c, out_age_ms);
#else
    return app_ble_temp_client_get_latest(out_temp_c, out_age_ms);
#endif
}

static bool c6_uart_try_parse_temp(const char *line, float *out_temp_c)
{
    if (line == NULL || out_temp_c == NULL) {
        return false;
    }

    // Preferred compact bridge format from C6 controller.
    if (strncmp(line, "TEMP,", 5) == 0) {
        char *endptr = NULL;
        float temp_c = strtof(line + 5, &endptr);
        if (endptr != line + 5) {
            *out_temp_c = temp_c;
            return true;
        }
    }

    // Fallback for verbose logs like: "... temp=27.29 C".
    const char *temp_field = strstr(line, "temp=");
    if (temp_field != NULL) {
        char *endptr = NULL;
        float temp_c = strtof(temp_field + 5, &endptr);
        if (endptr != temp_field + 5) {
            *out_temp_c = temp_c;
            return true;
        }
    }

    return false;
}

static void c6_uart_handle_line(const char *line)
{
    float temp_c = 0.0f;
    if (c6_uart_try_parse_temp(line, &temp_c)) {
        temp_cache_update(temp_c);
        ESP_LOGI(TAG, "[C6_TEMP] %.2f C", (double)temp_c);
        return;
    }

    ESP_LOGI(TAG, "[C6_UART_RX] %s", line);
}

static void c6_uart_rx_task(void *arg)
{
    (void)arg;

    char line[96];
    size_t idx = 0;
    uint8_t byte = 0;

    while (1) {
        int n = uart_read_bytes(C6_UART_PORT, &byte, 1, pdMS_TO_TICKS(200));
        if (n != 1) {
            continue;
        }

        if (byte == '\r' || byte == '\n') {
            if (idx > 0) {
                line[idx] = '\0';
                c6_uart_handle_line(line);
                idx = 0;
            }
            continue;
        }

        if (idx < sizeof(line) - 1U) {
            line[idx++] = (char)byte;
        } else {
            idx = 0;
        }
    }
}

static void c6_enable_if_configured(void)
{
#if CONFIG_BSP_CONTROL_C6_EN_PIN
    rtc_gpio_init(BSP_C6_EN_PIN);
    rtc_gpio_set_direction(BSP_C6_EN_PIN, RTC_GPIO_MODE_OUTPUT_ONLY);
    rtc_gpio_pulldown_dis(BSP_C6_EN_PIN);
    rtc_gpio_pullup_dis(BSP_C6_EN_PIN);
    rtc_gpio_hold_dis(BSP_C6_EN_PIN);
    rtc_gpio_set_level(BSP_C6_EN_PIN, 1);
    rtc_gpio_hold_en(BSP_C6_EN_PIN);
    ESP_LOGI(TAG, "C6 enable pin asserted");
#endif
}

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

static void app_uart_send_temperature_payload_v1(void)
{
    float temp_c = 0.0f;
    uint32_t age_ms = 0;
    if (!app_get_latest_temp(&temp_c, &age_ms)) {
        ESP_LOGW(TAG, "[UART_TX_TEMP] no temperature sample available yet");
        return;
    }

    int32_t centi = (int32_t)lroundf((double)temp_c * 100.0);
    if (centi > INT16_MAX) {
        centi = INT16_MAX;
    } else if (centi < INT16_MIN) {
        centi = INT16_MIN;
    }
    int16_t temp_centi = (int16_t)centi;

    app_vision_event_uart_send_payload_v1(
        UART_V1_MSG_TEMPERATURE,
        0,
        (uint8_t)(temp_centi & 0xFF),
        (uint8_t)(((uint16_t)temp_centi >> 8) & 0xFF),
        0,
        0
    );

    ESP_LOGI(TAG, "[UART_TX_TEMP] temp=%.2f C centi=%d age_ms=%lu",
             (double)temp_c,
             (int)temp_centi,
             (unsigned long)age_ms);
}

static void vision_event_task(void *arg)
{
    (void)arg;
    uint32_t last_temp_tx_ms = esp_log_timestamp();
    bool temp_sent_once = false;

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
        uint32_t now_ms = esp_log_timestamp();
        float latest_temp_c = 0.0f;
        if (app_get_latest_temp(&latest_temp_c, NULL) &&
            (!temp_sent_once || (uint32_t)(now_ms - last_temp_tx_ms) >= UART_TEMP_TX_PERIOD_MS)) {
            app_uart_send_temperature_payload_v1();
            last_temp_tx_ms = now_ms;
            temp_sent_once = true;
        }
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

    c6_enable_if_configured();
    vTaskDelay(pdMS_TO_TICKS(1200));
#if HCI_UART_SANITY_TEST_MODE
    hci_uart_sanity_test();
#endif

#if HCI_UART_RX_ONLY_TEST_MODE
    ESP_LOGW(TAG, "UART RX-only test mode active: BLE client init skipped");
#else
#if C6_UART_TEMP_RX_MODE
    ESP_LOGI(TAG, "Initialize C6 UART RX stream (no BLE client)");
    c6_uart_rx_init();
    BaseType_t c6_rx_task_ok = xTaskCreatePinnedToCore(
        c6_uart_rx_task,
        "c6_uart_rx",
        4 * 1024,
        NULL,
        5,
        NULL,
        0
    );
    ESP_ERROR_CHECK(c6_rx_task_ok == pdPASS ? ESP_OK : ESP_FAIL);
#else
    ESP_LOGI(TAG, "Initialize BLE temperature client (HCI UART RX=GPIO34 TX=GPIO7)");
    esp_err_t ble_ret = app_ble_temp_client_start();
    if (ble_ret != ESP_OK) {
        ESP_LOGW(TAG, "BLE temperature client start failed: %s", esp_err_to_name(ble_ret));
    }
#endif
#endif

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
