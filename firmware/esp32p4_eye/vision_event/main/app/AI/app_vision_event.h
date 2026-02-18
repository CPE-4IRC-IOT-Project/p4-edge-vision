#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t app_vision_event_start(i2c_master_bus_handle_t i2c_handle);
void app_vision_event_uart_send_payload_v1(uint8_t msg_type,
                                           uint8_t flags,
                                           uint8_t luma,
                                           uint8_t occupied,
                                           uint8_t stable_count,
                                           uint8_t raw_count);

#ifdef __cplusplus
}
#endif
