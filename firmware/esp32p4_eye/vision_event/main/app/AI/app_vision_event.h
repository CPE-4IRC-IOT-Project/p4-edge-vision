#pragma once

#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t app_vision_event_start(i2c_master_bus_handle_t i2c_handle);

#ifdef __cplusplus
}
#endif
