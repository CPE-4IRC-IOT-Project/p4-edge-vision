#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t app_ble_temp_client_start(void);
bool app_ble_temp_client_get_latest(float *out_temp_c, uint32_t *out_age_ms);

#ifdef __cplusplus
}
#endif
