#include "app_ble_temp_client.h"

#include "esp_log.h"

static const char *TAG = "ble_temp_client";

// BLE module temporarily disabled so the P4 app can focus on UART-only behavior.
#if 0

#include <math.h>
#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_hs_adv.h"
#include "host/ble_hs_mbuf.h"
#include "host/ble_store.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

// UUIDs shared with the ESP32-C6 temperature sensor.
static const ble_uuid128_t BLE_TEMP_SERVICE_UUID =
    BLE_UUID128_INIT(0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);
static const ble_uuid128_t BLE_TEMP_CHARACTERISTIC_UUID =
    BLE_UUID128_INIT(0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
                     0x78, 0x56, 0x34, 0x12, 0x78, 0x56, 0x34, 0x12);

#define BLE_TEMP_PAYLOAD_LEN 2

static bool s_ble_started = false;
static uint8_t s_own_addr_type = BLE_OWN_ADDR_PUBLIC;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_svc_start_handle = 0;
static uint16_t s_svc_end_handle = 0;
static uint16_t s_chr_val_handle = 0;
static uint16_t s_cccd_handle = 0;
static float s_last_temp_c = NAN;
static uint32_t s_last_temp_ms = 0;

void ble_store_config_init(void);

static void ble_temp_start_scan(void);
static int ble_temp_gap_event(struct ble_gap_event *event, void *arg);

static void ble_temp_reset_peer_state(void)
{
    s_svc_start_handle = 0;
    s_svc_end_handle = 0;
    s_chr_val_handle = 0;
    s_cccd_handle = 0;
}

static bool ble_temp_adv_matches(const struct ble_hs_adv_fields *fields)
{
    for (int i = 0; i < fields->num_uuids128; ++i) {
        if (ble_uuid_cmp(&fields->uuids128[i].u, &BLE_TEMP_SERVICE_UUID.u) == 0) {
            return true;
        }
    }

    return false;
}

static bool ble_temp_decode_payload(const struct os_mbuf *om, float *out_temp_c)
{
    uint8_t raw[BLE_TEMP_PAYLOAD_LEN] = {0};
    if (om == NULL || out_temp_c == NULL) {
        return false;
    }

    int rc = ble_hs_mbuf_to_flat(om, raw, sizeof(raw), NULL);
    if (rc != 0) {
        return false;
    }

    int16_t centi_c = (int16_t)((uint16_t)raw[0] | ((uint16_t)raw[1] << 8));
    *out_temp_c = (float)centi_c / 100.0f;
    return true;
}

static void ble_temp_store_and_log(const struct os_mbuf *om, const char *source)
{
    float temp_c = NAN;
    if (!ble_temp_decode_payload(om, &temp_c)) {
        ESP_LOGW(TAG, "Unable to decode temperature payload from %s", source);
        return;
    }

    s_last_temp_c = temp_c;
    s_last_temp_ms = esp_log_timestamp();
    ESP_LOGI(TAG, "[BLE_TEMP] source=%s temp=%.2f C", source, (double)temp_c);
}

static int ble_temp_on_read(uint16_t conn_handle,
                            const struct ble_gatt_error *error,
                            struct ble_gatt_attr *attr,
                            void *arg)
{
    (void)arg;
    if (error->status == 0) {
        ble_temp_store_and_log(attr->om, "read");
        return 0;
    }

    ESP_LOGW(TAG, "Temperature read failed; status=%d conn=%u",
             error->status,
             (unsigned)conn_handle);
    return 0;
}

static void ble_temp_read_once(uint16_t conn_handle)
{
    if (s_chr_val_handle == 0) {
        return;
    }

    int rc = ble_gattc_read(conn_handle, s_chr_val_handle, ble_temp_on_read, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "Failed to trigger temperature read; rc=%d", rc);
    }
}

static int ble_temp_on_cccd_write(uint16_t conn_handle,
                                  const struct ble_gatt_error *error,
                                  struct ble_gatt_attr *attr,
                                  void *arg)
{
    (void)attr;
    (void)arg;

    if (error->status == 0) {
        ESP_LOGI(TAG, "Temperature notifications enabled");
    } else {
        ESP_LOGW(TAG, "Failed to enable notifications; status=%d", error->status);
    }

    ble_temp_read_once(conn_handle);
    return 0;
}

static int ble_temp_on_dsc_disc(uint16_t conn_handle,
                                const struct ble_gatt_error *error,
                                uint16_t chr_val_handle,
                                const struct ble_gatt_dsc *dsc,
                                void *arg)
{
    (void)arg;

    if (error->status == 0) {
        if (chr_val_handle == s_chr_val_handle &&
            ble_uuid_u16(&dsc->uuid.u) == BLE_GATT_DSC_CLT_CFG_UUID16) {
            s_cccd_handle = dsc->handle;
        }
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
        if (s_cccd_handle != 0) {
            const uint8_t notify_enable[2] = {0x01, 0x00};
            int rc = ble_gattc_write_flat(conn_handle,
                                          s_cccd_handle,
                                          notify_enable,
                                          sizeof(notify_enable),
                                          ble_temp_on_cccd_write,
                                          NULL);
            if (rc == 0) {
                return 0;
            }
            ESP_LOGW(TAG, "Failed to write CCCD; rc=%d", rc);
        }

        // Fallback: even without notifications, read once.
        ble_temp_read_once(conn_handle);
        return 0;
    }

    ESP_LOGW(TAG, "Descriptor discovery failed; status=%d", error->status);
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    return 0;
}

static int ble_temp_on_chr_disc(uint16_t conn_handle,
                                const struct ble_gatt_error *error,
                                const struct ble_gatt_chr *chr,
                                void *arg)
{
    (void)arg;

    if (error->status == 0) {
        s_chr_val_handle = chr->val_handle;
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
        if (s_chr_val_handle == 0) {
            ESP_LOGW(TAG, "Temperature characteristic not found");
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }

        int rc = ble_gattc_disc_all_dscs(conn_handle,
                                         s_chr_val_handle,
                                         s_svc_end_handle,
                                         ble_temp_on_dsc_disc,
                                         NULL);
        if (rc != 0) {
            ESP_LOGW(TAG, "Failed to discover descriptors; rc=%d", rc);
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;
    }

    ESP_LOGW(TAG, "Characteristic discovery failed; status=%d", error->status);
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    return 0;
}

static int ble_temp_on_svc_disc(uint16_t conn_handle,
                                const struct ble_gatt_error *error,
                                const struct ble_gatt_svc *service,
                                void *arg)
{
    (void)arg;

    if (error->status == 0) {
        s_svc_start_handle = service->start_handle;
        s_svc_end_handle = service->end_handle;
        return 0;
    }

    if (error->status == BLE_HS_EDONE) {
        if (s_svc_start_handle == 0 || s_svc_end_handle == 0) {
            ESP_LOGW(TAG, "Temperature service not found");
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
            return 0;
        }

        int rc = ble_gattc_disc_chrs_by_uuid(conn_handle,
                                             s_svc_start_handle,
                                             s_svc_end_handle,
                                             &BLE_TEMP_CHARACTERISTIC_UUID.u,
                                             ble_temp_on_chr_disc,
                                             NULL);
        if (rc != 0) {
            ESP_LOGW(TAG, "Failed to discover temperature characteristic; rc=%d", rc);
            ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;
    }

    ESP_LOGW(TAG, "Service discovery failed; status=%d", error->status);
    ble_gap_terminate(conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    return 0;
}

static void ble_temp_connect(const ble_addr_t *addr)
{
    int rc = ble_gap_disc_cancel();
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "Failed to stop scan before connect; rc=%d", rc);
    }

    rc = ble_gap_connect(s_own_addr_type, addr, 30000, NULL, ble_temp_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "BLE connect failed; rc=%d", rc);
        ble_temp_start_scan();
    }
}

static void ble_temp_start_scan(void)
{
    struct ble_gap_disc_params disc_params = {0};
    disc_params.filter_duplicates = 1;
    disc_params.passive = 1;

    int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &disc_params, ble_temp_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "BLE scan start failed; rc=%d", rc);
    } else {
        ESP_LOGI(TAG, "Scanning for C6 temperature sensor...");
    }
}

static int ble_temp_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC: {
        struct ble_hs_adv_fields fields;
        if (ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data) != 0) {
            return 0;
        }

        if (ble_temp_adv_matches(&fields)) {
            ESP_LOGI(TAG, "Temperature sensor advertisement found");
            ble_temp_connect(&event->disc.addr);
        }
        return 0;
    }

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "BLE connect failed; status=%d", event->connect.status);
            ble_temp_start_scan();
            return 0;
        }

        s_conn_handle = event->connect.conn_handle;
        ble_temp_reset_peer_state();
        ESP_LOGI(TAG, "BLE connected, discovering temperature service");

        if (ble_gattc_disc_svc_by_uuid(s_conn_handle,
                                       &BLE_TEMP_SERVICE_UUID.u,
                                       ble_temp_on_svc_disc,
                                       NULL) != 0) {
            ESP_LOGW(TAG, "Unable to start service discovery");
            ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "BLE disconnected; reason=%d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        ble_temp_reset_peer_state();
        ble_temp_start_scan();
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
        if (event->notify_rx.attr_handle == s_chr_val_handle && event->notify_rx.om != NULL) {
            ble_temp_store_and_log(event->notify_rx.om, "notify");
        }
        return 0;

    case BLE_GAP_EVENT_DISC_COMPLETE:
        if (s_conn_handle == BLE_HS_CONN_HANDLE_NONE) {
            ble_temp_start_scan();
        }
        return 0;

    default:
        return 0;
    }
}

static void ble_temp_on_reset(int reason)
{
    ESP_LOGE(TAG, "NimBLE reset; reason=%d", reason);
}

static void ble_temp_on_sync(void)
{
    int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error determining BLE address type; rc=%d", rc);
        return;
    }

    ble_temp_start_scan();
}

static void ble_temp_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

esp_err_t app_ble_temp_client_start(void)
{
    if (s_ble_started) {
        return ESP_OK;
    }

    int rc = nimble_port_init();
    if (rc != ESP_OK) {
        ESP_LOGE(TAG, "nimble_port_init failed: %d", rc);
        return ESP_FAIL;
    }

    ble_hs_cfg.reset_cb = ble_temp_on_reset;
    ble_hs_cfg.sync_cb = ble_temp_on_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set("P4-BLE-TEMP-CLIENT");
    ble_store_config_init();

    nimble_port_freertos_init(ble_temp_host_task);
    s_ble_started = true;
    ESP_LOGI(TAG, "BLE temperature client started");
    return ESP_OK;
}

bool app_ble_temp_client_get_latest(float *out_temp_c, uint32_t *out_age_ms)
{
    if (isnan(s_last_temp_c)) {
        return false;
    }

    if (out_temp_c != NULL) {
        *out_temp_c = s_last_temp_c;
    }
    if (out_age_ms != NULL) {
        *out_age_ms = esp_log_timestamp() - s_last_temp_ms;
    }
    return true;
}

#endif

esp_err_t app_ble_temp_client_start(void)
{
    ESP_LOGW(TAG, "BLE code commented out: app_ble_temp_client_start skipped");
    return ESP_ERR_NOT_SUPPORTED;
}

bool app_ble_temp_client_get_latest(float *out_temp_c, uint32_t *out_age_ms)
{
    (void)out_temp_c;
    (void)out_age_ms;
    return false;
}
