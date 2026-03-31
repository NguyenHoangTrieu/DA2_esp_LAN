/**
 * @file ble_gatt_handler.c
 * @brief BLE GATT Central — GAP + GATTC stack init and event callbacks.
 *
 * Implements the full BLE GATT Central role using ESP-IDF Bluedroid APIs.
 * Manages a 8-slot device table for scanned and connected devices.
 * All operation results are forwarded to WAN MCU via ble_gatt_uplink.
 *
 * Coexistence note: This module uses esp_ble_gap_register_callback() and
 * esp_ble_gattc_register_callback() which are separate from the BLE Mesh
 * stack path.  BLE Mesh and GATT Central can coexist, but running both
 * CFBN:SCAN and CFBG:SCAN simultaneously reduces BLE airtime efficiency.
 * Recommend serializing scans when both stacks are active.
 *
 * sdkconfig additions required:
 *   CONFIG_BT_BLUEDROID_ENABLED=y    (already set for BLE Mesh)
 *   CONFIG_BT_BLE_42_FEATURES_SUPPORTED=y
 *   CONFIG_BT_GATTC_ENABLE=y
 */

#include "ble_gatt_handler.h"
#include "ble_gatt_config.h"
#include "ble_gatt_uplink.h"
#include "ble_gatt_downlink.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "ble_gatt_hdl";

#define GATTC_APP_ID   0x55   /* Unique app registration ID for GATT Central */

/* MAC address formatting macros */
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"

/* --------------------------------------------------------------------------
 * Module state
 * -------------------------------------------------------------------------- */

static bool          s_initialized    = false;
static bool          s_bt_registered  = false; /* GAP/GATTC callbacks + app_register done */
static esp_gatt_if_t s_gattc_if       = ESP_GATT_IF_NONE;
static ble_gatt_device_t *s_devices   = NULL;  /* PSRAM-allocated device table */

/* Stack id for the currently pending scan or connect (single-threaded ops) */
static volatile uint8_t s_pending_stack_id = 0;

/* --------------------------------------------------------------------------
 * Internal helpers
 * -------------------------------------------------------------------------- */

static int find_free_slot(void) {
    for (int i = 0; i < BLE_GATT_MAX_DEVICES; i++) {
        if (!s_devices[i].valid) return i;
    }
    return -1;
}

static int find_by_addr(const uint8_t *addr) {
    for (int i = 0; i < BLE_GATT_MAX_DEVICES; i++) {
        if (s_devices[i].valid &&
            memcmp(s_devices[i].addr, addr, ESP_BD_ADDR_LEN) == 0) {
            return i;
        }
    }
    return -1;
}

static int find_by_conn_id(uint16_t conn_id) {
    for (int i = 0; i < BLE_GATT_MAX_DEVICES; i++) {
        if (s_devices[i].valid && s_devices[i].conn_id == conn_id) return i;
    }
    return -1;
}

void ble_gatt_handler_clear_devices(void) {
    for (int i = 0; i < BLE_GATT_MAX_DEVICES; i++) {
        memset(&s_devices[i], 0, sizeof(s_devices[i]));
    }
}

/* --------------------------------------------------------------------------
 * Scan-done reporter — builds consolidated uplink for the RPC response
 * -------------------------------------------------------------------------- */
static void send_scan_done(void) {
    int scan_count = 0;
    for (int i = 0; i < BLE_GATT_MAX_DEVICES; i++) {
        if (s_devices[i].valid) scan_count++;
    }
    ESP_LOGI(TAG, "Scan done: %d device(s)", scan_count);

    char *buf = heap_caps_malloc(2048, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        ble_gatt_uplink_send_ok(s_pending_stack_id, "SCAN_DONE:0");
        return;
    }
    int pos = snprintf(buf, 2048, "SCAN_DONE:%d", scan_count);
    for (int i = 0; i < BLE_GATT_MAX_DEVICES && pos + 70 < 2048; i++) {
        if (!s_devices[i].valid) continue;
        char name[21];
        strncpy(name, s_devices[i].name, 20);
        name[20] = '\0';
        pos += snprintf(buf + pos, 2048 - pos,
                        "\x1ESCAN_RESULT:%d," MACSTR ",%d,%s",
                        i, MAC2STR(s_devices[i].addr),
                        (int)s_devices[i].rssi, name);
    }
    ble_gatt_uplink_send_ok(s_pending_stack_id, buf);
    free(buf);
}

/** Extract device name from advertising data. Returns false if not found. */
static bool extract_adv_name(const uint8_t *adv_data, uint8_t adv_data_len,
                               char *name_out, size_t name_max) {
    uint8_t idx = 0;
    while (idx < adv_data_len) {
        uint8_t len  = adv_data[idx];
        if (len == 0) break;
        if (idx + len >= adv_data_len) break;
        uint8_t type = adv_data[idx + 1];
        /* 0x08 = Shortened Local Name, 0x09 = Complete Local Name */
        if ((type == 0x08 || type == 0x09) && len > 1) {
            size_t copy_len = len - 1;
            if (copy_len >= name_max) copy_len = name_max - 1;
            memcpy(name_out, &adv_data[idx + 2], copy_len);
            name_out[copy_len] = '\0';
            return true;
        }
        idx += len + 1;
    }
    return false;
}

/* --------------------------------------------------------------------------
 * GAP Event Callback
 * -------------------------------------------------------------------------- */

static void gap_event_cb(esp_gap_ble_cb_event_t event,
                          esp_ble_gap_cb_param_t *param)
{
    switch (event) {

    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT:
        if (param->scan_param_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            /* Start scanning after params are set */
            /* Duration is set by the downlink task via esp_ble_gap_start_scanning */
        }
        break;

    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        if (param->scan_start_cmpl.status == ESP_BT_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Scan started successfully");
        } else {
            ESP_LOGE(TAG, "Scan start failed: %d",
                     param->scan_start_cmpl.status);
            ble_gatt_uplink_send_fail(s_pending_stack_id, "SCAN_START_FAILED");
        }
        break;

    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        /* Fired only when esp_ble_gap_stop_scanning() is called explicitly (e.g. STOP command) */
        send_scan_done();
        break;

    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan = param;

        /* Duration-based scan expired — this is the normal completion path */
        if (scan->scan_rst.search_evt == ESP_GAP_SEARCH_INQ_CMPL_EVT) {
            send_scan_done();
            break;
        }

        if (scan->scan_rst.search_evt != ESP_GAP_SEARCH_INQ_RES_EVT) break;

        const uint8_t *bda = scan->scan_rst.bda;

        /* Skip non-connectable (e.g., BLE Mesh ADV_NONCONN_IND) */
        if (scan->scan_rst.ble_evt_type == ESP_BLE_EVT_NON_CONN_ADV) break;

        /* Deduplication — skip if already in table */
        if (find_by_addr(bda) >= 0) break;

        int slot = find_free_slot();
        if (slot < 0) {
            ESP_LOGW(TAG, "Device table full, ignoring " MACSTR, MAC2STR(bda));
            break;
        }

        ble_gatt_device_t *dev = &s_devices[slot];
        memset(dev, 0, sizeof(*dev));
        memcpy(dev->addr, bda, ESP_BD_ADDR_LEN);
        dev->addr_type = scan->scan_rst.ble_addr_type;
        dev->rssi      = scan->scan_rst.rssi;
        dev->conn_id   = 0xFFFF;
        dev->gattc_if  = ESP_GATT_IF_NONE;
        dev->stack_id  = s_pending_stack_id;
        dev->valid     = true;

        /* Try to extract device name from advertising data */
        if (!extract_adv_name(scan->scan_rst.ble_adv,
                               scan->scan_rst.adv_data_len,
                               dev->name, BLE_GATT_DEV_NAME_LEN)) {
            snprintf(dev->name, BLE_GATT_DEV_NAME_LEN, "Unknown");
        }

        ESP_LOGI(TAG, "Scan[%d]: " MACSTR " RSSI=%d Name='%s'",
                 slot, MAC2STR(bda), (int)dev->rssi, dev->name);
        break;
    }

    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGD(TAG, "Connection params updated: interval=%u latency=%u timeout=%u",
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;

    default:
        break;
    }
}

/* --------------------------------------------------------------------------
 * GATTC Event Callback
 * -------------------------------------------------------------------------- */

static void gattc_event_cb(esp_gattc_cb_event_t event,
                            esp_gatt_if_t gattc_if,
                            esp_ble_gattc_cb_param_t *param)
{
    switch (event) {

    case ESP_GATTC_REG_EVT:
        if (param->reg.status == ESP_GATT_OK && param->reg.app_id == GATTC_APP_ID) {
            s_gattc_if = gattc_if;
            ESP_LOGI(TAG, "GATTC registered: if=%d", gattc_if);
        } else {
            ESP_LOGE(TAG, "GATTC register failed: status=%d app_id=%d",
                     param->reg.status, param->reg.app_id);
        }
        break;

    case ESP_GATTC_OPEN_EVT: {
        int idx = find_by_addr(param->open.remote_bda);
        if (idx < 0) {
            ESP_LOGW(TAG, "OPEN_EVT: unknown device");
            break;
        }
        ble_gatt_device_t *dev = &s_devices[idx];
        if (param->open.status != ESP_GATT_OK) {
            ESP_LOGE(TAG, "Connect failed: status=%d", param->open.status);
            char fail[48];
            snprintf(fail, sizeof(fail), "CONNECT:FAILED:%d", param->open.status);
            ble_gatt_uplink_send_fail(dev->stack_id, fail);
            dev->valid = false;
            break;
        }
        dev->conn_id  = param->open.conn_id;
        dev->gattc_if = gattc_if;

        char ok[80];
        snprintf(ok, sizeof(ok), "CONNECTED:%d:0x%04X:" MACSTR,
                 idx, dev->conn_id, MAC2STR(dev->addr));
        ble_gatt_uplink_send_ok(dev->stack_id, ok);

        /* Request MTU negotiation (optional, up to 512 bytes) */
        esp_ble_gattc_send_mtu_req(gattc_if, dev->conn_id);
        break;
    }

    case ESP_GATTC_CFG_MTU_EVT:
        ESP_LOGD(TAG, "MTU configured: %u", param->cfg_mtu.mtu);
        break;

    case ESP_GATTC_DISCONNECT_EVT: {
        int idx = find_by_conn_id(param->disconnect.conn_id);
        if (idx < 0) break;
        ble_gatt_device_t *dev = &s_devices[idx];
        char ok[48];
        snprintf(ok, sizeof(ok), "DISCONNECTED:%d:0x%04X", idx, dev->conn_id);
        ble_gatt_uplink_send_ok(dev->stack_id, ok);
        dev->conn_id  = 0xFFFF;
        dev->gattc_if = ESP_GATT_IF_NONE;
        break;
    }

    case ESP_GATTC_SEARCH_RES_EVT: {
        int idx = find_by_conn_id(param->search_res.conn_id);
        if (idx < 0) break;

        ble_gatt_device_t *dev = &s_devices[idx];
        uint8_t svc_idx = dev->num_services;
        if (svc_idx < BLE_GATT_MAX_SERVICES) {
            dev->services[svc_idx].start_handle = param->search_res.start_handle;
            dev->services[svc_idx].end_handle   = param->search_res.end_handle;
            if (param->search_res.srvc_id.uuid.len == ESP_UUID_LEN_16) {
                dev->services[svc_idx].uuid16 =
                    param->search_res.srvc_id.uuid.uuid.uuid16;
            } else {
                memcpy(dev->services[svc_idx].uuid128,
                       param->search_res.srvc_id.uuid.uuid.uuid128, 16);
            }
            dev->services[svc_idx].valid = true;
            dev->num_services++;
        }
        /* SERVICE lines are NOT sent individually here.
         * They are included in the batched DISC_DONE response at SEARCH_CMPL_EVT
         * so the WAN MCU sends a single complete packet to the server. */
        ESP_LOGD(TAG, "Service found: idx=%d svc#%u", idx, svc_idx);
        break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
        int idx = find_by_conn_id(param->search_cmpl.conn_id);
        if (idx < 0) break;
        ble_gatt_device_t *dev = &s_devices[idx];

        /* After service discovery, enumerate all characteristics */
        dev->num_chars = 0;
        for (int si = 0; si < dev->num_services; si++) {
            uint16_t cnt = BLE_GATT_MAX_CHARS - dev->num_chars;
            esp_gattc_char_elem_t char_buf[BLE_GATT_MAX_CHARS];
            esp_gatt_status_t ret = esp_ble_gattc_get_all_char(
                gattc_if, dev->conn_id,
                dev->services[si].start_handle,
                dev->services[si].end_handle,
                char_buf, &cnt, 0);

            if (ret == ESP_GATT_OK) {
                for (uint16_t ci = 0; ci < cnt && dev->num_chars < BLE_GATT_MAX_CHARS; ci++) {
                    ble_gatt_char_entry_t *ce =
                        &dev->chars[dev->num_chars++];
                    ce->handle     = char_buf[ci].char_handle;
                    ce->properties = char_buf[ci].properties;
                    if (char_buf[ci].uuid.len == ESP_UUID_LEN_16) {
                        ce->uuid16 = char_buf[ci].uuid.uuid.uuid16;
                    } else {
                        memcpy(ce->uuid128, char_buf[ci].uuid.uuid.uuid128, 16);
                    }
                    ce->valid = true;
                }
            }
        }

        /* Build one batched DISC_DONE response so the RPC caller sees all services and chars */
        char *batch = heap_caps_malloc(2048, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!batch) {
            char small[48];
            snprintf(small, sizeof(small), "DISC_DONE:%d:%u_CHARS", idx, dev->num_chars);
            ble_gatt_uplink_send_ok(dev->stack_id, small);
            break;
        }
        int pos = snprintf(batch, 2048, "DISC_DONE:%d:%u_CHARS", idx, dev->num_chars);
        /* Append all SERVICE lines first */
        for (uint8_t si = 0; si < dev->num_services && pos + 60 < 2048; si++) {
            ble_gatt_svc_entry_t *se = &dev->services[si];
            if (!se->valid) continue;
            if (se->uuid16) {
                pos += snprintf(batch + pos, 2048 - pos,
                                "\x1E" "SERVICE:%d:0x%04X:0x%04X:0x%04X",
                                idx, se->uuid16, se->start_handle, se->end_handle);
            } else {
                pos += snprintf(batch + pos, 2048 - pos,
                                "\x1E" "SERVICE:%d:128-BIT:0x%04X:0x%04X",
                                idx, se->start_handle, se->end_handle);
            }
        }
        /* Append all CHAR lines */
        for (uint8_t ci = 0; ci < dev->num_chars && pos + 60 < 2048; ci++) {
            ble_gatt_char_entry_t *ce = &dev->chars[ci];
            if (!ce->valid) continue;
            if (ce->uuid16) {
                pos += snprintf(batch + pos, 2048 - pos,
                                "\x1E" "CHAR:%d:0x%04X:0x%04X:0x%02X",
                                idx, ce->uuid16, ce->handle, ce->properties);
            } else {
                pos += snprintf(batch + pos, 2048 - pos,
                                "\x1E" "CHAR:%d:128-BIT:0x%04X:0x%02X",
                                idx, ce->handle, ce->properties);
            }
        }
        ble_gatt_uplink_send_ok(dev->stack_id, batch);
        heap_caps_free(batch);
        break;
    }

    case ESP_GATTC_READ_CHAR_EVT: {
        int idx = find_by_conn_id(param->read.conn_id);
        if (idx < 0) break;
        ble_gatt_device_t *dev = &s_devices[idx];

        if (param->read.status != ESP_GATT_OK) {
            char fail[48];
            snprintf(fail, sizeof(fail), "READ:FAILED:0x%04X:%d",
                     param->read.handle, param->read.status);
            ble_gatt_uplink_send_fail(dev->stack_id, fail);
            break;
        }

        /* Encode value as hex string */
        char hex[256] = {0};
        uint16_t copy_len = param->read.value_len;
        if (copy_len > 64) copy_len = 64;   /* cap at 128 hex chars */
        for (uint16_t i = 0; i < copy_len; i++) {
            snprintf(&hex[i * 2], 3, "%02X", param->read.value[i]);
        }
        char ok[320];
        snprintf(ok, sizeof(ok), "READ:%d:0x%04X:%s",
                 idx, param->read.handle, hex);
        ble_gatt_uplink_send_ok(dev->stack_id, ok);
        break;
    }

    case ESP_GATTC_WRITE_CHAR_EVT: {
        int idx = find_by_conn_id(param->write.conn_id);
        if (idx < 0) break;
        ble_gatt_device_t *dev = &s_devices[idx];

        if (param->write.status != ESP_GATT_OK) {
            char fail[48];
            snprintf(fail, sizeof(fail), "WRITE:FAILED:0x%04X:%d",
                     param->write.handle, param->write.status);
            ble_gatt_uplink_send_fail(dev->stack_id, fail);
        } else {
            char ok[48];
            snprintf(ok, sizeof(ok), "WRITE_OK:%d:0x%04X", idx, param->write.handle);
            ble_gatt_uplink_send_ok(dev->stack_id, ok);
        }
        break;
    }

    case ESP_GATTC_WRITE_DESCR_EVT: {
        int idx = find_by_conn_id(param->write.conn_id);
        if (idx < 0) break;
        ble_gatt_device_t *dev = &s_devices[idx];

        if (param->write.status != ESP_GATT_OK) {
            char fail[56];
            snprintf(fail, sizeof(fail), "DESCR_WRITE:FAILED:0x%04X:%d",
                     param->write.handle, param->write.status);
            ble_gatt_uplink_send_fail(dev->stack_id, fail);
        } else {
            char ok[48];
            snprintf(ok, sizeof(ok), "DESCR_WRITE_OK:%d:0x%04X",
                     idx, param->write.handle);
            ble_gatt_uplink_send_ok(dev->stack_id, ok);
        }
        break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
        int idx = find_by_conn_id(param->notify.conn_id);
        if (idx < 0) break;
        ble_gatt_device_t *dev = &s_devices[idx];

        const char *type = param->notify.is_notify ? "NOTIFY" : "INDICATE";
        /* Encode first 64 bytes as hex */
        char hex[256] = {0};
        uint16_t copy_len = param->notify.value_len;
        if (copy_len > 64) copy_len = 64;
        for (uint16_t i = 0; i < copy_len; i++) {
            snprintf(&hex[i * 2], 3, "%02X", param->notify.value[i]);
        }
        char ok[320];
        snprintf(ok, sizeof(ok), "%s:%d:0x%04X:%s",
                 type, idx, param->notify.handle, hex);
        ble_gatt_uplink_send_ok(dev->stack_id, ok);
        break;
    }

    default:
        break;
    }
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

esp_err_t ble_gatt_handler_init(void) {
    if (s_initialized) return ESP_OK;

    /* Allocate (or re-zero) device table from PSRAM */
    if (!s_devices) {
        s_devices = heap_caps_calloc(BLE_GATT_MAX_DEVICES, sizeof(ble_gatt_device_t),
                                     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_devices) {
            ESP_LOGE(TAG, "Failed to alloc device table from PSRAM");
            return ESP_ERR_NO_MEM;
        }
    } else {
        memset(s_devices, 0, BLE_GATT_MAX_DEVICES * sizeof(ble_gatt_device_t));
    }
    for (int i = 0; i < BLE_GATT_MAX_DEVICES; i++) {
        s_devices[i].conn_id  = 0xFFFF;
        s_devices[i].gattc_if = ESP_GATT_IF_NONE;
    }

    esp_err_t ret;

    /* Initialize BT controller and Bluedroid stack if not already done.
     * ble_native_handler_init() handles this automatically via esp_ble_mesh_init().
     * For GATT-only scenarios (no BLE Mesh), we must do it manually here. */
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "BT controller init failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "BT controller enable failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }
    if (esp_bluedroid_get_status() == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        ret = esp_bluedroid_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Bluedroid init failed: %s", esp_err_to_name(ret));
            return ret;
        }
        ret = esp_bluedroid_enable();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret));
            return ret;
        }
    }

    /* Register GAP and GATTC callbacks — only once, even across retries.
     * GAP may fail if BLE Native already claimed it; that is non-fatal.
     * GATTC app_register creates a new interface slot each call, so we
     * MUST NOT call it again after the first successful registration. */
    bool gap_available = true;

    if (!s_bt_registered) {
        ret = esp_ble_gap_register_callback(gap_event_cb);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "GAP register callback failed: %s (likely BLE Native conflict)",
                     esp_err_to_name(ret));
            gap_available = false;
        }

        ret = esp_ble_gattc_register_callback(gattc_event_cb);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "GATTC register callback failed: %s", esp_err_to_name(ret));
            return ret;
        }

        ret = esp_ble_gattc_app_register(GATTC_APP_ID);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "GATTC app register failed: %s", esp_err_to_name(ret));
            return ret;
        }

        s_bt_registered = true;
        if (!gap_available) {
            ESP_LOGW(TAG, "GAP unavailable (BLE Native conflict) — scan/connect disabled");
        }
    }

    /* Start uplink/downlink tasks (always start these, even if GAP unavailable) */
    ret = ble_gatt_uplink_task_start();
    if (ret != ESP_OK) return ret;

    ret = ble_gatt_downlink_task_start();
    if (ret != ESP_OK) return ret;

    s_initialized = true;
    ESP_LOGI(TAG, "BLE GATT Central initialized (CFBG: prefix ready)");
    return ESP_OK;
}

esp_err_t ble_gatt_handler_load_config(uint8_t stack_id,
                                        const char *json_str,
                                        uint16_t json_len)
{
    esp_err_t ret = ble_gatt_config_load(stack_id, json_str, json_len);
    if (ret == ESP_OK) {
        char ok[32];
        snprintf(ok, sizeof(ok), "JSON_LOADED");
        ble_gatt_uplink_send_ok(stack_id, ok);
    } else {
        ble_gatt_uplink_send_fail(stack_id, "JSON_PARSE_FAIL");
    }
    return ret;
}

esp_err_t ble_gatt_handler_execute(const uint8_t *data, uint16_t len) {
    return ble_gatt_downlink_enqueue(data, len);
}

esp_err_t ble_gatt_handler_find_by_addr(const uint8_t *addr, uint8_t *idx) {
    int i = find_by_addr(addr);
    if (i < 0) return ESP_ERR_NOT_FOUND;
    *idx = (uint8_t)i;
    return ESP_OK;
}

esp_err_t ble_gatt_handler_find_by_conn(uint16_t conn_id, uint8_t *idx) {
    int i = find_by_conn_id(conn_id);
    if (i < 0) return ESP_ERR_NOT_FOUND;
    *idx = (uint8_t)i;
    return ESP_OK;
}

ble_gatt_device_t *ble_gatt_handler_get_device(uint8_t idx) {
    if (idx >= BLE_GATT_MAX_DEVICES) return NULL;
    return &s_devices[idx];
}

esp_gatt_if_t ble_gatt_handler_get_if(void) {
    return s_gattc_if;
}

void ble_gatt_handler_report_scan_result(uint8_t stack_id,
                                          const ble_gatt_device_t *dev,
                                          uint8_t dev_idx)
{
    if (!dev) return;
    char report[128];
    snprintf(report, sizeof(report),
             "SCAN:" MACSTR ",%d,0x%04X,%s",
             MAC2STR(dev->addr),
             (int)dev->rssi,
             dev->conn_id,
             dev->name);
    ble_gatt_uplink_send_ok(stack_id, report);
}

void ble_gatt_handler_set_pending_stack(uint8_t stack_id) {
    s_pending_stack_id = stack_id;
}
