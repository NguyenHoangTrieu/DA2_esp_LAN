/**
 * @file ble_gatt_downlink.c
 * @brief BLE GATT Central downlink — parse CFBG: commands and dispatch.
 *
 * Verb set mirrors the STM32WB55 AT command interface (README) so the same
 * server-side logic works for both the AT module path (CFBL:) and the native
 * ESP32-S3 GATT Central path (CFBG:).
 *
 * All operations that produce async results (CONNECT, DISC, READ, WRITE, NOTIFY)
 * emit an immediate acknowledgement ("INITIATED") followed by a final result
 * delivered via the GATTC/GAP event callbacks in ble_gatt_handler.c.
 */

#include "ble_gatt_downlink.h"
#include "ble_gatt_config.h"
#include "ble_gatt_uplink.h"
#include "ble_gatt_handler.h"
#include "config_ble_mode.h"
#include "esp_log.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/idf_additions.h"
#include "esp_heap_caps.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <ctype.h>

static const char *TAG = "ble_gatt_dn";

/* MAC address formatting macros */
#define MAC2STR(a) (a)[0], (a)[1], (a)[2], (a)[3], (a)[4], (a)[5]
#define MACSTR "%02x:%02x:%02x:%02x:%02x:%02x"

/* --------------------------------------------------------------------------
 * Internal types
 * -------------------------------------------------------------------------- */
typedef struct {
    uint8_t  raw[BLE_GATT_DOWNLINK_ITEM_MAX];
    uint16_t len;
} downlink_item_t;

static QueueHandle_t s_dn_queue     = NULL;
static TaskHandle_t  s_dn_task      = NULL;
static StackType_t   *s_dn_stack     = NULL;
static StaticTask_t  *s_dn_tcb       = NULL;
static volatile bool s_task_running = false;

/* --------------------------------------------------------------------------
 * Helpers
 * -------------------------------------------------------------------------- */

/** Parse hex string (e.g. "0x000E" or "000E") to uint16 */
static uint16_t parse_u16(const char *s) {
    if (!s) return 0;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) s += 2;
    return (uint16_t)strtoul(s, NULL, 16);
}

/** Decode hex string (e.g. "01020304") into bytes.  Returns byte count. */
static uint16_t hex_decode(const char *hex, uint8_t *out, uint16_t out_max) {
    uint16_t len = (uint16_t)(strlen(hex) / 2);
    if (len > out_max) len = out_max;
    for (uint16_t i = 0; i < len; i++) {
        char b[3] = { hex[i * 2], hex[i * 2 + 1], '\0' };
        out[i] = (uint8_t)strtoul(b, NULL, 16);
    }
    return len;
}

/** Parse ":"−separated tokens.  Modifies a copy — caller provides buffer. */
static int split_tokens(const char *src, char *buf, size_t buf_sz,
                         const char **tokens, int max_tokens)
{
    if (!src || !buf || !tokens || max_tokens <= 0) return 0;
    size_t slen = strlen(src);
    if (slen >= buf_sz) slen = buf_sz - 1;
    memcpy(buf, src, slen);
    buf[slen] = '\0';

    int n = 0;
    char *p = buf;
    while (n < max_tokens) {
        tokens[n++] = p;
        char *colon = strchr(p, ':');
        if (!colon) break;
        *colon = '\0';
        p = colon + 1;
    }
    return n;
}

/* --------------------------------------------------------------------------
 * Verb handlers
 * -------------------------------------------------------------------------- */

/* CFBG:<slot>:SCAN:<duration_ms> */
static void handle_scan(uint8_t stack_id, const char *params) {
    /* Reject if a scan is already running to avoid SCAN_START_FAILED from
     * Bluedroid when start_scanning() is called on a busy controller. */
    if (ble_gatt_handler_is_scan_active()) {
        ESP_LOGW(TAG, "Scan already in progress, rejecting new request");
        ble_gatt_uplink_send_fail(stack_id, "SCAN:BUSY");
        return;
    }

    ble_gatt_stack_config_t *cfg = ble_gatt_config_get(stack_id);

    uint32_t duration_sec = 10;
    if (params && params[0]) {
        unsigned long v = strtoul(params, NULL, 10);
        if (v > 0 && v <= 120000) {
            /* Convert ms → seconds (esp_ble_gap_start_scanning takes seconds) */
            duration_sec = (v + 999) / 1000;
        }
    }

    /* Configure scan params from JSON config */
    esp_ble_scan_params_t scan_params = {
        .scan_type          = cfg->scan.active
                                  ? BLE_SCAN_TYPE_ACTIVE
                                  : BLE_SCAN_TYPE_PASSIVE,
        .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
        .scan_filter_policy = BLE_SCAN_FILTER_ALLOW_ALL,
        .scan_interval      = cfg->scan.interval,
        .scan_window        = cfg->scan.window,
        .scan_duplicate     = BLE_SCAN_DUPLICATE_DISABLE,
    };

    esp_err_t ret = esp_ble_gap_set_scan_params(&scan_params);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "set_scan_params failed: %s", esp_err_to_name(ret));
        ble_gatt_uplink_send_fail(stack_id, "SCAN:PARAM_FAIL");
        return;
    }

    /* Clear stale device table entries so fresh scan starts empty */
    ble_gatt_handler_clear_devices();

    /* Set pending stack so GAP callbacks know which channel to use */
    ble_gatt_handler_set_pending_stack(stack_id);

    ret = esp_ble_gap_start_scanning((uint32_t)duration_sec);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "start_scanning failed: %s", esp_err_to_name(ret));
        ble_gatt_uplink_send_fail(stack_id, "SCAN:START_FAIL");
        return;
    }
    /* Results will be sent as a single batched SCAN_DONE from GAP SCAN_STOP_COMPLETE */
    ESP_LOGI(TAG, "Scanning for %us...", (unsigned)duration_sec);
}

/* CFBG:<slot>:STOP */
static void handle_stop(uint8_t stack_id) {
    ble_gatt_handler_set_scan_stop_requested();
    esp_err_t ret = esp_ble_gap_stop_scanning();
    if (ret != ESP_OK) {
        ble_gatt_uplink_send_fail(stack_id, "STOP:FAILED");
    } else {
        ble_gatt_uplink_send_ok(stack_id, "SCAN_STOPPED");
    }
}

/* CFBG:<slot>:LIST */
static void handle_list(uint8_t stack_id) {
    char line[128];
    int count = 0;
    for (int i = 0; i < BLE_GATT_MAX_DEVICES; i++) {
        ble_gatt_device_t *d = ble_gatt_handler_get_device((uint8_t)i);
        if (d && d->valid) count++;
    }
    snprintf(line, sizeof(line), "LIST:%d", count);
    ble_gatt_uplink_send_ok(stack_id, line);

    for (int i = 0; i < BLE_GATT_MAX_DEVICES; i++) {
        ble_gatt_device_t *d = ble_gatt_handler_get_device((uint8_t)i);
        if (!d || !d->valid) continue;
        snprintf(line, sizeof(line),
                 "DEV:%d," MACSTR ",%d,0x%04X,%s",
                 i, MAC2STR(d->addr), (int)d->rssi, d->conn_id, d->name);
        ble_gatt_uplink_send_ok(stack_id, line);
    }
}

/* CFBG:<slot>:CLEAR */
static void handle_clear(uint8_t stack_id) {
    for (int i = 0; i < BLE_GATT_MAX_DEVICES; i++) {
        ble_gatt_device_t *d = ble_gatt_handler_get_device((uint8_t)i);
        if (!d) continue;
        if (d->conn_id != 0xFFFF) continue; /* skip connected devices */
        memset(d, 0, sizeof(*d));
        d->conn_id  = 0xFFFF;
        d->gattc_if = ESP_GATT_IF_NONE;
    }
    ble_gatt_uplink_send_ok(stack_id, "CLEARED");
}

/* CFBG:<slot>:INFO:<idx> */
static void handle_info(uint8_t stack_id, const char *params) {
    if (!params || !params[0]) {
        ble_gatt_uplink_send_fail(stack_id, "INFO:MISSING_IDX");
        return;
    }
    uint8_t idx = (uint8_t)strtoul(params, NULL, 10);
    ble_gatt_device_t *d = ble_gatt_handler_get_device(idx);
    if (!d || !d->valid) {
        ble_gatt_uplink_send_fail(stack_id, "INFO:INVALID_IDX");
        return;
    }
    char ok[96];
    snprintf(ok, sizeof(ok),
             "INFO:%d:" MACSTR ":RSSI=%d:CONN=0x%04X:NAME=%s",
             idx, MAC2STR(d->addr), (int)d->rssi, d->conn_id, d->name);
    ble_gatt_uplink_send_ok(stack_id, ok);
}

/* CFBG:<slot>:CONNECT:<mac>
 * MAC format: AA:BB:CC:DD:EE:FF */
static void handle_connect(uint8_t stack_id, const char *params) {
    if (!params || strlen(params) < 17) {
        ble_gatt_uplink_send_fail(stack_id, "CONNECT:INVALID_MAC");
        return;
    }

    esp_bd_addr_t addr;
    if (sscanf(params, "%hhx:%hhx:%hhx:%hhx:%hhx:%hhx",
               &addr[0], &addr[1], &addr[2],
               &addr[3], &addr[4], &addr[5]) != 6) {
        ble_gatt_uplink_send_fail(stack_id, "CONNECT:MAC_PARSE_FAIL");
        return;
    }

    esp_gatt_if_t gattc_if = ble_gatt_handler_get_if();
    if (gattc_if == ESP_GATT_IF_NONE) {
        ble_gatt_uplink_send_fail(stack_id, "CONNECT:GATTC_NOT_READY");
        return;
    }

    /* Find or create device slot */
    uint8_t slot_idx;
    if (ble_gatt_handler_find_by_addr(addr, &slot_idx) != ESP_OK) {
        /* Not in scan table — create an ad-hoc slot */
        ble_gatt_device_t *free_dev = NULL;
        for (int i = 0; i < BLE_GATT_MAX_DEVICES; i++) {
            ble_gatt_device_t *d = ble_gatt_handler_get_device((uint8_t)i);
            if (d && !d->valid) {
                free_dev = d;
                slot_idx = (uint8_t)i;
                break;
            }
        }
        if (!free_dev) {
            ble_gatt_uplink_send_fail(stack_id, "CONNECT:TABLE_FULL");
            return;
        }
        memset(free_dev, 0, sizeof(*free_dev));
        memcpy(free_dev->addr, addr, ESP_BD_ADDR_LEN);
        free_dev->conn_id  = 0xFFFF;
        free_dev->gattc_if = ESP_GATT_IF_NONE;
        free_dev->stack_id = stack_id;
        snprintf(free_dev->name, BLE_GATT_DEV_NAME_LEN, "Unknown");
        free_dev->valid = true;
    }

    /* Ensure device has current stack_id so OPEN_EVT callback can respond */
    ble_gatt_device_t *connecting_dev = ble_gatt_handler_get_device(slot_idx);
    if (connecting_dev) connecting_dev->stack_id = stack_id;

    /* Mark pending stack for scan callbacks (kept for compatibility) */
    ble_gatt_handler_set_pending_stack(stack_id);

    /* Use the addr_type recorded during scan. Hardcoding BLE_ADDR_TYPE_PUBLIC
     * causes intermittent failures for random-address devices once Bluedroid's
     * internal scan cache expires (typically a few seconds after scan). */
    esp_ble_addr_type_t addr_type = (connecting_dev)
                                    ? connecting_dev->addr_type
                                    : BLE_ADDR_TYPE_PUBLIC;

    esp_err_t ret = esp_ble_gattc_open(gattc_if, addr, addr_type, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gattc_open failed: %s", esp_err_to_name(ret));
        ble_gatt_uplink_send_fail(stack_id, "CONNECT:OPEN_FAILED");
        return;
    }

    /* Connection params are applied in OPEN_EVT success handler, after the
     * link is established. Calling gap_update_conn_params here would fail
     * because L2CAP does not know the BD_ADDR until the connection is up. */
    /* No CONNECTING ack here — OPEN_EVT will send CONNECTED once established */
}

/* CFBG:<slot>:DISCONNECT:<idx> */
static void handle_disconnect(uint8_t stack_id, const char *params) {
    if (!params || !params[0]) {
        ble_gatt_uplink_send_fail(stack_id, "DISCONNECT:MISSING_IDX");
        return;
    }
    uint8_t idx = (uint8_t)strtoul(params, NULL, 10);
    ble_gatt_device_t *d = ble_gatt_handler_get_device(idx);
    if (!d || !d->valid) {
        ble_gatt_uplink_send_fail(stack_id, "DISCONNECT:INVALID_IDX");
        return;
    }
    if (d->conn_id == 0xFFFF) {
        ble_gatt_uplink_send_fail(stack_id, "DISCONNECT:NOT_CONNECTED");
        return;
    }
    esp_err_t ret = esp_ble_gattc_close(d->gattc_if, d->conn_id);
    if (ret != ESP_OK) {
        ble_gatt_uplink_send_fail(stack_id, "DISCONNECT:CLOSE_FAILED");
    } else {
        char ok[32];
        snprintf(ok, sizeof(ok), "DISCONNECTING:%d", idx);
        ble_gatt_uplink_send_ok(stack_id, ok);
    }
}

/* CFBG:<slot>:DISC:<idx>  — discover services + characteristics */
static void handle_disc(uint8_t stack_id, const char *params) {
    if (!params || !params[0]) {
        ble_gatt_uplink_send_fail(stack_id, "DISC:MISSING_IDX");
        return;
    }
    uint8_t idx = (uint8_t)strtoul(params, NULL, 10);
    ble_gatt_device_t *d = ble_gatt_handler_get_device(idx);
    if (!d || !d->valid) {
        ble_gatt_uplink_send_fail(stack_id, "DISC:INVALID_IDX");
        return;
    }
    if (d->conn_id == 0xFFFF) {
        ble_gatt_uplink_send_fail(stack_id, "DISC:NOT_CONNECTED");
        return;
    }
    /* Reset caches */
    d->num_services = 0;
    d->num_chars    = 0;
    memset(d->services, 0, sizeof(d->services));
    memset(d->chars,    0, sizeof(d->chars));

    /* Store stack_id so SEARCH_CMPL_EVT can send the batched response */
    d->stack_id = stack_id;

    esp_err_t ret = esp_ble_gattc_search_service(d->gattc_if, d->conn_id, NULL);
    if (ret != ESP_OK) {
        ble_gatt_uplink_send_fail(stack_id, "DISC:SEARCH_FAILED");
    }
    /* No DISC_STARTED ack — SEARCH_CMPL_EVT will send batched DISC_DONE */
}

/* CFBG:<slot>:READ:<idx>:<handle> */
static void handle_read(uint8_t stack_id, const char *params) {
    char buf[64];
    const char *tokens[3];
    int n = split_tokens(params, buf, sizeof(buf), tokens, 3);
    if (n < 2) {
        ble_gatt_uplink_send_fail(stack_id, "READ:PARSE_FAIL");
        return;
    }
    uint8_t  idx    = (uint8_t)strtoul(tokens[0], NULL, 10);
    uint16_t handle = parse_u16(tokens[1]);

    ble_gatt_device_t *d = ble_gatt_handler_get_device(idx);
    if (!d || !d->valid || d->conn_id == 0xFFFF) {
        ble_gatt_uplink_send_fail(stack_id, "READ:NOT_CONNECTED");
        return;
    }
    /* Save stack_id so READ_CHAR_EVT can route the response via the SAME RPC channel.
       Do NOT send READ_STARTED here — that would consume g_last_rpc_id on the gateway
       before the actual value arrives (READ_CHAR_EVT), causing the value to be routed
       to telemetry instead of returning it as the RPC response the widget is waiting for. */
    d->stack_id = stack_id;
    esp_err_t ret = esp_ble_gattc_read_char(d->gattc_if, d->conn_id,
                                             handle, ESP_GATT_AUTH_REQ_NONE);
    if (ret != ESP_OK) {
        ble_gatt_uplink_send_fail(stack_id, "READ:FAILED");
    }
    /* On success: no immediate ACK — READ_CHAR_EVT will send READ:<idx>:0x<handle>:<hex> */
}

/* CFBG:<slot>:WRITE:<idx>:<handle>:<hex_data>     (with response)
 * CFBG:<slot>:WRITENR:<idx>:<handle>:<hex_data>   (no response) */
static void handle_write(uint8_t stack_id, const char *params, bool with_rsp) {
    char buf[512];
    const char *tokens[4];
    int n = split_tokens(params, buf, sizeof(buf), tokens, 4);
    if (n < 3) {
        ble_gatt_uplink_send_fail(stack_id, "WRITE:PARSE_FAIL");
        return;
    }
    uint8_t  idx    = (uint8_t)strtoul(tokens[0], NULL, 10);
    uint16_t handle = parse_u16(tokens[1]);
    const char *hex = tokens[2];

    ble_gatt_device_t *d = ble_gatt_handler_get_device(idx);
    if (!d || !d->valid || d->conn_id == 0xFFFF) {
        ble_gatt_uplink_send_fail(stack_id, "WRITE:NOT_CONNECTED");
        return;
    }

    uint8_t  data_buf[128];
    uint16_t data_len = hex_decode(hex, data_buf, sizeof(data_buf));
    if (data_len == 0) {
        ble_gatt_uplink_send_fail(stack_id, "WRITE:INVALID_HEX");
        return;
    }

    esp_gatt_write_type_t wtype = with_rsp ? ESP_GATT_WRITE_TYPE_RSP
                                           : ESP_GATT_WRITE_TYPE_NO_RSP;
    esp_err_t ret = esp_ble_gattc_write_char(d->gattc_if, d->conn_id,
                                              handle, data_len, data_buf,
                                              wtype, ESP_GATT_AUTH_REQ_NONE);
    if (ret != ESP_OK) {
        ble_gatt_uplink_send_fail(stack_id, "WRITE:FAILED");
    } else {
        if (!with_rsp) {
            /* No response expected — ack immediately */
            char ok[48];
            snprintf(ok, sizeof(ok), "WRITE_NR_OK:%d:0x%04X", idx, handle);
            ble_gatt_uplink_send_ok(stack_id, ok);
        }
        /* with_rsp: result arrives in ESP_GATTC_WRITE_CHAR_EVT */
    }
}

/* CFBG:<slot>:NOTIFY:<idx>:<cccd_handle>:<1|0>
 * CFBG:<slot>:INDICATE:<idx>:<cccd_handle>:<1|0> */
static void handle_cccd(uint8_t stack_id, const char *params, bool is_notify) {
    char buf[64];
    const char *tokens[4];
    int n = split_tokens(params, buf, sizeof(buf), tokens, 4);
    if (n < 3) {
        ble_gatt_uplink_send_fail(stack_id, "CCCD:PARSE_FAIL");
        return;
    }
    uint8_t  idx     = (uint8_t)strtoul(tokens[0], NULL, 10);
    uint16_t handle  = parse_u16(tokens[1]);
    bool     enable  = (tokens[2][0] == '1');

    ble_gatt_device_t *d = ble_gatt_handler_get_device(idx);
    if (!d || !d->valid || d->conn_id == 0xFFFF) {
        ble_gatt_uplink_send_fail(stack_id, "CCCD:NOT_CONNECTED");
        return;
    }

    /* CCCD value: 0x0001=notify, 0x0002=indicate, 0x0000=disable */
    uint16_t cccd_val;
    if (!enable) {
        cccd_val = 0x0000;
    } else {
        cccd_val = is_notify ? 0x0001 : 0x0002;
    }
    uint8_t val_buf[2] = { (uint8_t)(cccd_val & 0xFF), (uint8_t)(cccd_val >> 8) };

    /* Register with ESP-IDF BLE stack so NOTIFY_EVT is delivered to the callback.
     * char value handle = cccd_handle - 1 (BLE convention: CCCD immediately follows). */
    if (enable) {
        uint16_t char_handle = (handle > 0) ? (handle - 1) : handle;
        esp_err_t reg_ret = esp_ble_gattc_register_for_notify(
            d->gattc_if, d->addr, char_handle);
        if (reg_ret != ESP_OK) {
            ESP_LOGW("ble_gatt_dn", "register_for_notify failed: %d (char_handle=0x%04X)",
                     reg_ret, char_handle);
        }
    }

    esp_err_t ret = esp_ble_gattc_write_char_descr(
        d->gattc_if, d->conn_id,
        handle, 2, val_buf,
        ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);

    if (ret != ESP_OK) {
        ble_gatt_uplink_send_fail(stack_id,
            is_notify ? "NOTIFY:FAILED" : "INDICATE:FAILED");
    }
    /* Result arrives in ESP_GATTC_WRITE_DESCR_EVT */
}

/* --------------------------------------------------------------------------
 * Command dispatcher
 * -------------------------------------------------------------------------- */

static void dispatch_item(const uint8_t *data, uint16_t len) {
    if (!data || len < 8) {
        ESP_LOGD(TAG, "dispatch_item: invalid input (len=%d)", len);
        return;
    }
    if (strncmp((const char *)data, "CFBG:", 5) != 0) {
        ESP_LOGD(TAG, "dispatch_item: not CFBG prefix");
        return;
    }

    ESP_LOGD(TAG, "dispatch_item: received %.16s... (len=%d)", (const char*)data, len);

    /* Format: CFBG:<slot>:<verb>:<params...>
     * Skip slot field, then extract verb and params. */
    const char *after_prefix = (const char *)(data + 5);
    const char *slot_end = strchr(after_prefix, ':');
    if (!slot_end) {
        ESP_LOGW(TAG, "dispatch_item: malformed - no slot:verb separator");
        return; /* malformed — no verb */
    }

    const uint8_t stack_id = 0; /* native GATT always uses stack 0 */

    const char *verb    = slot_end + 1;
    const char *c2      = strchr(verb, ':');
    size_t      verb_len = c2 ? (size_t)(c2 - verb) : strlen(verb);
    const char *params  = c2 ? c2 + 1 : "";

    char verb_buf[32] = {0};
    if (verb_len >= sizeof(verb_buf)) verb_len = sizeof(verb_buf) - 1;
    memcpy(verb_buf, verb, verb_len);

    ESP_LOGI(TAG, "stack=%u verb='%s' params='%.20s'", stack_id, verb_buf, params);

    /* Check if GATT mode is active */
    if (!config_ble_mode_is_active(BLE_MODE_GATT)) {
        ESP_LOGW(TAG, "GATT command rejected: module not active (mode=%s)",
                 config_ble_mode_name(config_ble_mode_get()));
        ble_gatt_uplink_send_fail(stack_id, "MODULE_NOT_ACTIVE");
        return;
    }

    if      (strcmp(verb_buf, "SCAN")       == 0) handle_scan(stack_id, params);
    else if (strcmp(verb_buf, "STOP")       == 0) handle_stop(stack_id);
    else if (strcmp(verb_buf, "LIST")       == 0) handle_list(stack_id);
    else if (strcmp(verb_buf, "CLEAR")      == 0) handle_clear(stack_id);
    else if (strcmp(verb_buf, "INFO")       == 0) handle_info(stack_id, params);
    else if (strcmp(verb_buf, "CONNECT")    == 0) handle_connect(stack_id, params);
    else if (strcmp(verb_buf, "DISCONNECT") == 0) handle_disconnect(stack_id, params);
    else if (strcmp(verb_buf, "DISC")       == 0) handle_disc(stack_id, params);
    else if (strcmp(verb_buf, "READ")       == 0) handle_read(stack_id, params);
    else if (strcmp(verb_buf, "WRITE")      == 0) handle_write(stack_id, params, true);
    else if (strcmp(verb_buf, "WRITENR")    == 0) handle_write(stack_id, params, false);
    else if (strcmp(verb_buf, "NOTIFY")     == 0) handle_cccd(stack_id, params, true);
    else if (strcmp(verb_buf, "INDICATE")   == 0) handle_cccd(stack_id, params, false);
    else {
        ESP_LOGW(TAG, "Unknown verb '%s'", verb_buf);
        ble_gatt_uplink_send_fail(stack_id, "UNKNOWN_VERB");
    }
}

/* --------------------------------------------------------------------------
 * Task
 * -------------------------------------------------------------------------- */

static void downlink_task(void *arg) {
    ESP_LOGI(TAG, "Downlink task started");

    while (s_task_running) {
        downlink_item_t *item = NULL;
        if (xQueueReceive(s_dn_queue, &item, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (item) {
                dispatch_item(item->raw, item->len);
                free(item);
            }
        }
    }

    ESP_LOGI(TAG, "Downlink task exiting");
    vTaskDelete(NULL);
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

esp_err_t ble_gatt_downlink_task_start(void) {
    if (s_task_running) return ESP_OK;

    if (!s_dn_queue) {
        /* Store pointers (4 bytes/slot) instead of full items (1024 bytes/slot).
         * Items are malloc'd from PSRAM in enqueue; this queue only needs
         * QUEUE_DEPTH * sizeof(ptr) = 32 bytes of internal RAM. */
        s_dn_queue = xQueueCreate(BLE_GATT_DOWNLINK_QUEUE_DEPTH,
                                   sizeof(downlink_item_t *));
        if (!s_dn_queue) {
            ESP_LOGE(TAG, "Failed to create downlink queue");
            return ESP_ERR_NO_MEM;
        }
    }

    s_task_running = true;
    s_dn_stack = (StackType_t *)heap_caps_malloc(8 * 1024, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_dn_tcb = (StaticTask_t *)heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (!s_dn_stack || !s_dn_tcb) {
        ESP_LOGE(TAG, "Failed to allocate memory for downlink task");
        if (s_dn_stack) heap_caps_free(s_dn_stack);
        if (s_dn_tcb) heap_caps_free(s_dn_tcb);
        s_task_running = false;
        return ESP_FAIL;
    }

    s_dn_task = xTaskCreateStatic(downlink_task, "ble_gatt_dn", 8 * 1024 / sizeof(StackType_t),
                                  NULL, 5, s_dn_stack, s_dn_tcb);

    if (s_dn_task == NULL) {
        s_task_running = false;
        heap_caps_free(s_dn_stack);
        heap_caps_free(s_dn_tcb);
        ESP_LOGE(TAG, "Failed to create downlink task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BLE GATT Downlink task created in PSRAM");
    return ESP_OK;
}

void ble_gatt_downlink_task_stop(void) {
    s_task_running = false;
    vTaskDelay(pdMS_TO_TICKS(200));
    s_dn_task = NULL;
    if (s_dn_stack) heap_caps_free(s_dn_stack);
    if (s_dn_tcb) heap_caps_free(s_dn_tcb);
    s_dn_stack = NULL;
    s_dn_tcb = NULL;
}

esp_err_t ble_gatt_downlink_enqueue(const uint8_t *data, uint16_t len) {
    if (!s_dn_queue) {
        ESP_LOGW(TAG, "Enqueue: queue not initialized");
        return ESP_ERR_INVALID_ARG;
    }
    if (!data || len == 0) {
        ESP_LOGW(TAG, "Enqueue: invalid data");
        return ESP_ERR_INVALID_ARG;
    }
    if (len >= BLE_GATT_DOWNLINK_ITEM_MAX) len = BLE_GATT_DOWNLINK_ITEM_MAX - 1;

    downlink_item_t *item = (downlink_item_t *)malloc(sizeof(downlink_item_t));
    if (!item) {
        ESP_LOGE(TAG, "Enqueue: out of memory");
        return ESP_ERR_NO_MEM;
    }
    memcpy(item->raw, data, len);
    item->raw[len] = '\0';
    item->len = len;

    ESP_LOGD(TAG, "Enqueue: sending %d bytes to task", len);
    if (xQueueSend(s_dn_queue, &item, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "Downlink queue full (item=%d bytes)", len);
        free(item);
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGD(TAG, "Enqueue: item queued successfully");
    return ESP_OK;
}
