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
#include "esp_log.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
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

    /* Set pending stack so GAP callbacks know which channel to use */
    ble_gatt_handler_set_pending_stack(stack_id);

    ret = esp_ble_gap_start_scanning((uint32_t)duration_sec);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "start_scanning failed: %s", esp_err_to_name(ret));
        ble_gatt_uplink_send_fail(stack_id, "SCAN:START_FAIL");
        return;
    }

    char ok[48];
    snprintf(ok, sizeof(ok), "SCAN_STARTED:%u", (unsigned)(duration_sec * 1000));
    ble_gatt_uplink_send_ok(stack_id, ok);
}

/* CFBG:<slot>:STOP */
static void handle_stop(uint8_t stack_id) {
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

    /* Mark pending stack for OPEN_EVT callback */
    ble_gatt_handler_set_pending_stack(stack_id);

    ble_gatt_stack_config_t *cfg = ble_gatt_config_get(stack_id);
    esp_err_t ret = esp_ble_gattc_open(gattc_if, addr,
                                        BLE_ADDR_TYPE_PUBLIC, true);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "gattc_open failed: %s", esp_err_to_name(ret));
        ble_gatt_uplink_send_fail(stack_id, "CONNECT:OPEN_FAILED");
        return;
    }

    /* Set connection params from JSON config */
    esp_ble_conn_update_params_t conn_params = {
        .min_int  = cfg->connection.interval_min,
        .max_int  = cfg->connection.interval_max,
        .latency  = cfg->connection.latency,
        .timeout  = cfg->connection.supervision_timeout,
    };
    memcpy(conn_params.bda, addr, ESP_BD_ADDR_LEN);
    esp_ble_gap_update_conn_params(&conn_params);

    char ok[48];
    snprintf(ok, sizeof(ok), "CONNECTING:%d:" MACSTR, slot_idx, MAC2STR(addr));
    ble_gatt_uplink_send_ok(stack_id, ok);
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

    esp_err_t ret = esp_ble_gattc_search_service(d->gattc_if, d->conn_id, NULL);
    if (ret != ESP_OK) {
        ble_gatt_uplink_send_fail(stack_id, "DISC:SEARCH_FAILED");
    } else {
        char ok[40];
        snprintf(ok, sizeof(ok), "DISC_STARTED:%d", idx);
        ble_gatt_uplink_send_ok(stack_id, ok);
    }
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
    esp_err_t ret = esp_ble_gattc_read_char(d->gattc_if, d->conn_id,
                                             handle, ESP_GATT_AUTH_REQ_NONE);
    if (ret != ESP_OK) {
        ble_gatt_uplink_send_fail(stack_id, "READ:FAILED");
    } else {
        char ok[32];
        snprintf(ok, sizeof(ok), "READ_STARTED:%d:0x%04X", idx, handle);
        ble_gatt_uplink_send_ok(stack_id, ok);
    }
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
    if (!data || len < 7) return;
    if (strncmp((const char *)data, "CFBG:", 5) != 0) return;

    /* Native BLE GATT: always use stack 0 */
    const uint8_t stack_id = 0;

    /* Verb directly after "CFBG:" */
    const char *verb = (const char *)(data + 5);
    const char *c2   = strchr(verb, ':');
    size_t verb_len  = c2 ? (size_t)(c2 - verb) : strlen(verb);
    const char *params = c2 ? c2 + 1 : "";

    char verb_buf[32] = {0};
    if (verb_len >= sizeof(verb_buf)) verb_len = sizeof(verb_buf) - 1;
    memcpy(verb_buf, verb, verb_len);

    ESP_LOGI(TAG, "stack=%u verb='%s'", stack_id, verb_buf);

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
    downlink_item_t *item = NULL;
    ESP_LOGI(TAG, "Downlink task started");

    while (s_task_running) {
        if (!item) {
            item = (downlink_item_t *)malloc(sizeof(downlink_item_t));
            if (!item) {
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
        }
        if (xQueueReceive(s_dn_queue, item, pdMS_TO_TICKS(100)) == pdTRUE) {
            dispatch_item(item->raw, item->len);
        }
    }

    free(item);
    ESP_LOGI(TAG, "Downlink task exiting");
    vTaskDelete(NULL);
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

esp_err_t ble_gatt_downlink_task_start(void) {
    if (s_task_running) return ESP_OK;

    if (!s_dn_queue) {
        s_dn_queue = xQueueCreate(BLE_GATT_DOWNLINK_QUEUE_DEPTH,
                                   sizeof(downlink_item_t));
        if (!s_dn_queue) {
            ESP_LOGE(TAG, "Failed to create downlink queue");
            return ESP_ERR_NO_MEM;
        }
    }

    s_task_running = true;
    BaseType_t ret = xTaskCreate(downlink_task, "ble_gatt_dn",
                                  8 * 1024, NULL, 5, &s_dn_task);
    if (ret != pdPASS) {
        s_task_running = false;
        return ESP_FAIL;
    }
    return ESP_OK;
}

void ble_gatt_downlink_task_stop(void) {
    s_task_running = false;
    vTaskDelay(pdMS_TO_TICKS(200));
    s_dn_task = NULL;
}

esp_err_t ble_gatt_downlink_enqueue(const uint8_t *data, uint16_t len) {
    if (!s_dn_queue || !data || len == 0) return ESP_ERR_INVALID_ARG;
    if (len >= BLE_GATT_DOWNLINK_ITEM_MAX) len = BLE_GATT_DOWNLINK_ITEM_MAX - 1;

    downlink_item_t item;
    memcpy(item.raw, data, len);
    item.raw[len] = '\0';
    item.len = len;

    if (xQueueSend(s_dn_queue, &item, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "Downlink queue full");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
