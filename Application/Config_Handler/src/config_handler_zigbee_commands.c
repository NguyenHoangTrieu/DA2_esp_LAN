/**
 * @file config_handler_zigbee_commands.c
 * @brief Zigbee command parsers implementation
 *
 * Command format : "CFZB:<stack_id>:<func_name>[:<hex_data>]"
 * JSON format    : "CFZB:JSON:<stack_id>:<json_data>"
 *
 * Unlike LoRa (which passes through ASCII AT commands), Zigbee commands are
 * addressed by function name.  The optional hex_data field provides the binary
 * payload bytes appended to the HEX frame built by zigbee_handler.
 */

#include "config_handler.h"
#include "config_handler_zigbee_commands.h"
#include "zigbee_handler.h"
#include "zigbee_handler_task.h"
#include "module_monitor_task.h"
#include "esp_log.h"
#include "mcu_wan_handler.h"
#include "frame_types.h"
#include <string.h>
#include <stdlib.h>

#define ZIGBEE_MAX_STACKS 2

static const char *TAG = "zigbee_commands";

/* ============================================================================
 * Static Function Name → ID Mapping Table
 * ========================================================================== */

typedef struct { const char *name; zigbee_function_id_t id; } zb_func_entry_t;

static const zb_func_entry_t ZB_FUNC_MAP[] = {
    { "MODULE_HW_RESET",              ZIGBEE_FUNC_HW_RESET              },
    { "MODULE_SW_RESET",              ZIGBEE_FUNC_SW_RESET              },
    { "MODULE_FACTORY_RESET",         ZIGBEE_FUNC_FACTORY_RESET         },
    { "MODULE_GET_INFO",              ZIGBEE_FUNC_GET_INFO              },
    { "MODULE_ENTER_HEX_MODE",        ZIGBEE_FUNC_ENTER_HEX_MODE        },
    { "MODULE_START_NETWORK",         ZIGBEE_FUNC_START_NETWORK         },
    { "MODULE_STOP_NETWORK",          ZIGBEE_FUNC_STOP_NETWORK          },
    { "MODULE_GET_NET_STATUS",        ZIGBEE_FUNC_GET_NET_STATUS        },
    { "MODULE_SET_CHANNEL",           ZIGBEE_FUNC_SET_CHANNEL           },
    { "MODULE_SET_PANID",             ZIGBEE_FUNC_SET_PANID             },
    { "MODULE_SET_TX_POWER",          ZIGBEE_FUNC_SET_TX_POWER          },
    { "MODULE_SET_PERMIT_JOIN",       ZIGBEE_FUNC_SET_PERMIT_JOIN       },
    { "MODULE_NODE_JOIN_NOTIFY",      ZIGBEE_FUNC_NODE_JOIN_NOTIFY      },
    { "MODULE_NODE_LEAVE_NOTIFY",     ZIGBEE_FUNC_NODE_LEAVE_NOTIFY     },
    { "MODULE_NODE_ANNOUNCE_NOTIFY",  ZIGBEE_FUNC_NODE_ANNOUNCE_NOTIFY  },
    { "MODULE_QUERY_SHORT_ADDR",      ZIGBEE_FUNC_QUERY_SHORT_ADDR      },
    { "MODULE_QUERY_NODE_PORT_INFO",  ZIGBEE_FUNC_QUERY_NODE_PORT_INFO  },
    { "MODULE_DELETE_NODE",           ZIGBEE_FUNC_DELETE_NODE           },
    { "MODULE_ZCL_READ_ATTR",         ZIGBEE_FUNC_ZCL_READ_ATTR         },
    { "MODULE_ZCL_WRITE_ATTR",        ZIGBEE_FUNC_ZCL_WRITE_ATTR        },
    { "MODULE_ZCL_SEND_CONTROL_CMD",  ZIGBEE_FUNC_ZCL_SEND_CONTROL_CMD  },
    { "MODULE_ZCL_RECV_CONTROL_CMD",  ZIGBEE_FUNC_ZCL_RECV_CONTROL_CMD  },
    { "MODULE_ZCL_RECV_ATTR_REPORT",  ZIGBEE_FUNC_ZCL_RECV_ATTR_REPORT  },
    { "MODULE_ZCL_SET_REPORT_RULE",   ZIGBEE_FUNC_ZCL_SET_REPORT_RULE   },
    { "MODULE_SEND_UNICAST",          ZIGBEE_FUNC_SEND_UNICAST          },
    { "MODULE_SEND_BROADCAST",        ZIGBEE_FUNC_SEND_BROADCAST        },
};
#define ZB_FUNC_MAP_SIZE ((int)(sizeof(ZB_FUNC_MAP) / sizeof(ZB_FUNC_MAP[0])))

static zigbee_function_id_t find_func_id(const char *name) {
    for (int i = 0; i < ZB_FUNC_MAP_SIZE; i++) {
        if (strcmp(ZB_FUNC_MAP[i].name, name) == 0) {
            return ZB_FUNC_MAP[i].id;
        }
    }
    return ZIGBEE_FUNC_INVALID;
}

/* ============================================================================
 * Hex-string Decoder
 * Accepts "55 80 03" or "558003" (with or without spaces)
 * ========================================================================== */

static uint8_t nibble(char c) {
    if (c >= '0' && c <= '9') return (uint8_t)(c - '0');
    if (c >= 'A' && c <= 'F') return (uint8_t)(c - 'A' + 10);
    if (c >= 'a' && c <= 'f') return (uint8_t)(c - 'a' + 10);
    return 0;
}

static uint8_t hex_string_decode(const char *hex, uint8_t *out, uint8_t max_len) {
    uint8_t count = 0;
    const char *p = hex;
    while (*p && count < max_len) {
        while (*p == ' ') p++;
        if (*p == '\0') break;
        char hi = *p++;
        if (*p == '\0' || *p == ' ') {
            /* single hex nibble – should not happen in valid input */
            out[count++] = (uint8_t)(nibble(hi) & 0x0F);
        } else {
            char lo = *p++;
            out[count++] = (uint8_t)((nibble(hi) << 4) | nibble(lo));
        }
    }
    return count;
}

/* ============================================================================
 * Unified Zigbee Command Parser
 * ========================================================================== */

esp_err_t config_parse_zigbee_command(const uint8_t *data, uint16_t len) {
    if (!data || len < 8) {
        ESP_LOGE(TAG, "ZIGBEE CMD: invalid parameters (len=%u)", len);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check prefix "CFZB:" */
    if (strncmp((const char *)data, "CFZB:", 5) != 0) {
        ESP_LOGE(TAG, "ZIGBEE CMD: wrong prefix");
        return ESP_FAIL;
    }

    /* Parse: CFZB:<stack_id>:<func_name>[:<hex_data>] */
    const char *ptr   = (const char *)(data + 5);
    const char *colon = strchr(ptr, ':');
    if (!colon) {
        ESP_LOGE(TAG, "ZIGBEE CMD: missing stack_id separator");
        return ESP_FAIL;
    }

    uint8_t stack_id = (uint8_t)atoi(ptr);
    if (stack_id >= ZIGBEE_MAX_STACKS) {
        ESP_LOGE(TAG, "ZIGBEE CMD: invalid stack_id %u", stack_id);
        return ESP_FAIL;
    }

    /* func_name (and optional hex_data after second ':') */
    const char *func_part = colon + 1;
    char func_name[64]    = {0};
    const char *hex_part  = NULL;

    const char *colon2 = strchr(func_part, ':');
    if (colon2) {
        size_t fname_len = (size_t)(colon2 - func_part);
        if (fname_len >= sizeof(func_name)) fname_len = sizeof(func_name) - 1;
        memcpy(func_name, func_part, fname_len);
        hex_part = colon2 + 1;
    } else {
        size_t fname_len = strlen(func_part);
        if (fname_len >= sizeof(func_name)) fname_len = sizeof(func_name) - 1;
        memcpy(func_name, func_part, fname_len);
    }

    /* Look up function ID */
    zigbee_function_id_t fid = find_func_id(func_name);
    if (fid == ZIGBEE_FUNC_INVALID) {
        ESP_LOGE(TAG, "ZIGBEE CMD: unknown function '%s'", func_name);
        char err[64];
        int  el = snprintf(err, sizeof(err), "CFZB:%d:FAIL:%s:UNKNOWN_FUNC",
                           stack_id, func_name);
        if (el > 0) mcu_wan_enqueue_uplink(HANDLER_ZIGBEE, (uint8_t *)err, (uint16_t)el);
        return ESP_FAIL;
    }

    /* Decode optional hex payload */
    uint8_t payload[252]   = {0};
    uint8_t payload_len    = 0;
    if (hex_part && *hex_part != '\0') {
        payload_len = hex_string_decode(hex_part, payload, sizeof(payload));
    }

    ESP_LOGI(TAG, "ZIGBEE CMD: stack=%u func=%s payload_len=%u",
             stack_id, func_name, payload_len);

    /* Build command request */
    zigbee_command_request_t req = {0};
    req.stack_id  = stack_id;
    req.func_id   = fid;
    req.data_len  = payload_len;
    if (payload_len > 0) memcpy(req.data, payload, payload_len);

    esp_err_t ret = zigbee_handler_task_execute_command(&req);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ZIGBEE CMD: queue failed: %s", esp_err_to_name(ret));
        char err[64];
        int  el = snprintf(err, sizeof(err), "CFZB:%d:FAIL:%s:QUEUE_FULL",
                           stack_id, func_name);
        if (el > 0) mcu_wan_enqueue_uplink(HANDLER_ZIGBEE, (uint8_t *)err, (uint16_t)el);
        return ret;
    }

    ESP_LOGI(TAG, "ZIGBEE CMD: enqueued successfully");
    return ESP_OK;
}

/* ============================================================================
 * JSON Config Parser
 * ========================================================================== */

esp_err_t config_parse_zigbee_json(const uint8_t *data, uint16_t len) {
    if (!data || len < 15) {
        ESP_LOGE(TAG, "ZIGBEE JSON: invalid parameters (len=%u)", len);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check prefix "CFZB:JSON:" */
    if (strncmp((const char *)data, "CFZB:JSON:", 10) != 0) {
        ESP_LOGE(TAG, "ZIGBEE JSON: wrong prefix");
        return ESP_FAIL;
    }

    const char *ptr   = (const char *)(data + 10);
    const char *colon = strchr(ptr, ':');
    if (!colon) {
        ESP_LOGE(TAG, "ZIGBEE JSON: missing separator");
        return ESP_FAIL;
    }

    uint8_t     stack_id  = (uint8_t)atoi(ptr);
    const char *json_data = colon + 1;
    uint16_t    json_len  = len - (uint16_t)(json_data - (const char *)data);

    if (stack_id >= ZIGBEE_MAX_STACKS) {
        ESP_LOGE(TAG, "ZIGBEE JSON: invalid stack_id %u", stack_id);
        return ESP_FAIL;
    }
    if (json_len < 2 || json_len > 8912) {
        ESP_LOGE(TAG, "ZIGBEE JSON: invalid length %u", json_len);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "ZIGBEE JSON: stack=%u length=%u", stack_id, json_len);

    esp_err_t ret = module_monitor_send_config(stack_id, json_data, json_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "ZIGBEE JSON: module_monitor queue failed: %s",
                 esp_err_to_name(ret));
        uint8_t err_resp[] = "CFZB:JSON:FAIL:QUEUE";
        mcu_wan_enqueue_uplink(HANDLER_ZIGBEE, err_resp, sizeof(err_resp) - 1);
        return ret;
    }

    ESP_LOGI(TAG, "ZIGBEE JSON: forwarded to module_monitor_task");
    return ESP_OK;
}
