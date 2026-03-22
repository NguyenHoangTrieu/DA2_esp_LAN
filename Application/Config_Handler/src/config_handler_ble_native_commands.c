/**
 * @file config_handler_ble_native_commands.c
 * @brief Config handler BLE Native command parsers.
 *
 * Mirrors config_handler_ble_commands.c but for "CFBN:" prefixed commands
 * that target the ESP32-S3 native BLE Mesh provisioner.
 *
 * WAN MCU (DA2_esp) is not modified — it sends CFBN: strings through exactly
 * the same DT frame path it already uses for CFBL:, CFLR:, CFZB: commands.
 */

#include "config_handler_ble_native_commands.h"
#include "config_handler.h"
#include "ble_native_handler.h"
#include "ble_native_config.h"
#include "esp_log.h"
#include "mcu_wan_handler.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#define BLE_NATIVE_MAX_STACKS 2   /* Must match ble_native_config.h */

static const char *TAG = "cfbn_cmds";

/* --------------------------------------------------------------------------
 * config_parse_ble_native_command
 * -------------------------------------------------------------------------- */

esp_err_t config_parse_ble_native_command(const uint8_t *data, uint16_t len) {
    /* Minimum: "CFBN:0:X" = 8 chars */
    if (!data || len < 8) {
        ESP_LOGE(TAG, "CMD: invalid params (len=%u)", len);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check prefix */
    if (strncmp((const char *)data, "CFBN:", 5) != 0) {
        ESP_LOGE(TAG, "CMD: bad prefix");
        return ESP_FAIL;
    }

    /* Extract stack_id */
    const char *ptr = (const char *)(data + 5);
    const char *colon = strchr(ptr, ':');
    if (!colon) {
        ESP_LOGE(TAG, "CMD: missing ':' after stack_id");
        return ESP_FAIL;
    }

    uint8_t stack_id = (uint8_t)atoi(ptr);
    if (stack_id >= BLE_NATIVE_MAX_STACKS) {
        ESP_LOGE(TAG, "CMD: invalid stack_id %u", stack_id);
        /* Send immediate error uplink */
        char err[48];
        int n = snprintf(err, sizeof(err), "CFBN:%u:FAIL:INVALID_STACK", stack_id);
        if (n > 0) {
            mcu_wan_enqueue_uplink(HANDLER_BLE_NATIVE, (uint8_t *)err, (uint16_t)n);
        }
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "CMD: stack=%u len=%u", stack_id, len);

    /* Check that config has been loaded for this stack */
    if (!ble_native_config_is_loaded(stack_id)) {
        ESP_LOGW(TAG, "CMD: stack=%u not configured yet — send CFBN:JSON first", stack_id);
        char err[64];
        int n = snprintf(err, sizeof(err), "CFBN:%u:FAIL:NOT_CONFIGURED", stack_id);
        if (n > 0) {
            mcu_wan_enqueue_uplink(HANDLER_BLE_NATIVE, (uint8_t *)err, (uint16_t)n);
        }
        return ESP_FAIL;
    }

    /* Route full raw command to handler (includes "CFBN:<id>:<verb>:...") */
    esp_err_t ret = ble_native_handler_execute(data, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CMD: execute failed: %s", esp_err_to_name(ret));
        char err[64];
        int n = snprintf(err, sizeof(err), "CFBN:%u:FAIL:QUEUE_FULL", stack_id);
        if (n > 0) {
            mcu_wan_enqueue_uplink(HANDLER_BLE_NATIVE, (uint8_t *)err, (uint16_t)n);
        }
        return ret;
    }

    return ESP_OK;
}

/* --------------------------------------------------------------------------
 * config_parse_ble_native_json
 * -------------------------------------------------------------------------- */

esp_err_t config_parse_ble_native_json(const uint8_t *data, uint16_t len) {
    /* Minimum: "CFBN:JSON:0:{}" = 14 chars */
    if (!data || len < 14) {
        ESP_LOGE(TAG, "JSON: invalid params (len=%u)", len);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check prefix */
    if (strncmp((const char *)data, "CFBN:JSON:", 10) != 0) {
        ESP_LOGE(TAG, "JSON: bad prefix");
        return ESP_FAIL;
    }

    /* Parse: CFBN:JSON:<stack_id>:<json_data> */
    const char *ptr = (const char *)(data + 10);
    const char *colon = strchr(ptr, ':');
    if (!colon) {
        ESP_LOGE(TAG, "JSON: missing ':' after stack_id");
        return ESP_FAIL;
    }

    uint8_t stack_id = (uint8_t)atoi(ptr);
    const char *json_data = colon + 1;
    uint16_t json_len = len - (uint16_t)(json_data - (const char *)data);

    if (stack_id >= BLE_NATIVE_MAX_STACKS) {
        ESP_LOGE(TAG, "JSON: invalid stack_id %u", stack_id);
        return ESP_FAIL;
    }
    if (json_len < 2 || json_len > CONFIG_CMD_MAX_LEN - 16) {
        ESP_LOGE(TAG, "JSON: invalid JSON length %u", json_len);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "JSON: stack=%u json_len=%u", stack_id, json_len);

    /* Hand off to handler which will parse and apply mesh keys */
    esp_err_t ret = ble_native_handler_load_config(stack_id, json_data, json_len);
    if (ret != ESP_OK) {
        /* Uplink error already sent by handler */
        ESP_LOGE(TAG, "JSON: load_config failed: %s", esp_err_to_name(ret));
    }
    return ret;
}
