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

static const char *TAG = "cfbn_cmds";
/* --------------------------------------------------------------------------
 * config_parse_ble_native_command

 * -------------------------------------------------------------------------- */

esp_err_t config_parse_ble_native_command(const uint8_t *data, uint16_t len) {
    /* Minimum: "CFBN:SCAN" = 9 chars */
    if (!data || len < 9) {
        ESP_LOGE(TAG, "CMD: invalid params (len=%u)", len);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check prefix */
    if (strncmp((const char *)data, "CFBN:", 5) != 0) {
        ESP_LOGE(TAG, "CMD: bad prefix");
        return ESP_FAIL;
    }

    /* Native BLE: no slot — always stack 0 */
    const uint8_t stack_id = 0;

    /* Check that config has been loaded */
    if (!ble_native_config_is_loaded(stack_id)) {
        ESP_LOGW(TAG, "CMD: not configured yet — send CFBN:JSON first");
        const char *err = "CFBN:FAIL:NOT_CONFIGURED";
        mcu_wan_enqueue_uplink(HANDLER_BLE_NATIVE,
                               (uint8_t *)err, (uint16_t)strlen(err));
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "CMD: len=%u", len);

    /* Route full raw command to handler */
    esp_err_t ret = ble_native_handler_execute(data, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CMD: execute failed: %s", esp_err_to_name(ret));
        const char *err = "CFBN:FAIL:QUEUE_FULL";
        mcu_wan_enqueue_uplink(HANDLER_BLE_NATIVE,
                               (uint8_t *)err, (uint16_t)strlen(err));
    }
    return ret;
}

/* --------------------------------------------------------------------------
 * config_parse_ble_native_json
 * -------------------------------------------------------------------------- */

esp_err_t config_parse_ble_native_json(const uint8_t *data, uint16_t len) {
    /* Minimum: "CFBN:JSON:{}" = 12 chars */
    if (!data || len < 12) {
        ESP_LOGE(TAG, "JSON: invalid params (len=%u)", len);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check prefix */
    if (strncmp((const char *)data, "CFBN:JSON:", 10) != 0) {
        ESP_LOGE(TAG, "JSON: bad prefix");
        return ESP_FAIL;
    }

    /* Native BLE: no slot — always stack 0 */
    const uint8_t stack_id = 0;

    /* JSON data starts immediately after "CFBN:JSON:" */
    const char *json_data = (const char *)(data + 10);
    uint16_t json_len = len - 10;

    if (json_len < 2 || json_len > CONFIG_CMD_MAX_LEN - 16) {
        ESP_LOGE(TAG, "JSON: invalid JSON length %u", json_len);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "JSON: json_len=%u", json_len);

    esp_err_t ret = ble_native_handler_load_config(stack_id, json_data, json_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "JSON: load_config failed: %s", esp_err_to_name(ret));
    }
    return ret;
}
