/**
 * @file config_handler_ble_gatt_commands.c
 * @brief BLE GATT Central command router — CFBG: prefix dispatcher.
 */

#include "config_handler_ble_gatt_commands.h"
#include "ble_gatt_handler.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "cfg_ble_gatt";

/* --------------------------------------------------------------------------
 * CFBG:JSON:<json>
 * (no slot — GATT Central is native on LAN MCU, always stack 0)
 * -------------------------------------------------------------------------- */
void config_parse_ble_gatt_json(const char *data, uint16_t len) {
    if (!data || len < 12) {
        ESP_LOGE(TAG, "JSON: payload too short");
        return;
    }

    /* Format: "CFBG:JSON:<json>" */
    if (strncmp(data, "CFBG:JSON:", 10) != 0) {
        ESP_LOGE(TAG, "JSON: unexpected prefix");
        return;
    }

    /* JSON starts immediately after "CFBG:JSON:" */
    const char *json = data + 10;
    uint16_t json_len = len - 10;

    ESP_LOGI(TAG, "Loading GATT JSON (%u bytes)", json_len);
    ble_gatt_handler_load_config(0, json, json_len);
}

/* --------------------------------------------------------------------------
 * CFBG:<slot>:<verb>[:<params>]
 * -------------------------------------------------------------------------- */
void config_parse_ble_gatt_command(const char *data, uint16_t len) {
    if (!data || len < 8) {
        ESP_LOGE(TAG, "CMD: payload too short");
        return;
    }
    ESP_LOGI(TAG, "Routing CFBG command: %.*s", (int)len, data);
    ble_gatt_handler_execute((const uint8_t *)data, len);
}
