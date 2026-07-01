/**
 * @file ble_gatt_config.c
 * @brief BLE GATT Central JSON config parser.
 *
 * Parses "CFBG:JSON:<slot>:<json>" payloads and populates ble_gatt_stack_config_t.
 * All scan and connection parameters are loaded from JSON — nothing hardcoded.
 */

#include "ble_gatt_config.h"
#include "cJSON.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ble_gatt_cfg";

/* --------------------------------------------------------------------------
 * Static storage
 * -------------------------------------------------------------------------- */

static ble_gatt_stack_config_t s_configs[BLE_GATT_MAX_STACKS];
static bool s_initialized = false;

static void ensure_init(void) {
    if (!s_initialized) {
        memset(s_configs, 0, sizeof(s_configs));
        for (int i = 0; i < BLE_GATT_MAX_STACKS; i++) {
            s_configs[i].stack_id = (uint8_t)i;
            /* Sensible defaults applied even before JSON load */
            s_configs[i].scan.interval = 160;  /* 100 ms */
            s_configs[i].scan.window   = 80;   /*  50 ms */
            s_configs[i].scan.active   = true;
            s_configs[i].connection.interval_min        = 16;
            s_configs[i].connection.interval_max        = 32;
            s_configs[i].connection.latency             = 0;
            s_configs[i].connection.supervision_timeout = 500;
        }
        s_initialized = true;
    }
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

esp_err_t ble_gatt_config_load(uint8_t stack_id,
                                const char *json_str,
                                uint16_t json_len)
{
    ensure_init();
    if (stack_id >= BLE_GATT_MAX_STACKS || !json_str || json_len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *root = cJSON_ParseWithLength(json_str, json_len);
    if (!root) {
        ESP_LOGE(TAG, "stack=%u: JSON parse failed", stack_id);
        return ESP_ERR_INVALID_ARG;
    }

    ble_gatt_stack_config_t *cfg = &s_configs[stack_id];

    /* Locate "ble_gatt" sub-object */
    cJSON *j_gatt = cJSON_GetObjectItemCaseSensitive(root, "ble_gatt");
    if (!j_gatt) {
        ESP_LOGW(TAG, "stack=%u: no 'ble_gatt' key, keeping defaults", stack_id);
        cJSON_Delete(root);
        cfg->loaded = true;
        return ESP_OK;
    }

    /* --- Scan parameters --- */
    cJSON *j_scan = cJSON_GetObjectItemCaseSensitive(j_gatt, "scan");
    if (j_scan) {
        cJSON *j_iv = cJSON_GetObjectItemCaseSensitive(j_scan, "interval");
        cJSON *j_wn = cJSON_GetObjectItemCaseSensitive(j_scan, "window");
        cJSON *j_ac = cJSON_GetObjectItemCaseSensitive(j_scan, "active");

        if (cJSON_IsNumber(j_iv) && j_iv->valuedouble >= 4 && j_iv->valuedouble <= 16384)
            cfg->scan.interval = (uint16_t)j_iv->valuedouble;
        if (cJSON_IsNumber(j_wn) && j_wn->valuedouble >= 4 && j_wn->valuedouble <= 16384)
            cfg->scan.window = (uint16_t)j_wn->valuedouble;
        if (cJSON_IsBool(j_ac))
            cfg->scan.active = cJSON_IsTrue(j_ac);
    }

    /* --- Connection parameters --- */
    cJSON *j_conn = cJSON_GetObjectItemCaseSensitive(j_gatt, "connection");
    if (j_conn) {
        cJSON *j_imin = cJSON_GetObjectItemCaseSensitive(j_conn, "interval_min");
        cJSON *j_imax = cJSON_GetObjectItemCaseSensitive(j_conn, "interval_max");
        cJSON *j_lat  = cJSON_GetObjectItemCaseSensitive(j_conn, "latency");
        cJSON *j_sup  = cJSON_GetObjectItemCaseSensitive(j_conn, "supervision_timeout");

        if (cJSON_IsNumber(j_imin) && j_imin->valuedouble >= 6)
            cfg->connection.interval_min = (uint16_t)j_imin->valuedouble;
        if (cJSON_IsNumber(j_imax) && j_imax->valuedouble >= 6)
            cfg->connection.interval_max = (uint16_t)j_imax->valuedouble;
        if (cJSON_IsNumber(j_lat))
            cfg->connection.latency = (uint16_t)j_lat->valuedouble;
        if (cJSON_IsNumber(j_sup) && j_sup->valuedouble >= 10)
            cfg->connection.supervision_timeout = (uint16_t)j_sup->valuedouble;
    }

    cJSON_Delete(root);
    cfg->loaded = true;

    ESP_LOGI(TAG, "stack=%u loaded: scan iv=%u win=%u active=%d | conn imin=%u imax=%u",
             stack_id,
             cfg->scan.interval, cfg->scan.window, (int)cfg->scan.active,
             cfg->connection.interval_min, cfg->connection.interval_max);

    return ESP_OK;
}

ble_gatt_stack_config_t *ble_gatt_config_get(uint8_t stack_id) {
    ensure_init();
    if (stack_id >= BLE_GATT_MAX_STACKS) return &s_configs[0];
    return &s_configs[stack_id];
}

bool ble_gatt_config_is_loaded(uint8_t stack_id) {
    ensure_init();
    if (stack_id >= BLE_GATT_MAX_STACKS) return false;
    return s_configs[stack_id].loaded;
}
