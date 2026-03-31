/**
 * @file config_ble_mode.c
 * @brief BLE Module Mode Control implementation
 */

#include "config_ble_mode.h"
#include "esp_log.h"

static const char *TAG = "config_ble_mode";

/* Current BLE module mode */
static ble_module_mode_t s_current_mode = BLE_MODE_DISABLED;

esp_err_t config_ble_mode_set(ble_module_mode_t new_mode) {
    if (s_current_mode == new_mode) {
        return ESP_OK;  /* Already in this mode */
    }

    const char *old_name = config_ble_mode_name(s_current_mode);
    const char *new_name = config_ble_mode_name(new_mode);

    ESP_LOGI(TAG, "BLE mode transition: %s → %s", old_name, new_name);

    /* Log which module is being disabled */
    if (s_current_mode == BLE_MODE_GATT) {
        ESP_LOGW(TAG, "Disabling GATT Central (CFBG:) — switching to %s", new_name);
    } else if (s_current_mode == BLE_MODE_NATIVE) {
        ESP_LOGW(TAG, "Disabling BLE Native Mesh (CFBN:) — switching to %s", new_name);
    }

    /* Log which module is being enabled */
    if (new_mode == BLE_MODE_GATT) {
        ESP_LOGI(TAG, "Enabling GATT Central (CFBG:) — native commands will be rejected");
    } else if (new_mode == BLE_MODE_NATIVE) {
        ESP_LOGI(TAG, "Enabling BLE Native Mesh (CFBN:) — GATT commands will be rejected");
    } else {
        ESP_LOGW(TAG, "Disabling all BLE modules");
    }

    s_current_mode = new_mode;
    return ESP_OK;
}

ble_module_mode_t config_ble_mode_get(void) {
    return s_current_mode;
}

bool config_ble_mode_is_active(ble_module_mode_t mode) {
    return (s_current_mode == mode);
}

const char *config_ble_mode_name(ble_module_mode_t mode) {
    switch (mode) {
        case BLE_MODE_DISABLED: return "DISABLED";
        case BLE_MODE_GATT:     return "GATT";
        case BLE_MODE_NATIVE:   return "NATIVE";
        default:                return "UNKNOWN";
    }
}
