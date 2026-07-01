/**
 * @file config_ble_mode.h
 * @brief BLE Module Mode Control — mutual exclusion for GATT vs Native
 * 
 * Only one BLE module can be active at a time due to Bluedroid GAP constraint.
 * This module tracks and manages which one is active.
 */

#ifndef CONFIG_BLE_MODE_H
#define CONFIG_BLE_MODE_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BLE module mode enumeration
 */
typedef enum {
    BLE_MODE_DISABLED  = 0,  /**< No BLE module active */
    BLE_MODE_GATT      = 1,  /**< GATT Central (CFBG:) active */
    BLE_MODE_NATIVE    = 2,  /**< BLE Native Mesh (CFBN:) active */
} ble_module_mode_t;

/**
 * @brief Set BLE module mode and disable the other
 * 
 * When switching to GATT mode, BLE Native commands are ignored.
 * When switching to NATIVE mode, GATT commands are ignored.
 * 
 * @param new_mode The new mode (GATT or NATIVE)
 * @return ESP_OK on success
 */
esp_err_t config_ble_mode_set(ble_module_mode_t new_mode);

/**
 * @brief Get current BLE module mode
 * 
 * @return Current active mode
 */
ble_module_mode_t config_ble_mode_get(void);

/**
 * @brief Check if a specific module is active
 * 
 * @param mode Mode to check
 * @return true if active, false otherwise
 */
bool config_ble_mode_is_active(ble_module_mode_t mode);

/**
 * @brief Get mode name for logging
 * 
 * @param mode Mode to get name for
 * @return String description of mode
 */
const char *config_ble_mode_name(ble_module_mode_t mode);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_BLE_MODE_H
