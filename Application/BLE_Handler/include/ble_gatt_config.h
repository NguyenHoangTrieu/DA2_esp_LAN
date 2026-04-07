/**
 * @file ble_gatt_config.h
 * @brief BLE GATT Central config — all parameters loaded from JSON at runtime.
 *
 * Nothing is hardcoded to a specific device type.  The server provides the
 * JSON configuration via "CFBG:JSON:<slot>:<json>" and all scan/connection
 * parameters are taken from that config.
 *
 * Prefix: CFBG  (CF + BLE Gatt Central)
 * Handler ID: HANDLER_BLE_GATT (0x07)
 */

#ifndef BLE_GATT_CONFIG_H
#define BLE_GATT_CONFIG_H

#include "esp_err.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Sizing constants
 * -------------------------------------------------------------------------- */
#define BLE_GATT_MAX_STACKS      2   /**< Parallel GATT Central stacks */
#define BLE_GATT_MAX_DEVICES     32  /**< Scanned / connected device slots (PSRAM) */
#define BLE_GATT_MAX_CHARS       32  /**< Per-device characteristic cache */
#define BLE_GATT_DEV_NAME_LEN    32  /**< Max device name length          */
#define BLE_GATT_MAX_SERVICES    12  /**< Per-device service cache         */

/* --------------------------------------------------------------------------
 * Scan parameters (from JSON "scan" object)
 * -------------------------------------------------------------------------- */
typedef struct {
    uint16_t interval;   /**< scan interval ×0.625 ms  (default 160 = 100 ms) */
    uint16_t window;     /**< scan window   ×0.625 ms  (default  80 =  50 ms) */
    bool     active;     /**< true = active scanning (includes scan-response)  */
} ble_gatt_scan_cfg_t;

/* --------------------------------------------------------------------------
 * Connection parameters (from JSON "connection" object)
 * -------------------------------------------------------------------------- */
typedef struct {
    uint16_t interval_min;        /**< ×1.25 ms (default  16 =  20 ms) */
    uint16_t interval_max;        /**< ×1.25 ms (default  32 =  40 ms) */
    uint16_t latency;             /**< peripheral latency    (default   0) */
    uint16_t supervision_timeout; /**< ×10 ms   (default 500 = 5000 ms) */
} ble_gatt_conn_cfg_t;

/* --------------------------------------------------------------------------
 * Per-stack runtime configuration
 * -------------------------------------------------------------------------- */
typedef struct {
    uint8_t             stack_id;
    ble_gatt_scan_cfg_t scan;
    ble_gatt_conn_cfg_t connection;
    bool                loaded;
} ble_gatt_stack_config_t;

/* --------------------------------------------------------------------------
 * Characteristic cache entry
 * -------------------------------------------------------------------------- */
typedef struct {
    uint16_t handle;
    uint16_t uuid16;   /**< 16-bit UUID shortform (0 if 128-bit only) */
    uint8_t  uuid128[16];
    uint8_t  properties;
    bool     valid;
} ble_gatt_char_entry_t;

/* --------------------------------------------------------------------------
 * Service cache entry
 * -------------------------------------------------------------------------- */
typedef struct {
    uint16_t start_handle;
    uint16_t end_handle;
    uint16_t uuid16;
    uint8_t  uuid128[16];
    bool     valid;
} ble_gatt_svc_entry_t;

/* --------------------------------------------------------------------------
 * Per-device entry (index 0-7, mirroring AT+LIST from STM32 module)
 * -------------------------------------------------------------------------- */
typedef struct {
    esp_bd_addr_t       addr;
    esp_ble_addr_type_t addr_type;
    int8_t              rssi;
    char                name[BLE_GATT_DEV_NAME_LEN];
    uint16_t            conn_id;    /**< 0xFFFF = not connected */
    esp_gatt_if_t       gattc_if;
    uint8_t             stack_id;
    ble_gatt_svc_entry_t  services[BLE_GATT_MAX_SERVICES];
    ble_gatt_char_entry_t chars[BLE_GATT_MAX_CHARS];
    uint8_t             num_chars;
    uint8_t             num_services;
    bool                valid;
} ble_gatt_device_t;

/* --------------------------------------------------------------------------
 * API
 * -------------------------------------------------------------------------- */

/**
 * @brief Load JSON configuration for one GATT Central stack.
 *
 * Expected JSON shape:
 * @code
 * {
 *   "stack_id": "008",
 *   "stack_type": "esp32_native_ble_gatt",
 *   "ble_gatt": {
 *     "scan": {
 *       "interval": 160,
 *       "window":    80,
 *       "active":  true
 *     },
 *     "connection": {
 *       "interval_min":         16,
 *       "interval_max":         32,
 *       "latency":               0,
 *       "supervision_timeout": 500
 *     }
 *   }
 * }
 * @endcode
 *
 * @param stack_id   Stack index (0 or 1)
 * @param json_str   Null-terminated JSON string
 * @param json_len   Length in bytes
 * @return ESP_OK on success
 */
esp_err_t ble_gatt_config_load(uint8_t stack_id,
                                const char *json_str,
                                uint16_t json_len);

/**
 * @brief Get the configuration for one stack.
 * @return Pointer to config (never NULL), check .loaded flag.
 */
ble_gatt_stack_config_t *ble_gatt_config_get(uint8_t stack_id);

/**
 * @brief Return true if JSON has been loaded for the given stack.
 */
bool ble_gatt_config_is_loaded(uint8_t stack_id);

#ifdef __cplusplus
}
#endif

#endif /* BLE_GATT_CONFIG_H */
