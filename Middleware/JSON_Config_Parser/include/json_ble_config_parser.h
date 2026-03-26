/**
 * @file json_ble_config_parser.h
 * @brief BLE-specific JSON configuration parser
 */

#ifndef JSON_BLE_CONFIG_PARSER_H
#define JSON_BLE_CONFIG_PARSER_H

#include "json_config_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Constants
 * ========================================================================== */

#define BLE_MAX_FUNCTIONS 20
#define BLE_COMMAND_LEN 128
#define BLE_RESPONSE_LEN 64

/* ============================================================================
 * Enums
 * ========================================================================== */

/**
 * @brief Hardcoded BLE function IDs (15 core + 5 promoted optional = 20 total)
 * 
 * Core Functions (0-14): Required for basic BLE operation
 * Promoted Optional (15-19): PC App scan/send/discover workflow; optional if not in JSON config
 */
typedef enum {
  JSON_BLE_FUNC_HW_RESET = 0,
  JSON_BLE_FUNC_SW_RESET,
  JSON_BLE_FUNC_FACTORY_RESET,
  JSON_BLE_FUNC_GET_INFO,
  JSON_BLE_FUNC_SET_NAME,
  JSON_BLE_FUNC_SET_COMM_CONFIG,
  JSON_BLE_FUNC_SET_RF_PARAMS,
  JSON_BLE_FUNC_ENTER_CMD_MODE,
  JSON_BLE_FUNC_ENTER_DATA_MODE,
  JSON_BLE_FUNC_START_BROADCAST,
  JSON_BLE_FUNC_CONNECT,
  JSON_BLE_FUNC_DISCONNECT,
  JSON_BLE_FUNC_GET_CONNECTION_STATUS,
  JSON_BLE_FUNC_ENTER_SLEEP,
  JSON_BLE_FUNC_WAKEUP,
  // Promoted Optional Functions (15-19) - PC App scan/send workflow
  JSON_BLE_FUNC_START_DISCOVERY = 15,
  JSON_BLE_FUNC_SEND_DATA,
  JSON_BLE_FUNC_GET_DIAGNOSTICS,
  JSON_BLE_FUNC_DISCOVER_SERVICES,
  JSON_BLE_FUNC_DISCOVER_CHARACTERISTICS,
  JSON_BLE_FUNC_MAX
} json_ble_function_id_t;

/* ============================================================================
 * Structures
 * ========================================================================== */

/**
 * @brief BLE function configuration
 */
typedef struct {
  bool available;
  json_ble_function_id_t function_id;
  char command[BLE_COMMAND_LEN];
  bool is_prefix;
  bool is_hex;                        ///< true = binary/hex, false = ASCII/AT
  gpio_control_t gpio_start[MAX_GPIO_ACTIONS];
  uint8_t gpio_start_count;
  uint16_t delay_start_ms;
  char expect_response[BLE_RESPONSE_LEN];
  uint16_t timeout_ms;
  gpio_control_t gpio_end[MAX_GPIO_ACTIONS];
  uint8_t gpio_end_count;
  uint16_t delay_end_ms;
} json_ble_function_config_t;

/**
 * @brief Complete BLE module configuration
 */
typedef struct {
  module_metadata_t metadata;
  json_ble_function_config_t functions[BLE_MAX_FUNCTIONS];
  uint8_t function_count;
} json_ble_module_config_t;

/**
 * @brief Parse complete BLE module configuration from JSON
 *
 * Parses:
 * - Metadata (via common parser)
 * - BLE functions with validation against hardcoded function names
 *
 * @param json_str JSON string to parse
 * @param config Output BLE configuration
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t json_ble_config_parse(const char *json_str,
                                json_ble_module_config_t *config);

#ifdef __cplusplus
}
#endif

#endif // JSON_BLE_CONFIG_PARSER_H
