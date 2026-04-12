/**
 * @file config_handler.h
 * @brief Configuration handler for LAN MCU (Master side)
 */

#ifndef CONFIG_HANDLER_H
#define CONFIG_HANDLER_H

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include <stdbool.h>
#include <stdint.h>

// Command buffer size
// Maximum length for config command data
// Increased to support JSON module configs (~8-16KB typical)
#define CONFIG_CMD_MAX_LEN 16384
#define CONFIG_QUEUE_SIZE 10

/**
 * @brief Command source for ACK routing
 */
typedef enum {
  CONFIG_SOURCE_WAN_MCU = 0,  // From WAN MCU (forward to MQTT/HTTP server)
  CONFIG_SOURCE_UART = 1,      // From PC App via UART (ACK to UART)
  CONFIG_SOURCE_USB = 2,       // From PC App via USB (ACK to USB)
  CONFIG_SOURCE_UNKNOWN = 0xFF
} config_source_t;

/**
 * @brief Command type codes for LAN MCU
 */
typedef enum {
  CONFIG_UPDATE_FIRMWARE = 0,  // "CFFW" - Firmware update command (set URL + trigger)
  CONFIG_SET_FIRMWARE_URL = 17, // "CFFU" - Set firmware URL only (no trigger, saved to NVS)
  CONFIG_UPDATE_LORA = 1,      // "CFLR" - reserved placeholder (use LORA_JSON/CMD below)
  CONFIG_UPDATE_CAN = 2,       // "CFCB" or "CFCM" - CAN config command
  CONFIG_UPDATE_SCAN = 3,      // "CFSC" - Config query command1
  CONFIG_UPDATE_STACK = 4,     // "CFST" - Stack config command
  CONFIG_UPDATE_RS485 = 5,     // "CFRS:BR:" - RS485 baud rate command
  CONFIG_UPDATE_BLE_JSON = 6,  // "CFML:JSON" - BLE JSON config
  CONFIG_UPDATE_BLE_CMD = 7,   // "CFML:<stack>:<cmd>" - Unified BLE command parser
  CONFIG_UPDATE_LORA_JSON    = 8, // "CFLR:JSON" - LoRa JSON config
  CONFIG_UPDATE_LORA_CMD     = 9, // "CFLR:<stack>:<cmd>" - Unified LoRa command parser
  CONFIG_UPDATE_ZIGBEE_JSON  = 10, // "CFZB:JSON" - Zigbee JSON config
  CONFIG_UPDATE_ZIGBEE_CMD   = 11, // "CFZB:<stack>:<func>" - Unified Zigbee command parser
  CONFIG_UPDATE_BLE_NATIVE_JSON = 12, // "CFBN:JSON" - BLE Native (ESP32 direct mesh) JSON config
  CONFIG_UPDATE_BLE_NATIVE_CMD  = 13, // "CFBN:<stack>:<verb>" - BLE Native command
  CONFIG_UPDATE_RS485_JSON = 14,      // "CFRS:JSON:" - RS485 GPIO mode config
  CONFIG_UPDATE_BLE_GATT_JSON  = 15, // "CFBG:JSON" - BLE GATT Central JSON config
  CONFIG_UPDATE_BLE_GATT_CMD   = 16, // "CFBG:<stack>:<verb>" - BLE GATT Central command
  CONFIG_TYPE_UNKNOWN = 0xFF
} config_type_t;

/**
 * @brief Firmware update command structure
 */
typedef struct {
  char url[256];     // Firmware URL (optional, can use default)
  bool force_update; // Force update even if same version
} fota_lan_command_t;

/**
 * @brief Generic configuration command for LAN
 */
typedef struct {
  config_type_t type;
  config_source_t source;  // Command source for ACK routing
  char raw_data[CONFIG_CMD_MAX_LEN];
  uint16_t data_len;
} config_command_t;

// Main config handler queue (receives commands from LAN comm)
// Declared in config_handler.c, used by mcu_lan_handler
extern QueueHandle_t g_config_handler_queue;

/**
 * @brief Start config handler task
 */
void config_handler_task_start(void);

/**
 * @brief Stop config handler task
 */
void config_handler_task_stop(void);

/**
 * @brief Parse command type from raw data
 *
 * @param cmd Command string
 * @param len Command length
 * @return config_type_t Command type
 */
config_type_t config_parse_type(const char *cmd, uint16_t len);

/* ===== Module JSON Config NVS Functions ===== */

/**
 * @brief Save module JSON config to NVS
 * @param stack_id Stack ID (0 or 1)
 * @param json_str JSON config string
 * @param json_len JSON string length
 * @return esp_err_t ESP_OK on success
 */
esp_err_t config_save_module_json_to_nvs(uint8_t stack_id, const char *json_str, uint16_t json_len);

/**
 * @brief Load module JSON config from NVS
 * @param stack_id Stack ID (0 or 1)
 * @param json_str Output: pointer to JSON string (malloc'd, caller must free)
 * @param json_len Output: JSON string length
 * @return esp_err_t ESP_OK on success, ESP_ERR_NOT_FOUND if no config
 */
esp_err_t config_load_module_json_from_nvs(uint8_t stack_id, char **json_str, uint16_t *json_len);

/**
 * @brief Delete module JSON config from NVS for a specific stack
 *
 * Call this when a module swap is detected on boot to prevent a stale
 * config from being applied to the newly installed module.
 *
 * @param stack_id Stack ID (0 or 1)
 * @return esp_err_t ESP_OK on success or if the key was already absent
 */
esp_err_t config_delete_module_json_from_nvs(uint8_t stack_id);

/**
 * @brief Save global config variables to NVS (stack IDs, etc.)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t config_save_global_vars_to_nvs(void);

/**
 * @brief Load global config variables from NVS
 * @return esp_err_t ESP_OK on success
 */
esp_err_t config_load_global_vars_from_nvs(void);

/**
 * @brief Save RS485 baud rate to NVS
 */
esp_err_t config_save_rs485_baud(uint32_t baud_rate);
esp_err_t config_save_fota_lan_url_to_nvs(void);

/* ===== BLE Config NVS Functions ===== */

/**
 * @brief Save BLE JSON config (GATT or Native) and mode to NVS.
 * @param mode     BLE_MODE_GATT (1) or BLE_MODE_NATIVE (2)
 * @param json_str JSON string (without CFBG:JSON: / CFBN:JSON: prefix)
 * @param json_len JSON length
 */
esp_err_t config_save_ble_json_to_nvs(uint8_t mode, const char *json_str, uint16_t json_len);

/**
 * @brief Load BLE JSON config from NVS.
 * @param[out] mode     BLE mode that was saved
 * @param[out] json_str malloc'd buffer — caller must free()
 * @param[out] json_len JSON length
 * @return ESP_OK, ESP_ERR_NOT_FOUND if nothing saved, or error
 */
esp_err_t config_load_ble_json_from_nvs(uint8_t *mode, char **json_str, uint16_t *json_len);

/**
 * @brief Restore BLE handler from NVS on boot.
 *        Calls the appropriate BLE handler init + load_config.
 *        No-op if no BLE config has been saved to NVS yet.
 */
void config_restore_ble_from_nvs(void);

/**
 * @brief Initialize configuration system (load or save defaults)
 */
esp_err_t config_init(void);

#endif // CONFIG_HANDLER_H
