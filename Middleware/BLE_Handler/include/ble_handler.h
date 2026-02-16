/**
 * @file ble_handler.h
 * @brief BLE Handler Middleware - Transportation Layer Gateway
 */

#ifndef BLE_HANDLER_H
#define BLE_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Type Definitions ===== */

/**
 * @brief BLE Module Function Identifiers (20 functions total)
 */
typedef enum {
    /* --- Core Functions (0-14) --- */
    BLE_FUNC_HW_RESET = 0,              ///< Hardware reset via GPIO
    BLE_FUNC_SW_RESET = 1,              ///< Software reset via command
    BLE_FUNC_FACTORY_RESET = 2,         ///< Factory reset (full config wipe)
    BLE_FUNC_GET_INFO = 3,              ///< Get module version/info/MAC
    BLE_FUNC_SET_NAME = 4,              ///< Set device name
    BLE_FUNC_SET_COMM_CONFIG = 5,       ///< Configure UART/SPI/I2C parameters
    BLE_FUNC_SET_RF_PARAMS = 6,         ///< Set TX power, frequency
    BLE_FUNC_ENTER_CMD_MODE = 7,        ///< Enter AT command mode
    BLE_FUNC_ENTER_DATA_MODE = 8,       ///< Enter transparent data mode
    BLE_FUNC_START_BROADCAST = 9,       ///< Start advertising (peripheral mode)
    BLE_FUNC_CONNECT = 10,              ///< Connect to remote device
    BLE_FUNC_DISCONNECT = 11,           ///< Disconnect from device
    BLE_FUNC_GET_CONNECTION_STATUS = 12,///< Check connection status & RSSI
    BLE_FUNC_ENTER_SLEEP = 13,          ///< Enter low-power mode
    BLE_FUNC_WAKEUP = 14,               ///< Wake from sleep
    
    /* --- Optional Functions (15-19) --- */
    BLE_FUNC_START_DISCOVERY = 15,      ///< Scan for BLE devices (optional)
    BLE_FUNC_SEND_DATA = 16,            ///< Send data in transparent mode (optional)
    BLE_FUNC_GET_DIAGNOSTICS = 17,      ///< Get RSSI, link quality (optional)
    BLE_FUNC_SET_SECURITY_CONFIG = 18,  ///< Configure security/pairing (optional)
    BLE_FUNC_ENTER_BOOTLOADER = 19,     ///< Enter bootloader mode (optional)
    
    BLE_FUNC_COUNT = 20,                ///< Total number of functions
    BLE_FUNC_INVALID = 0xFF
} ble_function_id_t;

/**
 * @brief BLE Device Connection State
 */
typedef struct {
    uint8_t mac_address[6];             ///< BLE device MAC address
    char device_name[32];               ///< Optional device name
    bool connected;                     ///< Connection state
    int8_t rssi;                        ///< Received Signal Strength Indicator
    uint32_t last_activity_ms;          ///< Timestamp of last activity
} ble_device_t;

/**
 * @brief Function configuration from JSON
 */
typedef struct {
    bool available;                     ///< Is function available in JSON config
    char command[128];                  ///< AT command or binary command
    uint8_t gpio_start[8];              ///< GPIO pins to control before command
    uint8_t gpio_start_state[8];        ///< GPIO states (0=LOW, 1=HIGH)
    uint8_t gpio_start_count;           ///< Number of GPIO controls
    uint32_t delay_start_ms;            ///< Delay before command
    char expect_response[64];           ///< Expected response string
    uint32_t timeout_ms;                ///< Command timeout
    uint8_t gpio_end[8];                ///< GPIO pins to control after command
    uint8_t gpio_end_state[8];          ///< GPIO states
    uint8_t gpio_end_count;             ///< Number of GPIO controls
    uint32_t delay_end_ms;              ///< Delay after command
} ble_function_config_t;

/**
 * @brief BLE module configuration (loaded from JSON)
 */
typedef struct {
    uint8_t module_id;                  ///< 0x00 or 0x01 (stack ID)
    char module_type[32];               ///< "BLE"
    char module_name[32];               ///< "JDY-23" or similar
    char comm_port_type[16];            ///< "uart", "spi", "i2c"
    uint32_t baudrate;                  ///< For UART communication
    ble_function_config_t functions[BLE_FUNC_COUNT]; ///< All 20 functions
} ble_module_config_t;

/**
 * @brief Function execution result
 */
typedef struct {
    esp_err_t status;                   ///< Execution status
    char response[256];                 ///< Command response
    uint16_t response_len;              ///< Response length
    uint32_t execution_time_ms;         ///< Total execution time
} ble_exec_result_t;

/* ===== Public API Functions ===== */

/**
 * @brief Initialize BLE handler middleware
 * 
 * Must be called before any other BLE handler functions.
 * 
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t ble_handler_init(void);

/**
 * @brief Load JSON configuration for BLE module
 * 
 * @param stack_id Stack ID (0 or 1)
 * @param json_config JSON configuration string
 * @param json_len JSON string length
 * @return ESP_OK on success, ESP_FAIL if JSON invalid
 */
esp_err_t ble_handler_load_config(uint8_t stack_id, 
                                   const char *json_config, 
                                   uint16_t json_len);

/* --- Core Non-Prefix Functions (for baseboard initialization) --- */

/**
 * @brief Hardware reset BLE module
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_hw_reset(uint8_t stack_id);

/**
 * @brief Software reset BLE module
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_sw_reset(uint8_t stack_id);

/**
 * @brief Factory reset BLE module
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_factory_reset(uint8_t stack_id);

/**
 * @brief Get BLE module info
 * @param stack_id Stack ID (0 or 1)
 * @param buffer Output buffer for info string
 * @param max_len Maximum buffer length
 * @return ESP_OK on success
 */
esp_err_t ble_handler_get_info(uint8_t stack_id, char *buffer, size_t max_len);

/**
 * @brief Enter command mode
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_enter_cmd_mode(uint8_t stack_id);

/**
 * @brief Get connection status
 * @param stack_id Stack ID (0 or 1)
 * @param buffer Output buffer for status string
 * @param max_len Maximum buffer length
 * @return ESP_OK on success
 */
esp_err_t ble_handler_get_connection_status(uint8_t stack_id, char *buffer, size_t max_len);

/**
 * @brief Enter sleep mode
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_enter_sleep(uint8_t stack_id);

/**
 * @brief Wakeup from sleep mode
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_wakeup(uint8_t stack_id);

/* --- Command Matching & Execution Functions (for prefix commands) --- */

/**
 * @brief Get function configuration by matching command prefix
 * 
 * Searches through loaded BLE config to find function matching the command.
 * Used by config handler to build command execution requests with proper
 * GPIO controls and delays.
 * 
 * @param stack_id Stack ID (0 or 1)
 * @param command Command string to match (prefix or exact)
 * @param func_config Output buffer for matched function config
 * @return ESP_OK if matched, ESP_ERR_NOT_FOUND if no match
 */
esp_err_t ble_handler_get_function_by_command(uint8_t stack_id,
                                               const char *command,
                                               ble_function_config_t *func_config);

/**
 * @brief Execute command with pre-matched function config (for task layer)
 * 
 * Executes command using function_config already matched by config handler.
 * Logic identical to ble_execute_function_internal but takes config directly.
 * 
 * Used by task layer after config handler calls ble_handler_get_function_by_command().
 * 
 * @param stack_id Stack ID (0 or 1)
 * @param command Raw command string (e.g., "AT+SCAN=5000")
 * @param func_config Function config from JSON (GPIO, delays, timeout)
 * @param result Output execution result
 * @return ESP_OK on success
 */
esp_err_t ble_handler_execute_command_with_config(uint8_t stack_id,
                                                   const char *command,
                                                   const ble_function_config_t *func_config,
                                                   ble_exec_result_t *result);

/**
 * @brief Send command with explicit binary format
 * 
 * For modules that use binary protocol (e.g., 0xC0 0xC0 prefix).
 * Can also handle AT commands or ASCII - format-agnostic.
 * 
 * @param stack_id Stack ID (0 or 1)
 * @param cmd_bytes Command bytes (binary/AT/ASCII)
 * @param cmd_len Command length
 * @param response Response buffer
 * @param resp_len Response buffer size
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
esp_err_t ble_handler_send_binary_command(uint8_t stack_id,
                                           const uint8_t *cmd_bytes,
                                           uint16_t cmd_len,
                                           uint8_t *response,
                                           uint16_t resp_len,
                                           uint16_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif // BLE_HANDLER_H
