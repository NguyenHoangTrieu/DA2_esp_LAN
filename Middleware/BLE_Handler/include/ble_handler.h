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

/**
 * @brief Streaming response callback (TASK 2.1)
 * 
 * Called for each response received during streaming operations (e.g., SCAN).
 * 
 * @param data Response data buffer
 * @param len Length of response data
 * @param user_data User-provided context pointer
 */
typedef void (*ble_stream_callback_t)(const uint8_t *data, 
                                       uint16_t len, 
                                       void *user_data);

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

/* --- Core Functions (0-14) --- */

/**
 * @brief Execute hardware reset via GPIO
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_hw_reset(uint8_t stack_id);

/**
 * @brief Execute software reset via AT command
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_sw_reset(uint8_t stack_id);

/**
 * @brief Execute factory reset
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_factory_reset(uint8_t stack_id);

/**
 * @brief Get module info (version, MAC, etc.)
 * @param stack_id Stack ID (0 or 1)
 * @param buffer Output buffer for info
 * @param max_len Maximum buffer length
 * @return ESP_OK on success
 */
esp_err_t ble_handler_get_info(uint8_t stack_id, 
                                char *buffer, 
                                size_t max_len);

/**
 * @brief Set device name
 * @param stack_id Stack ID (0 or 1)
 * @param name Device name (max 32 chars)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_set_name(uint8_t stack_id, 
                                const char *name);

/**
 * @brief Configure communication parameters (UART baudrate, SPI mode, etc.)
 * @param stack_id Stack ID (0 or 1)
 * @param config_param Configuration parameter string
 * @return ESP_OK on success
 */
esp_err_t ble_handler_set_comm_config(uint8_t stack_id, 
                                       const char *config_param);

/**
 * @brief Set RF parameters (TX power, frequency, channel)
 * @param stack_id Stack ID (0 or 1)
 * @param rf_param RF parameter string
 * @return ESP_OK on success
 */
esp_err_t ble_handler_set_rf_params(uint8_t stack_id, 
                                     const char *rf_param);

/**
 * @brief Enter AT command mode
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_enter_cmd_mode(uint8_t stack_id);

/**
 * @brief Enter transparent data mode
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_enter_data_mode(uint8_t stack_id);

/**
 * @brief Start broadcasting (advertising in peripheral mode)
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_start_broadcast(uint8_t stack_id);

/**
 * @brief Connect to remote BLE device
 * @param stack_id Stack ID (0 or 1)
 * @param address Remote device address (MAC format)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_connect(uint8_t stack_id, 
                               const char *address);

/**
 * @brief Disconnect from current device
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_disconnect(uint8_t stack_id);

/**
 * @brief Get connection status and signal strength
 * @param stack_id Stack ID (0 or 1)
 * @param buffer Output buffer for status
 * @param max_len Maximum buffer length
 * @return ESP_OK on success
 */
esp_err_t ble_handler_get_connection_status(uint8_t stack_id, 
                                             char *buffer, 
                                             size_t max_len);

/**
 * @brief Enter low-power sleep mode
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_enter_sleep(uint8_t stack_id);

/**
 * @brief Wake from sleep mode
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_wakeup(uint8_t stack_id);

/* --- Optional Functions (15-19) --- */

/**
 * @brief Start BLE device discovery (scan)
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not configured
 */
esp_err_t ble_handler_start_discovery(uint8_t stack_id);

/**
 * @brief Send data in transparent mode
 * @param stack_id Stack ID (0 or 1)
 * @param data Data to send
 * @param len Data length
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not configured
 */
esp_err_t ble_handler_send_data(uint8_t stack_id, 
                                 const uint8_t *data, 
                                 uint16_t len);

/**
 * @brief Get diagnostics (RSSI, link quality, etc.)
 * @param stack_id Stack ID (0 or 1)
 * @param buffer Output buffer for diagnostics
 * @param max_len Maximum buffer length
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not configured
 */
esp_err_t ble_handler_get_diagnostics(uint8_t stack_id, 
                                       char *buffer, 
                                       size_t max_len);

/**
 * @brief Configure security (pairing, bonding, PIN)
 * @param stack_id Stack ID (0 or 1)
 * @param security_param Security parameter string
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not configured
 */
esp_err_t ble_handler_set_security(uint8_t stack_id, 
                                    const char *security_param);

/**
 * @brief Manage whitelist (add/remove device MAC)
 * @param stack_id Stack ID (0 or 1)
 * @param mac_address Device MAC address
 * @param add true=add to whitelist, false=remove
 * @return ESP_OK on success, ESP_ERR_NOT_SUPPORTED if not configured
 */
esp_err_t ble_handler_manage_whitelist(uint8_t stack_id, 
                                        const char *mac_address, 
                                        bool add);

/* ===== Streaming Mode API (TASK 2.1) ===== */

/**
 * @brief Execute BLE function with streaming response support
 * 
 * Used for commands that generate multiple responses over time (e.g., SCAN).
 * The callback will be invoked for each response received during the stream_duration_ms.
 * 
 * @param stack_id Stack ID (0 or 1)
 * @param func_id Function ID to execute
 * @param param Optional parameter string (NULL if not needed)
 * @param stream_duration_ms Duration to collect responses (milliseconds)
 * @param callback Function to call for each response
 * @param user_data User context passed to callback
 * @return ESP_OK on success, ESP_ERR_* on failure
 */
esp_err_t ble_execute_function_streaming(uint8_t stack_id,
                                         ble_function_id_t func_id,
                                         const char *param,
                                         uint32_t stream_duration_ms,
                                         ble_stream_callback_t callback,
                                         void *user_data);

/* ===== Device Management APIs (NEW - Task 1.1) ===== */

/**
 * @brief Add a BLE device to tracked list
 * @param stack_id Stack ID (0 or 1)
 * @param mac_address Device MAC address (6 bytes)
 * @param device_name Optional device name
 * @return ESP_OK on success, ESP_ERR_NO_MEM if list full
 */
esp_err_t ble_handler_add_device(uint8_t stack_id,
                                  const uint8_t *mac_address,
                                  const char *device_name);

/**
 * @brief Remove a BLE device from tracked list
 * @param stack_id Stack ID (0 or 1)
 * @param mac_address Device MAC address (6 bytes)
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if not tracked
 */
esp_err_t ble_handler_remove_device(uint8_t stack_id,
                                     const uint8_t *mac_address);

/**
 * @brief Get number of tracked devices
 * @param stack_id Stack ID (0 or 1)
 * @return Device count, or 0 if invalid stack
 */
uint8_t ble_handler_get_device_count(uint8_t stack_id);

/**
 * @brief Get device info by MAC address
 * @param stack_id Stack ID (0 or 1)
 * @param mac_address Device MAC address (6 bytes)
 * @param device_out Output device structure
 * @return ESP_OK on success, ESP_ERR_NOT_FOUND if not tracked
 */
esp_err_t ble_handler_get_device(uint8_t stack_id,
                                  const uint8_t *mac_address,
                                  ble_device_t *device_out);

/**
 * @brief Update device last activity timestamp
 * @param stack_id Stack ID (0 or 1)
 * @param mac_address Device MAC address (6 bytes)
 * @return ESP_OK on success
 */
esp_err_t ble_handler_update_device_activity(uint8_t stack_id,
                                              const uint8_t *mac_address);

/* ===== Enhanced Features (NEW - Task 1.1 Approved Enhancements) ===== */

/**
 * @brief Execute function with automatic retry on timeout
 * 
 * Implements automatic recovery:
 * - Retry command up to 3 times on timeout
 * - Fallback to SW reset if all retries fail
 * - Fallback to HW reset if SW reset fails
 * 
 * @param stack_id Stack ID (0 or 1)
 * @param func_id Function ID to execute
 * @param param Optional parameter
 * @param result Output execution result
 * @return ESP_OK on success, ESP_FAIL if all recovery attempts fail
 */
esp_err_t ble_handler_execute_with_recovery(uint8_t stack_id,
                                             ble_function_id_t func_id,
                                             const char *param,
                                             ble_exec_result_t *result);

/**
 * @brief Parse incoming frame from BLE device
 * 
 * Extracts MAC address and payload from UART/SPI frame.
 * Supports both ASCII and binary protocols.
 * 
 * @param data Raw frame data
 * @param len Frame length
 * @param mac_out Output MAC address (6 bytes)
 * @param payload_out Output payload buffer
 * @param payload_len_out Output payload length
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if parse fails
 */
esp_err_t ble_handler_parse_frame(const uint8_t *data,
                                   uint16_t len,
                                   uint8_t *mac_out,
                                   uint8_t *payload_out,
                                   uint16_t *payload_len_out);

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

/**
 * @brief Send raw command (alias for ble_handler_send_binary_command)
 * 
 * Pass-through API for commands in any format (AT, binary, ASCII).
 * Module interprets based on its configuration.
 * 
 * @param stack_id Stack ID (0 or 1)
 * @param command Raw command bytes
 * @param cmd_len Command length
 * @param response Response buffer
 * @param resp_len Pointer to response length (in/out)
 * @param timeout_ms Timeout in milliseconds
 * @return ESP_OK on success
 */
static inline esp_err_t ble_send_raw_command(uint8_t stack_id,
                                             const uint8_t *command,
                                             uint16_t cmd_len,
                                             uint8_t *response,
                                             uint16_t *resp_len,
                                             uint16_t timeout_ms) {
    return ble_handler_send_binary_command(stack_id, command, cmd_len,
                                           response, *resp_len, timeout_ms);
}

/**
 * @brief Send raw command with streaming response support (NEW - Phase 3)
 * 
 * Pass-through API for commands that generate multiple responses.
 * Command format is module-specific (AT/binary/ASCII), not interpreted.
 * 
 * @param stack_id Stack ID (0 or 1)
 * @param command Raw command bytes  
 * @param cmd_len Command length
 * @param duration_ms Duration to collect responses
 * @param callback Function called for each response line
 * @param user_data User context passed to callback
 * @return ESP_OK on success
 */
esp_err_t ble_send_raw_command_streaming(uint8_t stack_id,
                                         const uint8_t *command,
                                         uint16_t cmd_len,
                                         uint32_t duration_ms,
                                         ble_stream_callback_t callback,
                                         void *user_data);

/* ===== Internal Helpers (for task layer) ===== */

/**
 * @brief Execute a function internally (used by task layer)
 * @param stack_id Stack ID
 * @param func_id Function ID
 * @param param Optional parameter for functions that need it
 * @param result Output execution result
 * @return ESP_OK on success
 */
esp_err_t ble_handler_execute_function(uint8_t stack_id,
                                        ble_function_id_t func_id,
                                        const char *param,
                                        ble_exec_result_t *result);

#ifdef __cplusplus
}
#endif

#endif // BLE_HANDLER_H
