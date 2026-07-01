/**
 * @file ble_handler_task.h
 * @brief BLE Handler Task - Transportation Layer Gateway
 */

#ifndef BLE_HANDLER_TASK_H
#define BLE_HANDLER_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "ble_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Type Definitions ===== */

/**
 * @brief Uplink packet from BLE device
 */
typedef struct {
    uint8_t stack_id;                   ///< Stack ID (0 or 1) - NEW for multi-stack support
    uint32_t timestamp_ms;              ///< Timestamp
    uint8_t payload[256];               ///< Sensor data or response
    uint16_t payload_len;               ///< Payload length
} ble_uplink_packet_t;

/**
 * @brief Downlink packet to BLE device
 */
typedef struct {
    uint8_t stack_id;                   ///< Stack ID (0 or 1) - NEW for multi-stack support
    uint32_t timeout_ms;                ///< Send timeout
    uint8_t payload[256];               ///< Command or data
    uint16_t payload_len;               ///< Payload length
} ble_downlink_packet_t;

/**
 * @brief BLE command execution request (from config handler)
 * 
 * Reuses ble_function_config_t from middleware to avoid field duplication.
 * Config handler enqueues this after matching command prefix from JSON.
 */
typedef struct {
    uint8_t stack_id;                       ///< Stack ID (0 or 1)
    char command[256];                      ///< Full command string from server
    uint16_t command_len;                   ///< Command length
    bool is_streaming;                      ///< true=streaming responses, false=single response
    ble_function_config_t func_config;      ///< Embedded function config (GPIO/delays/timeout)
} ble_command_request_t;

/* ===== Public API Functions ===== */

/**
 * @brief Start BLE handler task for specific stack
 * 
 * Creates FreeRTOS task for BLE management. Task will:
 * 1. Initialize BLE handler middleware
 * 2. Load configuration from NVS
 * 3. Initialize BLE module via hardware reset
 * 4. Enter main loop for data processing
 * 
 * @param stack_id Stack ID (0 or 1) - NEW for multi-stack support
 * @return ESP_OK on success
 */
esp_err_t ble_handler_task_start(uint8_t stack_id);

/**
 * @brief Check if BLE handler is running for specific stack
 * 
 * @param stack_id Stack ID (0 or 1)
 * @return true if running, false otherwise
 */
bool ble_handler_is_running(uint8_t stack_id);

/**
 * @brief Stop BLE handler task for specific stack
 * 
 * Gracefully shuts down the task:
 * 1. Disconnect all BLE devices
 * 2. Clear resources
 * 3. Delete task
 * 
 * @param stack_id Stack ID (0 or 1) - NEW for multi-stack support
 * @return ESP_OK on success
 */
esp_err_t ble_handler_task_stop(uint8_t stack_id);

/**
 * @brief Load JSON configuration for BLE handler
 * 
 * Called by config_handler after receiving JSON from PC app via WAN MCU.
 * 
 * @param stack_id Stack ID (0 or 1) - NEW for multi-stack support
 * @param json_config JSON configuration string
 * @param len JSON length
 * @return ESP_OK on success
 */
esp_err_t ble_handler_task_load_config(uint8_t stack_id, const char *json_config, uint16_t len);

/**
 * @brief Execute BLE command with full GPIO/delay/timeout control
 * 
 * Called by config_handler after parsing command and matching with JSON config.
 * Command is enqueued to BLE task for execution.
 * 
 * @param request Command request packet
 * @return ESP_OK if enqueued successfully
 */
esp_err_t ble_handler_task_execute_command(const ble_command_request_t *request);

/**
 * @brief Enqueue uplink data from BLE device to server
 * 
 * Called by BLE handler task when data is received from BLE device.
 * Data will be forwarded to WAN MCU via MCU_WAN_Handler.
 * 
 * Gateway doesn't care about individual device MAC addresses - it only
 * manages communication with the BLE module itself.
 * 
 * @param stack_id Stack ID (0 or 1) - NEW for multi-stack support
 * @param data Sensor data or device response
 * @param len Data length
 * @return true if queued successfully, false if queue full
 */
bool ble_handler_task_enqueue_uplink(uint8_t stack_id,
                                      const uint8_t *data,
                                      uint16_t len);

/**
 * @brief Enqueue downlink data from server to BLE device
 * 
 * Called by MCU_WAN_Handler when command/data arrives from server.
 * Data will be sent to BLE module.
 * 
 * Gateway doesn't route to specific devices - it forwards data to the
 * BLE module which handles device-level routing.
 * 
 * Format: [Stack ID (1B)][Data (NB)]
 * 
 * @param data Downlink data (stack_id + payload)
 * @param len Data length (min 1 byte for stack_id)
 * @return true if queued successfully, false if queue full
 */
bool ble_handler_task_enqueue_downlink(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // BLE_HANDLER_TASK_H
