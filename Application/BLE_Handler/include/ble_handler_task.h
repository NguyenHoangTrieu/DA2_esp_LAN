/**
 * @file ble_handler_task.h
 * @brief BLE Handler Task - Transportation Layer Gateway
 * 
 * FreeRTOS task that manages BLE device connections and routes data
 * between BLE devices and the server via WAN MCU.
 * 
 * This layer handles:
 * - Device discovery and connection management
 * - Bidirectional data routing (uplink/downlink)
 * - Configuration loading from NVS
 * - PC App command execution
 * 
 * @author Embedded Team
 * @date February 2026
 */

#ifndef BLE_HANDLER_TASK_H
#define BLE_HANDLER_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Type Definitions ===== */

/**
 * @brief Uplink packet from BLE device
 */
typedef struct {
    uint8_t stack_id;                   ///< Stack ID (0 or 1) - NEW for multi-stack support
    uint8_t device_address[6];          ///< Source device MAC
    uint32_t timestamp_ms;              ///< Timestamp
    uint8_t payload[256];               ///< Sensor data or response
    uint16_t payload_len;               ///< Payload length
} ble_uplink_packet_t;

/**
 * @brief Downlink packet to BLE device
 */
typedef struct {
    uint8_t stack_id;                   ///< Stack ID (0 or 1) - NEW for multi-stack support
    uint8_t device_address[6];          ///< Target device MAC
    uint32_t timeout_ms;                ///< Send timeout
    uint8_t payload[256];               ///< Command or data
    uint16_t payload_len;               ///< Payload length
} ble_downlink_packet_t;

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
 * @brief Enqueue uplink data from BLE device to server
 * 
 * Called by BLE handler task when data is received from BLE device.
 * Data will be forwarded to WAN MCU via MCU_WAN_Handler.
 * 
 * @param stack_id Stack ID (0 or 1) - NEW for multi-stack support
 * @param device_address Source device MAC address
 * @param data Sensor data or device response
 * @param len Data length
 * @return true if queued successfully, false if queue full
 */
bool ble_handler_task_enqueue_uplink(uint8_t stack_id,
                                      const uint8_t *device_address,
                                      const uint8_t *data,
                                      uint16_t len);

/**
 * @brief Enqueue downlink data from server to BLE device
 * 
 * Called by MCU_WAN_Handler when command/data arrives from server.
 * Data will be sent to target BLE device.
 * 
 * Format: [Stack ID (1B)][Target MAC (6B)][Data (NB)]
 * 
 * @param data Downlink data (stack_id + MAC + payload)
 * @param len Data length (min 7 bytes for stack_id + MAC)
 * @return true if queued successfully, false if queue full
 */
bool ble_handler_task_enqueue_downlink(const uint8_t *data, uint16_t len);

/**
 * @brief Get connected devices list for specific stack
 * 
 * Returns information about currently connected BLE devices.
 * 
 * @param stack_id Stack ID (0 or 1) - NEW for multi-stack support
 * @param device_count Output device count
 * @param devices Output buffer for device array
 * @return Number of connected devices returned
 */
uint8_t ble_handler_task_get_connected_devices(uint8_t stack_id,
                                                uint8_t *device_count,
                                                uint8_t devices[][6]);

/**
 * @brief Manually trigger device discovery on specific stack
 * 
 * Used by PC App for device scanning. Results will be available
 * via ble_handler_task_get_discovered_devices().
 * 
 * @param stack_id Stack ID (0 or 1) - NEW for multi-stack support
 * @param scan_duration_ms Scan duration in milliseconds
 * @return ESP_OK if scan started
 */
esp_err_t ble_handler_task_start_discovery(uint8_t stack_id, uint32_t scan_duration_ms);

/**
 * @brief Get discovered devices from last scan on specific stack
 * 
 * @param stack_id Stack ID (0 or 1) - NEW for multi-stack support
 * @param devices Output buffer for discovered devices (MAC addresses)
 * @param max_count Maximum number of devices to return
 * @return Number of discovered devices
 */
uint8_t ble_handler_task_get_discovered_devices(uint8_t stack_id,
                                                 uint8_t devices[][6],
                                                 uint8_t max_count);

#ifdef __cplusplus
}
#endif

#endif // BLE_HANDLER_TASK_H
