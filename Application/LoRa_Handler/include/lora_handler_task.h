/**
 * @file lora_handler_task.h
 * @brief LoRa Handler Task – Transportation Layer Gateway
 */

#ifndef LORA_HANDLER_TASK_H
#define LORA_HANDLER_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "frame_types.h"
#include "lora_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LORA_UPLINK_PAYLOAD_MAX_LEN INTER_MCU_PAYLOAD_MAX_LEN
#define LORA_DOWNLINK_PAYLOAD_MAX_LEN INTER_MCU_PAYLOAD_MAX_LEN

/* ===== Type Definitions ===== */

/**
 * @brief Uplink packet from LoRa device
 */
typedef struct {
    uint8_t  stack_id;          ///< Stack ID (0 or 1)
    uint32_t timestamp_ms;      ///< Timestamp (ms since boot)
    uint8_t  payload[LORA_UPLINK_PAYLOAD_MAX_LEN];      ///< Sensor data or command response
    uint16_t payload_len;       ///< Payload length
} lora_uplink_packet_t;

/**
 * @brief Downlink packet to LoRa device
 */
typedef struct {
    uint8_t  stack_id;          ///< Stack ID (0 or 1)
    uint32_t timeout_ms;        ///< Send timeout
    uint8_t  payload[LORA_DOWNLINK_PAYLOAD_MAX_LEN];      ///< Command or binary data
    uint16_t payload_len;       ///< Payload length
} lora_downlink_packet_t;

/**
 * @brief LoRa command execution request (from config handler)
 *
 * Reuses lora_function_config_t from middleware to avoid field duplication.
 * Config handler enqueues this after matching command prefix from JSON.
 */
typedef struct {
    uint8_t  stack_id;                   ///< Stack ID (0 or 1)
    char     command[256];               ///< Full command string from server
    uint16_t command_len;                ///< Command length
    bool     is_streaming;               ///< true = streaming responses
    lora_function_config_t func_config;  ///< Embedded function config (GPIO/delays/timeout)
} lora_command_request_t;

/* ===== Public API ===== */

/**
 * @brief Start LoRa handler tasks for a specific stack.
 *
 * Creates three FreeRTOS tasks:
 *  - Uplink task  (priority 5): batches data from LoRa module → WAN MCU
 *  - Downlink task (priority 6): executes commands from server
 *  - Listener task (priority 4): captures unsolicited events
 *
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t lora_handler_task_start(uint8_t stack_id);

/**
 * @brief Check whether LoRa handler tasks are running for a specific stack.
 *
 * @param stack_id Stack ID (0 or 1)
 * @return true if running
 */
bool lora_handler_is_running(uint8_t stack_id);

/**
 * @brief Gracefully stop LoRa handler tasks for a specific stack.
 *
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t lora_handler_task_stop(uint8_t stack_id);

/**
 * @brief Load JSON configuration and run the LoRa module startup sequence.
 *
 * Calls lora_handler_load_config() then performs:
 *  1. lora_handler_hw_reset()
 *  2. vTaskDelay(500 ms)
 *  3. lora_handler_get_info()
 *
 * Note: There is no "enter_cmd_mode" for LoRa modules – they are always in
 * command mode after reset.
 *
 * @param stack_id   Stack ID (0 or 1)
 * @param json_config JSON configuration string
 * @param len         JSON length in bytes
 * @return ESP_OK on success
 */
esp_err_t lora_handler_task_load_config(uint8_t stack_id,
                                         const char *json_config,
                                         uint16_t len);

/**
 * @brief Execute a LoRa command request (enqueue to downlink task).
 *
 * @param request Pointer to filled lora_command_request_t
 * @return ESP_OK if enqueued, ESP_ERR_TIMEOUT if queue full
 */
esp_err_t lora_handler_task_execute_command(const lora_command_request_t *request);

/**
 * @brief Enqueue uplink data from LoRa module to server.
 *
 * @param stack_id Stack ID (0 or 1)
 * @param data     Data buffer
 * @param len      Data length in bytes
 * @return true if queued successfully
 */
bool lora_handler_task_enqueue_uplink(uint8_t stack_id,
                                       const uint8_t *data,
                                       uint16_t len);

/**
 * @brief Enqueue downlink data from server to LoRa module.
 *
 * Format: [Stack ID (1 B)][Payload (N B)]
 *
 * @param data Raw downlink bytes (first byte = stack_id)
 * @param len  Total length (min 1 byte)
 * @return true if queued successfully
 */
bool lora_handler_task_enqueue_downlink(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif // LORA_HANDLER_TASK_H
