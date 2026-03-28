/**
 * @file zigbee_handler_task.h
 * @brief Zigbee Handler Task – Application Layer
 *
 * Mirrors lora_handler_task.h with Zigbee-specific differences:
 *  - Response prefix "CFZB:" (vs "CFLR:")
 *  - All commands use ASCII AT format (unified with BLE/LoRa)
 *  - Listener forwards ASCII async events (e.g. +JOIN:, +LEFT:, +ATTRREPORT:)
 *  - Startup: HW_RESET → 500 ms → GET_INFO
 */

#ifndef ZIGBEE_HANDLER_TASK_H
#define ZIGBEE_HANDLER_TASK_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "zigbee_handler.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Type Definitions ===== */

/**
 * @brief Uplink packet from Zigbee module (async events / command responses).
 */
typedef struct {
    uint8_t  stack_id;
    uint32_t timestamp_ms;
    uint8_t  payload[256];
    uint16_t payload_len;
} zigbee_uplink_packet_t;

/**
 * @brief Command execution request (from config handler).
 *
 * func_id selects the function from the JSON config.
 * data[] carries optional binary payload appended to the HEX frame.
 */
typedef struct {
    uint8_t              stack_id;
    zigbee_function_id_t func_id;
    uint8_t              data[252];     ///< Binary payload (e.g. addr + cluster bytes)
    uint8_t              data_len;
} zigbee_command_request_t;

/* ===== Public API ===== */

/**
 * @brief Start Zigbee handler tasks for a specific stack.
 *
 * Creates three FreeRTOS tasks:
 *  - Uplink task   (priority 5): batches responses → WAN MCU
 *  - Downlink task (priority 6): executes commands from server
 *  - Listener task (priority 4): captures unsolicited async events
 *
 * @param stack_id Stack ID (0 or 1)
 * @return ESP_OK on success
 */
esp_err_t zigbee_handler_task_start(uint8_t stack_id);

/**
 * @brief Check whether Zigbee handler tasks are running.
 */
bool zigbee_handler_is_running(uint8_t stack_id);

/**
 * @brief Gracefully stop Zigbee handler tasks.
 */
esp_err_t zigbee_handler_task_stop(uint8_t stack_id);

/**
 * @brief Load JSON configuration and initialise the Zigbee module.
 *
 * Sequence:
 *  1. zigbee_handler_load_config()
 *  2. MODULE_HW_RESET  (gpio NRST)
 *  3. vTaskDelay(500 ms)
 *  4. MODULE_ENTER_HEX_MODE  (AT+EXIT\r\n)
 *  5. vTaskDelay(200 ms)
 *  6. MODULE_GET_INFO
 *
 * @param stack_id   Stack ID (0 or 1)
 * @param json_config JSON string
 * @param len         Length in bytes
 */
esp_err_t zigbee_handler_task_load_config(uint8_t stack_id,
                                           const char *json_config,
                                           uint16_t len);

/**
 * @brief Enqueue a command request to the downlink task.
 */
esp_err_t zigbee_handler_task_execute_command(const zigbee_command_request_t *req);

/**
 * @brief Enqueue uplink data to the uplink task queue.
 */
bool zigbee_handler_task_enqueue_uplink(uint8_t stack_id,
                                         const uint8_t *data,
                                         uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* ZIGBEE_HANDLER_TASK_H */
