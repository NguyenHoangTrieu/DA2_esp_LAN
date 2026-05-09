/**
 * @file ble_native_downlink.h
 * @brief BLE Native downlink — receive CFBN: commands and execute mesh operations.
 *
 * Command format expected by this module:
 *   "CFBN:<stack_id>:<verb>:<json_params>"
 *
 * Supported verbs (all configurable — actual command names come from JSON config):
 *   SCAN         — start provisioner scan for unprovisioned beacons
 *   PROVISION    — provision a specific device UUID
 *   CONTROL      — send a mesh model message (uses JSON param_schema from config)
 *   NODE_LIST    — report all known provisioned nodes
 *   STATUS       — send a mesh GET to one node and wait for status reply
 *
 * The CONTROL verb's payload must be JSON:
 *   { "cmd": "<cmd_name>", "addr": "<unicast_hex>", "params": { ... } }
 *
 * where "cmd" matches an entry in the JSON config command table.
 */

#ifndef BLE_NATIVE_DOWNLINK_H
#define BLE_NATIVE_DOWNLINK_H

#include "esp_err.h"
#include "frame_types.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Queue depth for incoming commands */
#define BLE_NATIVE_DOWNLINK_QUEUE_DEPTH  8

/* Maximum length of one downlink command item (verb + JSON params) */
#define BLE_NATIVE_DOWNLINK_ITEM_MAX     INTER_MCU_PAYLOAD_MAX_LEN

/**
 * @brief Start the downlink task.
 *        Must be called once from ble_native_handler_init().
 */
esp_err_t ble_native_downlink_task_start(void);

/**
 * @brief Stop and delete the downlink task.
 */
void ble_native_downlink_task_stop(void);

/**
 * @brief Enqueue a raw CFBN: command string for execution.
 *
 * The full command string (e.g. "CFBN:0:CONTROL:{...}") is passed here.
 * The task parses it and dispatches via the BLE Mesh app layer.
 *
 * @param data   Command data buffer (does NOT need null terminator)
 * @param len    Length in bytes
 * @return ESP_OK on success, ESP_ERR_NO_MEM if queue full
 */
esp_err_t ble_native_downlink_enqueue(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* BLE_NATIVE_DOWNLINK_H */
