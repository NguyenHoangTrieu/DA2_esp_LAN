/**
 * @file ble_native_config.h
 * @brief BLE Native Mesh Config — runtime structures loaded from server JSON
 *
 * All BLE Mesh operating parameters (keys, models, command-to-opcode mapping)
 * are populated from JSON sent by the server (CFBN:JSON:<stack_id>:<json>).
 * Nothing in this file is hardcoded to a specific device type or LED protocol.
 */

#ifndef BLE_NATIVE_CONFIG_H
#define BLE_NATIVE_CONFIG_H

#include "esp_err.h"
#include "esp_ble_mesh_defs.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --------------------------------------------------------------------------
 * Sizing constants
 * -------------------------------------------------------------------------- */
#define BLE_NATIVE_MAX_STACKS        2   /**< Maximum parallel BLE native stacks */
#define BLE_NATIVE_MAX_COMMANDS      16  /**< Command table entries per stack */
#define BLE_NATIVE_CMD_NAME_LEN      32  /**< Max command name length */
#define BLE_NATIVE_SCHEMA_LEN        64  /**< Max param_schema string length */
#define BLE_NATIVE_PROV_NAME_LEN     32  /**< Max provisioner name length */

/* --------------------------------------------------------------------------
 * Command table entry  —  filled from JSON "commands" array
 *
 *  JSON example:
 *   { "name":"ONOFF",
 *     "model_id":"0x1000", "opcode":"0x8202",
 *     "ack_model_id":"0x1000", "ack_opcode":"0x8204",
 *     "param_schema":"value:uint8" }
 * -------------------------------------------------------------------------- */
typedef struct {
    char     name[BLE_NATIVE_CMD_NAME_LEN];  /**< Command name (from server) */
    uint16_t model_id;                        /**< BT SIG model ID */
    uint32_t opcode;                          /**< Mesh opcode to send */
    uint16_t ack_model_id;                    /**< Model opcode for ACK (0 = no-ack) */
    uint32_t ack_opcode;                      /**< Opcode of expected status reply */
    char     param_schema[BLE_NATIVE_SCHEMA_LEN]; /**< "key:type[,key:type…]" */
    bool     valid;
} ble_native_cmd_entry_t;

/* --------------------------------------------------------------------------
 * Mesh configuration  —  filled from JSON "mesh" sub-object
 * -------------------------------------------------------------------------- */
typedef struct {
    char     provisioner_name[BLE_NATIVE_PROV_NAME_LEN];
    uint8_t  net_key[16];          /**< 128-bit Network Key */
    uint8_t  app_key[16];          /**< 128-bit Application Key */
    uint8_t  ttl;                  /**< Default mesh TTL (typically 7) */
    uint16_t primary_unicast_addr; /**< Provisioner unicast address (1-based) */
    uint16_t next_unicast_addr;    /**< Next address to assign when provisioning */
    bool     valid;
} ble_native_mesh_cfg_t;

/* --------------------------------------------------------------------------
 * Per-stack runtime configuration
 * -------------------------------------------------------------------------- */
typedef struct {
    uint8_t                stack_id;
    ble_native_mesh_cfg_t  mesh;
    ble_native_cmd_entry_t commands[BLE_NATIVE_MAX_COMMANDS];
    uint8_t                num_commands;
    bool                   loaded;   /**< true after first JSON load */
} ble_native_stack_config_t;

/* --------------------------------------------------------------------------
 * API
 * -------------------------------------------------------------------------- */

/**
 * @brief Load JSON configuration for one stack.
 *
 * Expected top-level JSON shape:
 * @code
 * {
 *   "stack_id": "0",
 *   "stack_type": "esp32_native_ble",
 *   "ble_native": {
 *     "mesh": {
 *       "provisioner_name": "DA2_GW",
 *       "net_key": "A1B2C3D4E5F6A7B8C9DAEBFCAD1E2F30",
 *       "app_key": "0102030405060708090A0B0C0D0E0F10",
 *       "ttl": 7,
 *       "primary_unicast_addr": 1
 *     },
 *     "commands": [
 *       { "name":"ONOFF",
 *         "model_id":"0x1000","opcode":"0x8202",
 *         "ack_model_id":"0x1000","ack_opcode":"0x8204",
 *         "param_schema":"value:uint8" }
 *     ]
 *   }
 * }
 * @endcode
 *
 * @param stack_id   Target stack index (0 or 1)
 * @param json_str   Null-terminated JSON string
 * @param json_len   Length of json_str (bytes)
 * @return ESP_OK on success
 */
esp_err_t ble_native_config_load(uint8_t stack_id,
                                  const char *json_str,
                                  uint16_t json_len);

/**
 * @brief Look up a command entry by name.
 *
 * @param stack_id   Stack index
 * @param name       Command name string (e.g. "ONOFF")
 * @param[out] out   Filled with matching entry on success
 * @return ESP_OK if found, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t ble_native_config_find_cmd(uint8_t stack_id,
                                      const char *name,
                                      ble_native_cmd_entry_t *out);

/**
 * @brief Get mesh configuration for a stack.
 *
 * @param stack_id   Stack index
 * @param[out] out   Filled with mesh config on success
 * @return ESP_OK if config is loaded, ESP_ERR_INVALID_STATE otherwise
 */
esp_err_t ble_native_config_get_mesh(uint8_t stack_id,
                                      ble_native_mesh_cfg_t *out);

/**
 * @brief Check if a stack has been configured.
 */
bool ble_native_config_is_loaded(uint8_t stack_id);

/**
 * @brief Claim next unicast address for a newly provisioned node and advance
 *        the internal counter.
 *
 * @param stack_id       Stack index
 * @param[out] addr_out  Assigned unicast address
 * @return ESP_OK on success
 */
esp_err_t ble_native_config_alloc_unicast(uint8_t stack_id, uint16_t *addr_out);

/**
 * @brief Get the total number of valid commands loaded for a stack.
 *
 * @param stack_id  Stack index
 * @return Number of valid commands (0 if not loaded)
 */
uint8_t ble_native_config_get_num_cmds(uint8_t stack_id);

/**
 * @brief Retrieve a command entry by index.
 *
 * @param stack_id  Stack index
 * @param index     Command index (0..num_commands-1)
 * @param[out] out  Filled on success
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG if out of range
 */
esp_err_t ble_native_config_get_cmd_by_index(uint8_t stack_id,
                                               uint8_t index,
                                               ble_native_cmd_entry_t *out);

#ifdef __cplusplus
}
#endif

#endif /* BLE_NATIVE_CONFIG_H */
