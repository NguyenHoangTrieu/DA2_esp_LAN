/**
 * @file json_zigbee_config_parser.h
 * @brief Zigbee-specific JSON configuration parser
 *
 * Extends the common JSON parser for Zigbee E180-ZG120B binary-frame protocol.
 * Key differences from BLE/LoRa:
 *  - cmd_type / cmd_code fields instead of ASCII command string
 *  - response_format: "ascii" | "hex"
 *  - expect_response stores ASCII string or space-separated hex (e.g. "55 80 03")
 *  - cmd_type == -1 means AT-mode command (uses `command` string field)
 */

#ifndef JSON_ZIGBEE_CONFIG_PARSER_H
#define JSON_ZIGBEE_CONFIG_PARSER_H

#include "json_config_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Constants
 * ========================================================================== */

#define ZIGBEE_MAX_FUNCTIONS    28      ///< 26 defined + 2 reserved slots
#define ZIGBEE_COMMAND_LEN      64      ///< AT command (AT mode only)
#define ZIGBEE_RESPONSE_LEN     64      ///< ASCII expect_response string
#define ZIGBEE_RESPONSE_BYTES   16      ///< Max bytes in parsed hex response prefix

/* ============================================================================
 * Enums
 * ========================================================================== */

/**
 * @brief Zigbee function IDs (26 functions – coordinator gateway build)
 *
 * Index values must match the order of ZIGBEE_FUNCTION_NAMES[] in the .c file.
 */
typedef enum {
    // -- Group 1: Lifecycle (0-4) -----------------------------------------------
    JSON_ZIGBEE_FUNC_HW_RESET = 0,
    JSON_ZIGBEE_FUNC_SW_RESET,
    JSON_ZIGBEE_FUNC_FACTORY_RESET,
    JSON_ZIGBEE_FUNC_GET_INFO,
    JSON_ZIGBEE_FUNC_ENTER_HEX_MODE,
    // -- Group 2: Network Management (5-11) -------------------------------------
    JSON_ZIGBEE_FUNC_START_NETWORK,
    JSON_ZIGBEE_FUNC_STOP_NETWORK,
    JSON_ZIGBEE_FUNC_GET_NET_STATUS,
    JSON_ZIGBEE_FUNC_SET_CHANNEL,
    JSON_ZIGBEE_FUNC_SET_PANID,
    JSON_ZIGBEE_FUNC_SET_TX_POWER,
    JSON_ZIGBEE_FUNC_SET_PERMIT_JOIN,
    // -- Group 3: Node Discovery (12-17) ----------------------------------------
    JSON_ZIGBEE_FUNC_NODE_JOIN_NOTIFY,
    JSON_ZIGBEE_FUNC_NODE_LEAVE_NOTIFY,
    JSON_ZIGBEE_FUNC_NODE_ANNOUNCE_NOTIFY,
    JSON_ZIGBEE_FUNC_QUERY_SHORT_ADDR,
    JSON_ZIGBEE_FUNC_QUERY_NODE_PORT_INFO,
    JSON_ZIGBEE_FUNC_DELETE_NODE,
    // -- Group 4: ZCL Control (18-23) -------------------------------------------
    JSON_ZIGBEE_FUNC_ZCL_READ_ATTR,
    JSON_ZIGBEE_FUNC_ZCL_WRITE_ATTR,
    JSON_ZIGBEE_FUNC_ZCL_SEND_CONTROL_CMD,
    JSON_ZIGBEE_FUNC_ZCL_RECV_CONTROL_CMD,
    JSON_ZIGBEE_FUNC_ZCL_RECV_ATTR_REPORT,
    JSON_ZIGBEE_FUNC_ZCL_SET_REPORT_RULE,
    // -- Group 5: Data TX (24-25) -----------------------------------------------
    JSON_ZIGBEE_FUNC_SEND_UNICAST,
    JSON_ZIGBEE_FUNC_SEND_BROADCAST,
    // sentinel
    JSON_ZIGBEE_FUNC_MAX
} json_zigbee_function_id_t;

/* ============================================================================
 * Structures
 * ========================================================================== */

/**
 * @brief Zigbee function configuration (parsed from JSON)
 */
typedef struct {
    bool available;                                     ///< Function present in JSON
    json_zigbee_function_id_t function_id;              ///< Enum index
    // AT-mode fields (used when cmd_type == -1)
    char command[ZIGBEE_COMMAND_LEN];                   ///< AT command string
    bool is_prefix;                                     ///< True if data appended at runtime
    // HEX-mode fields
    int8_t  cmd_type;                                   ///< Frame CMD_TYPE byte (-1 = AT mode)
    int8_t  cmd_code;                                   ///< Frame CMD_CODE byte
    // Response
    char    response_format[8];                         ///< "ascii" or "hex"
    char    expect_response[ZIGBEE_RESPONSE_LEN];       ///< Response string or hex spec
    uint8_t expect_response_bytes[ZIGBEE_RESPONSE_BYTES]; ///< Parsed hex bytes (if hex format)
    uint8_t expect_response_len;                        ///< Length of parsed hex bytes
    // Timing / GPIO
    gpio_control_t gpio_start[MAX_GPIO_ACTIONS];
    uint8_t  gpio_start_count;
    uint16_t delay_start_ms;
    uint16_t timeout_ms;
    gpio_control_t gpio_end[MAX_GPIO_ACTIONS];
    uint8_t  gpio_end_count;
    uint16_t delay_end_ms;
    // Async-only flag (no command sent, only received from listener)
    bool is_async_event;
} json_zigbee_function_config_t;

/**
 * @brief Complete Zigbee module configuration (parsed from JSON)
 */
typedef struct {
    module_metadata_t metadata;
    json_zigbee_function_config_t functions[ZIGBEE_MAX_FUNCTIONS];
    uint8_t function_count;
} json_zigbee_module_config_t;

/* ============================================================================
 * Public API
 * ========================================================================== */

/**
 * @brief Parse complete Zigbee module configuration from JSON string.
 *
 * Validates module_type == "ZIGBEE", iterates "functions" array,
 * validates each function name against the hardcoded table, and
 * parses both standard (BLE/LoRa-compatible) fields and Zigbee-specific
 * fields (cmd_type, cmd_code, response_format).
 *
 * @param json_str  NULL-terminated JSON string
 * @param config    [out] Caller-allocated output structure
 * @return ESP_OK on success
 */
esp_err_t json_zigbee_config_parse(const char *json_str,
                                   json_zigbee_module_config_t *config);

#ifdef __cplusplus
}
#endif

#endif // JSON_ZIGBEE_CONFIG_PARSER_H
