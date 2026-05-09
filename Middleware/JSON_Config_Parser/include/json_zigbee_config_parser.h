/**
 * @file json_zigbee_config_parser.h
 * @brief Zigbee-specific JSON configuration parser
 *
 * Unified format identical to BLE/LoRa parsers:
 *  - command: "55 CMD_TYPE CMD_CODE" template (is_hex=true) or AT string (is_hex=false)
 *  - is_hex: true = binary/HEX mode (E180 native protocol), false = ASCII/AT mode
 *  - is_prefix: true = runtime data is appended after the command bytes
 *  - expect_response: space-separated hex bytes string (HEX mode) or ASCII prefix
 *  NOTE: cmd_type, cmd_code, resp_format JSON fields are obsolete and ignored.
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

#define ZIGBEE_MAX_FUNCTIONS    56      ///< 51 defined + 5 reserved slots
#define ZIGBEE_COMMAND_LEN      64      ///< Command string (AT text or hex template)
#define ZIGBEE_RESPONSE_LEN     64      ///< Expect_response (ASCII prefix or hex bytes string)

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
    // -- Group 6: Lifecycle extras – P1 (26-29) ---------------------------------
    JSON_ZIGBEE_FUNC_SET_COMM_CONFIG,       ///< L5 P1 – set baud rate
    JSON_ZIGBEE_FUNC_ENTER_BOOTLOADER,      ///< L7 P1 – OTA boot GPIO
    JSON_ZIGBEE_FUNC_LEAVE_NETWORK,         ///< N5 P1 – self-initiated leave
    JSON_ZIGBEE_FUNC_SET_DEVICE_TYPE,       ///< N6 P1 – Coord/Router/EndDevice
    // -- Group 7: Node extras – P1 (30-33) --------------------------------------
    JSON_ZIGBEE_FUNC_QUERY_IEEE_ADDR,       ///< D4 P1 – IEEE EUI-64 lookup
    JSON_ZIGBEE_FUNC_ZCL_BIND,             ///< B1 P1 – ZDO bind request
    JSON_ZIGBEE_FUNC_ZCL_UNBIND,           ///< B2 P1 – ZDO unbind
    JSON_ZIGBEE_FUNC_SEND_MULTICAST,       ///< TX3 P1 – group multicast
    // -- Group 8: Optional P2 (34-44) -------------------------------------------
    JSON_ZIGBEE_FUNC_ENTER_AT_MODE,        ///< L8 P2 – HEX→AT mode switch
    JSON_ZIGBEE_FUNC_AUTO_FIND_TARGET,     ///< D5 P2 – binding partner discover
    JSON_ZIGBEE_FUNC_ZCL_DISCOVER_ATTR,   ///< Z5 P2 – ZCL discover attributes
    JSON_ZIGBEE_FUNC_ZCL_IDENTIFY,        ///< Z6 P2 – ZCL identify
    JSON_ZIGBEE_FUNC_ZCL_GET_BIND_TABLE,  ///< B3 P2 – read bind table
    JSON_ZIGBEE_FUNC_ENTER_TRANSPARENT_MODE, ///< TX4 P2 – pass-through mode
    JSON_ZIGBEE_FUNC_SET_DEST_ADDR,       ///< A1 P2 – default target address
    JSON_ZIGBEE_FUNC_SET_DEST_EP,         ///< A2 P2 – default target endpoint
    JSON_ZIGBEE_FUNC_SET_LP_LEVEL,        ///< PM1 P2 – sleep level (end device)
    JSON_ZIGBEE_FUNC_ENTER_SLEEP,         ///< PM2 P2 – force sleep
    JSON_ZIGBEE_FUNC_WAKEUP,              ///< PM3 P2 – wake via GPIO
    // Group 9: Mode switching (45)
    JSON_ZIGBEE_FUNC_EXIT_SEND_MODE,      ///< M1 – exit transparent/send mode (+++)
    // Group 10: Boot/misc events (46-50)
    JSON_ZIGBEE_FUNC_BOOT_NOTIFY,         ///< async boot/restart notify
    JSON_ZIGBEE_FUNC_NET_STATUS_NOTIFY,   ///< async network status change
    JSON_ZIGBEE_FUNC_FIND_BIND_NOTIFY,    ///< async auto-bind result
    JSON_ZIGBEE_FUNC_SEND_CONFIRM,        ///< async ZCL send confirmation
    JSON_ZIGBEE_FUNC_ZCL_DEFAULT_RSP,     ///< async ZCL default response
    // sentinel
    JSON_ZIGBEE_FUNC_MAX
} json_zigbee_function_id_t;

/* ============================================================================
 * Structures
 * ========================================================================== */

/**
 * @brief Zigbee function configuration (parsed from JSON)
 *
 * Unified format: identical layout to json_ble_function_config_t and
 * json_lora_function_config_t.  No cmd_type / cmd_code / response_format.
 */
typedef struct {
    bool available;                                 ///< Function present in JSON
    json_zigbee_function_id_t function_id;          ///< Enum index
    char command[ZIGBEE_COMMAND_LEN];               ///< AT command string
    bool is_prefix;                                 ///< True if runtime data appended
    bool is_hex;                                    ///< true = binary/hex, false = ASCII/AT
    // Timing / GPIO
    gpio_control_t gpio_start[MAX_GPIO_ACTIONS];
    uint8_t  gpio_start_count;
    uint16_t delay_start_ms;
    char     expect_response[ZIGBEE_RESPONSE_LEN];  ///< ASCII response prefix string
    uint16_t timeout_ms;
    gpio_control_t gpio_end[MAX_GPIO_ACTIONS];
    uint8_t  gpio_end_count;
    uint16_t delay_end_ms;
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
 * and maps each function name against the hardcoded ZIGBEE_FUNCTION_NAMES[]
 * table.  Ignores unknown/obsolete JSON fields (cmd_type, cmd_code,
 * resp_format) silently.
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
