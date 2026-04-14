/**
 * @file zigbee_handler.h
 * @brief Zigbee Handler Middleware – Transportation Layer
 *
 * Provides AT command execution for the E180-ZG120B (and compatible) Zigbee coordinator modules.
 *
 * Unified format: all commands are ASCII AT strings (same as BLE/LoRa handlers).
 *  is_hex == false : ASCII/AT command – send command string, match ASCII response
 *  is_hex == true  : binary/hex command – reserved for future use
 */

#ifndef ZIGBEE_HANDLER_H
#define ZIGBEE_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"
#include "json_zigbee_config_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Constants ===== */

#define ZIGBEE_MAX_STACKS       2
#define ZIGBEE_COMMAND_LEN      64      ///< AT command string length
#define ZIGBEE_RESPONSE_LEN     64      ///< ASCII expect_response prefix length
#define ZIGBEE_RESP_BUF_SIZE    512     ///< Response buffer per command
#define ZIGBEE_ASYNC_BUF_SIZE   256     ///< Async event buffer

/* ===== Type Definitions ===== */

/**
 * @brief Zigbee function identifiers (26 functions, 28 slots).
 *        Mirrors json_zigbee_function_id_t for use at the handler layer.
 */
typedef enum {
    ZIGBEE_FUNC_HW_RESET              = 0,
    ZIGBEE_FUNC_SW_RESET              = 1,
    ZIGBEE_FUNC_FACTORY_RESET         = 2,
    ZIGBEE_FUNC_GET_INFO              = 3,
    ZIGBEE_FUNC_ENTER_HEX_MODE        = 4,
    ZIGBEE_FUNC_START_NETWORK         = 5,
    ZIGBEE_FUNC_STOP_NETWORK          = 6,
    ZIGBEE_FUNC_GET_NET_STATUS        = 7,
    ZIGBEE_FUNC_SET_CHANNEL           = 8,
    ZIGBEE_FUNC_SET_PANID             = 9,
    ZIGBEE_FUNC_SET_TX_POWER          = 10,
    ZIGBEE_FUNC_SET_PERMIT_JOIN       = 11,
    ZIGBEE_FUNC_NODE_JOIN_NOTIFY      = 12,
    ZIGBEE_FUNC_NODE_LEAVE_NOTIFY     = 13,
    ZIGBEE_FUNC_NODE_ANNOUNCE_NOTIFY  = 14,
    ZIGBEE_FUNC_QUERY_SHORT_ADDR      = 15,
    ZIGBEE_FUNC_QUERY_NODE_PORT_INFO  = 16,
    ZIGBEE_FUNC_DELETE_NODE           = 17,
    ZIGBEE_FUNC_ZCL_READ_ATTR         = 18,
    ZIGBEE_FUNC_ZCL_WRITE_ATTR        = 19,
    ZIGBEE_FUNC_ZCL_SEND_CONTROL_CMD  = 20,
    ZIGBEE_FUNC_ZCL_RECV_CONTROL_CMD  = 21,
    ZIGBEE_FUNC_ZCL_RECV_ATTR_REPORT  = 22,
    ZIGBEE_FUNC_ZCL_SET_REPORT_RULE   = 23,
    ZIGBEE_FUNC_SEND_UNICAST          = 24,
    ZIGBEE_FUNC_SEND_BROADCAST        = 25,
    // P1 extras (26-33)
    ZIGBEE_FUNC_SET_COMM_CONFIG       = 26,
    ZIGBEE_FUNC_ENTER_BOOTLOADER      = 27,
    ZIGBEE_FUNC_LEAVE_NETWORK         = 28,
    ZIGBEE_FUNC_SET_DEVICE_TYPE       = 29,
    ZIGBEE_FUNC_QUERY_IEEE_ADDR       = 30,
    ZIGBEE_FUNC_ZCL_BIND              = 31,
    ZIGBEE_FUNC_ZCL_UNBIND            = 32,
    ZIGBEE_FUNC_SEND_MULTICAST        = 33,
    // P2 optional (34-44)
    ZIGBEE_FUNC_ENTER_AT_MODE         = 34,
    ZIGBEE_FUNC_AUTO_FIND_TARGET      = 35,
    ZIGBEE_FUNC_ZCL_DISCOVER_ATTR     = 36,
    ZIGBEE_FUNC_ZCL_IDENTIFY          = 37,
    ZIGBEE_FUNC_ZCL_GET_BIND_TABLE    = 38,
    ZIGBEE_FUNC_ENTER_TRANSPARENT_MODE = 39,
    ZIGBEE_FUNC_SET_DEST_ADDR         = 40,
    ZIGBEE_FUNC_SET_DEST_EP           = 41,
    ZIGBEE_FUNC_SET_LP_LEVEL          = 42,
    ZIGBEE_FUNC_ENTER_SLEEP           = 43,
    ZIGBEE_FUNC_WAKEUP                = 44,

    ZIGBEE_FUNC_COUNT   = 48,           ///< Capacity (45 defined + 3 reserved)
    ZIGBEE_FUNC_INVALID = 0xFF
} zigbee_function_id_t;

/**
 * @brief Function configuration at the handler layer.
 *        Unified format: same fields as BLE/LoRa function configs.
 */
typedef struct {
    bool     available;
    bool     is_hex;                            ///< false = ASCII/AT, true = binary
    bool     is_prefix;
    char     command[ZIGBEE_COMMAND_LEN];       ///< AT command string
    char     expect_response[ZIGBEE_RESPONSE_LEN]; ///< ASCII response prefix
    uint16_t timeout_ms;
    /* GPIO fields */
    gpio_control_t gpio_start[MAX_GPIO_ACTIONS];
    uint8_t        gpio_start_count;
    uint32_t       delay_start_ms;
    gpio_control_t gpio_end[MAX_GPIO_ACTIONS];
    uint8_t        gpio_end_count;
    uint32_t       delay_end_ms;
} zigbee_function_config_t;

/**
 * @brief Per-stack Zigbee module configuration (loaded from JSON).
 */
typedef struct {
    uint8_t  module_id;
    char     module_type[32];
    char     module_name[32];
    char     comm_port_type[16];
    uint32_t baudrate;
    zigbee_function_config_t functions[ZIGBEE_FUNC_COUNT];
} zigbee_module_config_t;

/**
 * @brief Result of a synchronous Zigbee command execution.
 */
typedef struct {
    esp_err_t status;
    uint8_t   response[ZIGBEE_RESP_BUF_SIZE];
    uint16_t  response_len;
    uint32_t  execution_time_ms;
} zigbee_exec_result_t;

/* ===== Public API ===== */

/**
 * @brief Initialise Zigbee handler middleware.
 *        Creates per-stack bus mutexes. Must be called once.
 */
esp_err_t zigbee_handler_init(void);

/**
 * @brief Load JSON configuration for a Zigbee stack.
 */
esp_err_t zigbee_handler_load_config(uint8_t stack_id,
                                      const char *json_config,
                                      uint16_t json_len);

/**
 * @brief Unified AT/Zigbee command handler — two modes via is_hex:
 *
 *  is_hex==false (ASCII/AT): send "command[\r\n]" + ASCII prefix match in response
 *  is_hex==true  (Binary):   build "55 LEN CMD_TYPE CMD_CODE [DATA] XOR" and
 *                            match binary response prefix from expect_response
 */ 
esp_err_t zigbee_handler_execute_command_with_config(
    uint8_t stack_id,
    zigbee_function_id_t func_id,
    const uint8_t *data,
    uint8_t data_len,
    zigbee_exec_result_t *result);

/**
 * @brief Non-blocking background listen for unsolicited Zigbee events.
 *
 * Tries to acquire the per-stack bus mutex with a 50 ms timeout.
 * Returns ESP_ERR_TIMEOUT immediately if command task owns the bus.
 *
 * @param stack_id Stack ID
 * @param buf      Caller-allocated receive buffer
 * @param max      Buffer capacity in bytes
 * @param out_len  [out] Bytes received
 */
esp_err_t zigbee_handler_listen(uint8_t stack_id,
                                 uint8_t *buf,
                                 size_t   max,
                                 size_t  *out_len);

/**
 * @brief Retrieve a copy of the function config for a given function ID.
 */
esp_err_t zigbee_handler_get_function_config(uint8_t stack_id,
                                              zigbee_function_id_t func_id,
                                              zigbee_function_config_t *out);

/**
 * @brief Look up function config by command string (AT prefix or exact match).
 *
 * Pass 1: prefix match for AT+ commands, exact match for others.
 * Pass 2: function-name fallback for GPIO-only triggers.
 *
 * @param stack_id    Stack ID
 * @param command     Raw command string from server (e.g. "AT+INFO?")
 * @param func_config [out] Filled on success
 * @return ESP_OK if found, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t zigbee_handler_get_function_by_command(uint8_t stack_id,
                                                   const char *command,
                                                   zigbee_function_config_t *func_config);

/**
 * @brief Look up function config by function name (e.g. "MODULE_GET_NET_STATUS")
 *
 * @param stack_id    Stack ID
 * @param func_name   Function name
 * @param func_config [out] Matched function config
 * @return ESP_OK if found, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t zigbee_handler_get_function_by_name(uint8_t stack_id,
                                               const char *func_name,
                                               zigbee_function_config_t *func_config);

/**
 * @brief Execute a raw command string using a pre-resolved function config.
 *
 * Sends @p command directly to the module UART (no func_id lookup).
 * GPIO, timeout, and expected-response metadata come from @p fc.
 *
 * @param stack_id   Stack ID
 * @param command    Null-terminated AT command string (e.g. "AT+INFO?")
 * @param command_len Length of command (0 = auto strlen)
 * @param fc         Pre-resolved function config (GPIO/timeout/expect)
 * @param result     [out] Response buffer (may be NULL)
 * @return ESP_OK on success
 */
esp_err_t zigbee_handler_execute_command_raw(uint8_t stack_id,
                                              const char *command,
                                              uint16_t command_len,
                                              const zigbee_function_config_t *fc,
                                              zigbee_exec_result_t *result);

/**
 * @brief Diagnostic test: Send AT+RESET and log raw response.
 *
 * Useful for debugging module response issues. Sends AT+RESET command
 * and logs all bytes received within 2000ms timeout in both hex and ASCII.
 * Can be called from test task to verify module communication.
 *
 * @param stack_id Stack ID
 * @return ESP_OK on success
 */
esp_err_t zigbee_handler_test_at_reset(uint8_t stack_id);

/**
 * @brief One-time hardcoded AT mode ensure for E180-ZG120B.
 *
 * Checks NVS flag "zb_init/at_ok_s{stack_id}".  On first call (or if
 * verification fails) it:
 *   1. HW-resets the module via TCA P01 (RESET#)
 *   2. Exits transparent mode with "+++"
 *   3. Sends HEX enter-AT command [55 03 00 16 16]
 *   4. Verifies with AT+INFO?  (checks for "TYPE=" / "NO NET" / "MAC=")
 *   5. Saves NVS flag on success
 *
 * Subsequent calls within the same session are no-ops (static flag).
 * Must be called after UART is initialised (i.e. after zigbee_handler_load_config).
 */
esp_err_t zigbee_handler_ensure_at_mode(uint8_t stack_id);

#ifdef __cplusplus
}
#endif

#endif /* ZIGBEE_HANDLER_H */
