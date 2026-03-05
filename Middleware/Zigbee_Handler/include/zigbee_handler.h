/**
 * @file zigbee_handler.h
 * @brief Zigbee Handler Middleware – Transportation Layer
 *
 * Provides binary HEX-frame command building, execution, and async-event
 * listening for the E180-ZG120B (and compatible) Zigbee coordinator module.
 *
 * Frame format  : [0x55][LEN][CMD_TYPE][CMD_CODE][DATA 0-252 bytes][XOR]
 * LEN           : 2 + data_len
 * XOR           : CMD_TYPE ^ CMD_CODE ^ data[0] ^ ... ^ data[n-1]
 * cmd_type == -1: AT-mode command – ASCII string in function_config.command
 *
 * Module modes  : AT → HEX (per stack, tracked internally)
 * Async events  : CMD_TYPE 0x80 (node events) and 0x82 (ZCL events)
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
#define ZIGBEE_FRAME_MAX_LEN    264     ///< 4 header + 252 data + 1 XOR + 7 guard
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

    ZIGBEE_FUNC_COUNT   = 28,           ///< Capacity (26 defined + 2 reserved)
    ZIGBEE_FUNC_INVALID = 0xFF
} zigbee_function_id_t;

/**
 * @brief Function configuration at the handler layer.
 *        Copied from the JSON parser config on load.
 */
typedef struct {
    bool     available;
    int8_t   cmd_type;                      ///< -1 = AT mode
    int8_t   cmd_code;
    bool     is_prefix;
    bool     is_async_event;
    char     command[ZIGBEE_COMMAND_LEN];   ///< AT command string (cmd_type==-1)
    char     response_format[8];            ///< "ascii" | "hex"
    uint8_t  expect_response_bytes[ZIGBEE_RESPONSE_BYTES];
    uint8_t  expect_response_len;
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
 * @brief Execute a Zigbee command with pre-matched function config.
 *
 * For cmd_type == -1 (AT mode): sends ASCII command string.
 * For cmd_type >= 0           : builds binary HEX frame from cmd_type/cmd_code
 *                               + caller-supplied data bytes.
 *
 * @param stack_id   Stack ID (0 or 1)
 * @param func_id    Function enum value
 * @param data       Optional payload bytes appended to frame (may be NULL)
 * @param data_len   Length of data (0 if none)
 * @param result     [out] Execution result
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

#ifdef __cplusplus
}
#endif

#endif /* ZIGBEE_HANDLER_H */
