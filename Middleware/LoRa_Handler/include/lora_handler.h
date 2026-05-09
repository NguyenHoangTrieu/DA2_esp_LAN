/**
 * @file lora_handler.h
 * @brief LoRa Handler Middleware – Transportation Layer Gateway
 *
 * Mirror of ble_handler.h for the LoRaWAN module layer.
 * Provides the same command-matching, execution and background-listen API
 * with LoRaWAN-specific function identifiers (25 functions, gateway build).
 */

#ifndef LORA_HANDLER_H
#define LORA_HANDLER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Type Definitions ===== */

/**
 * @brief LoRa Module Function Identifiers (25 functions, gateway build)
 */
typedef enum {
    /* -- Lifecycle (0-3) ----------------------------------------------------- */
    LORA_FUNC_HW_RESET = 0,
    LORA_FUNC_SW_RESET,
    LORA_FUNC_GET_INFO,
    LORA_FUNC_FACTORY_RESET,
    /* -- Region / Class (4-5) ------------------------------------------------ */
    LORA_FUNC_SET_REGION,
    LORA_FUNC_SET_CLASS,
    /* -- OTAA Provisioning (6-11) -------------------------------------------- */
    LORA_FUNC_SET_JOIN_MODE,
    LORA_FUNC_SET_DEVEUI,
    LORA_FUNC_GET_DEVEUI,
    LORA_FUNC_SET_APPEUI,
    LORA_FUNC_SET_APPKEY,
    LORA_FUNC_JOIN,
    /* -- Join Status / ABP (12-15) ------------------------------------------- */
    LORA_FUNC_GET_JOIN_STATUS,
    LORA_FUNC_SET_DEVADDR,
    LORA_FUNC_SET_NWKSKEY,
    LORA_FUNC_SET_APPSKEY,
    /* -- MAC / RF (16-21) ---------------------------------------------------- */
    LORA_FUNC_SET_DR,
    LORA_FUNC_SET_ADR,
    LORA_FUNC_SET_TXP,
    LORA_FUNC_SET_CHANNEL,
    LORA_FUNC_SET_CONFIRM,
    LORA_FUNC_SET_PUBLIC_NET,
    /* -- Data plane (22-24) -------------------------------------------------- */
    LORA_FUNC_SEND_UNCONFIRMED,
    LORA_FUNC_SEND_CONFIRMED,
    LORA_FUNC_READ_RECV,
    /* -- Port (25) ----------------------------------------------------------- */
    LORA_FUNC_SET_PORT,
    /* -- ABP extended (26) --------------------------------------------------- */
    LORA_FUNC_GET_DEVADDR,
    /* -- MAC extended (27-30) ------------------------------------------------ */
    LORA_FUNC_SET_RETRY,
    LORA_FUNC_SET_REPT,
    LORA_FUNC_SET_RXWIN2,
    LORA_FUNC_SET_DELAY,
    /* -- Data plane extended (31-32) ----------------------------------------- */
    LORA_FUNC_SEND_HEX,
    LORA_FUNC_SEND_CONFIRMED_HEX,
    /* -- Utility (33-34) ----------------------------------------------------- */
    LORA_FUNC_CHECK_PAYLOAD_LEN,
    LORA_FUNC_GET_VDD,
    /* -- Power management (35-38) -------------------------------------------- */
    LORA_FUNC_LOWPOWER,
    LORA_FUNC_LOWPOWER_AUTO_ON,
    LORA_FUNC_LOWPOWER_AUTO_OFF,
    LORA_FUNC_WAKEUP_NOTIFY,

    /* -- LoRa P2P / TEST mode (39-42) ---------------------------------------- */
    LORA_FUNC_ENTER_P2P_MODE,
    LORA_FUNC_SET_P2P_CONFIG,
    LORA_FUNC_SEND_P2P_PKT,
    LORA_FUNC_ENTER_P2P_RX,

    LORA_FUNC_COUNT    = 44,     ///< Capacity (43 defined + 1 reserved)
    LORA_FUNC_INVALID  = 0xFF
} lora_function_id_t;

/**
 * @brief Function configuration loaded from JSON
 */
typedef struct {
    bool available;                     ///< Is function present in JSON config
    bool is_hex;                        ///< true = binary/hex (send raw bytes), false = ASCII AT + CRLF
    bool is_prefix;                     ///< true = command is prefix, runtime data appended after it
    char command[128];                  ///< AT command or hex bytes (space-separated)
    uint8_t gpio_start[8];              ///< GPIO pins to set before command
    uint8_t gpio_start_state[8];        ///< GPIO states (0=LOW, 1=HIGH)
    uint8_t gpio_start_count;           ///< Number of pre-command GPIO actions
    uint32_t delay_start_ms;            ///< Delay before sending command
    char expect_response[64];           ///< Expected response terminator
    uint32_t timeout_ms;                ///< Command timeout in ms
    uint8_t gpio_end[8];                ///< GPIO pins to set after command
    uint8_t gpio_end_state[8];          ///< GPIO end states
    uint8_t gpio_end_count;             ///< Number of post-command GPIO actions
    uint32_t delay_end_ms;              ///< Delay after command
} lora_function_config_t;

/**
 * @brief LoRa module configuration (loaded from JSON)
 */
typedef struct {
    uint8_t  module_id;                     ///< Stack ID (0 or 1)
    char     module_type[32];               ///< "LORA"
    char     module_name[32];               ///< e.g. "RAK3172"
    char     comm_port_type[16];            ///< "uart" / "spi" / "i2c" / "usb"
    uint32_t baudrate;                      ///< UART baud rate
    bool     crlf_terminated;              ///< true = append \r\n to ASCII commands
    lora_function_config_t functions[LORA_FUNC_COUNT]; ///< All 40 slots (39 defined + 1 reserved)
} lora_module_config_t;

/**
 * @brief Function execution result
 */
typedef struct {
    esp_err_t status;                   ///< Execution status code
    char      response[2048];           ///< Module response (large for JOIN / streaming)
    uint16_t  response_len;             ///< Response byte count
    uint32_t  execution_time_ms;        ///< Total wall-clock execution time
} lora_exec_result_t;

/* ===== Public API ===== */

/**
 * @brief Initialise LoRa handler middleware.
 *
 * Must be called once before any other lora_handler_* function.
 * Creates the handler mutex and per-stack bus mutexes.
 */
esp_err_t lora_handler_init(void);

/**
 * @brief Load JSON configuration for a LoRa stack.
 *
 * @param stack_id   Stack ID (0 or 1)
 * @param json_config NULL-terminated JSON string
 * @param json_len   Length of JSON string
 * @return ESP_OK on success, ESP_FAIL if JSON invalid or module_type ≠ "LORA"
 */
esp_err_t lora_handler_load_config(uint8_t stack_id,
                                    const char *json_config,
                                    uint16_t json_len);

/* --- Core lifecycle functions --- */
esp_err_t lora_handler_hw_reset(uint8_t stack_id);
esp_err_t lora_handler_sw_reset(uint8_t stack_id);
esp_err_t lora_handler_factory_reset(uint8_t stack_id);
esp_err_t lora_handler_get_info(uint8_t stack_id, char *buffer, size_t max_len);

/* --- LoRaWAN join / status --- */
esp_err_t lora_handler_join(uint8_t stack_id);
esp_err_t lora_handler_get_join_status(uint8_t stack_id, char *buffer, size_t max_len);

/* --- Command matching & execution (used by task layer) --- */

/**
 * @brief Match a command string against the loaded JSON config.
 *
 * Two-pass search:
 *   Pass 1 – prefix match on func_cfg->command (is_prefix=true) or exact match.
 *   Pass 2 – function_name exact match (for GPIO-only triggers).
 *
 * @param stack_id    Stack ID (0 or 1)
 * @param command     Command string sent by server
 * @param func_config [out] Matched function config (GPIO, delays, timeout)
 * @return ESP_OK if matched, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t lora_handler_get_function_by_command(uint8_t stack_id,
                                                const char *command,
                                                lora_function_config_t *func_config);

/**
 * @brief Look up function config by function name (e.g. "MODULE_SW_RESET")
 *
 * @param stack_id    Stack ID (0 or 1)
 * @param func_name   Function name (e.g. "MODULE_SW_RESET", "MODULE_SET_REGION")
 * @param func_config [out] Matched function config
 * @return ESP_OK if found, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t lora_handler_get_function_by_name(uint8_t stack_id,
                                             const char *func_name,
                                             lora_function_config_t *func_config);

/**
 * @brief Execute a command using pre-matched function config.
 *
 * Called by the task layer after lora_handler_get_function_by_command().
 * Acquires the per-stack bus mutex before write+read so the background
 * listener cannot consume command-response bytes.
 *
 * @param stack_id    Stack ID (0 or 1)
 * @param command     Full command string (with or without trailing CRLF)
 * @param func_config Function config from JSON (GPIO, delays, timeout)
 * @param result      [out] Execution result
 * @return ESP_OK on success
 */
esp_err_t lora_handler_execute_command_with_config(uint8_t stack_id,
                                                    const char *command,
                                                    const lora_function_config_t *func_config,
                                                    lora_exec_result_t *result);

/**
 * @brief Send a raw binary command to the LoRa module.
 *
 * Acquires the bus mutex for the duration of the write+optional read.
 */
esp_err_t lora_handler_send_binary_command(uint8_t stack_id,
                                            const uint8_t *cmd_bytes,
                                            uint16_t cmd_len,
                                            uint8_t *response,
                                            uint16_t resp_len,
                                            uint16_t timeout_ms);

/**
 * @brief Non-blocking background listen for unsolicited LoRa module events.
 *
 * Tries to acquire the per-stack bus mutex with a 50 ms timeout.
 * Returns ESP_ERR_TIMEOUT immediately if the command task owns the bus,
 * allowing the listener task to yield without stalling commands.
 * Short 100 ms read window (LoRa bus may be slower than BLE).
 *
 * @param stack_id Stack ID (0 or 1)
 * @param buf      Caller-allocated receive buffer
 * @param max      Buffer capacity (bytes, including null terminator)
 * @param out_len  [out] Bytes written (not counting '\0')
 * @return ESP_OK with data, ESP_ERR_TIMEOUT if bus busy or no data
 */
esp_err_t lora_handler_listen(uint8_t stack_id, char *buf, size_t max, size_t *out_len);

#ifdef __cplusplus
}
#endif

#endif // LORA_HANDLER_H
