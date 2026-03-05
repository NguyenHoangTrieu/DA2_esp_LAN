/**
 * @file json_lora_config_parser.h
 * @brief LoRa-specific JSON configuration parser
 */

#ifndef JSON_LORA_CONFIG_PARSER_H
#define JSON_LORA_CONFIG_PARSER_H

#include "json_config_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Constants
 * ========================================================================== */

#define LORA_MAX_FUNCTIONS  28      ///< 25 defined + 3 reserved slots
#define LORA_COMMAND_LEN    128
#define LORA_RESPONSE_LEN   64

/* ============================================================================
 * Enums
 * ========================================================================== */

/**
 * @brief LoRaWAN function IDs (25 functions – gateway build)
 *
 * Index values must match the order of LORA_FUNCTION_NAMES[] in the .c file.
 */
typedef enum {
    // ── Lifecycle (0–3) ──────────────────────────────────────────────────────
    JSON_LORA_FUNC_HW_RESET = 0,
    JSON_LORA_FUNC_SW_RESET,
    JSON_LORA_FUNC_GET_INFO,
    JSON_LORA_FUNC_FACTORY_RESET,
    // ── Region / Class (4–5) ─────────────────────────────────────────────────
    JSON_LORA_FUNC_SET_REGION,
    JSON_LORA_FUNC_SET_CLASS,
    // -- OTAA Provisioning (6-11) ------------------------------------------------
    JSON_LORA_FUNC_SET_JOIN_MODE,
    JSON_LORA_FUNC_SET_DEVEUI,
    JSON_LORA_FUNC_GET_DEVEUI,
    JSON_LORA_FUNC_SET_APPEUI,
    JSON_LORA_FUNC_SET_APPKEY,
    JSON_LORA_FUNC_JOIN,
    // -- Join Status / ABP (12-15) -----------------------------------------------
    JSON_LORA_FUNC_GET_JOIN_STATUS,
    JSON_LORA_FUNC_SET_DEVADDR,
    JSON_LORA_FUNC_SET_NWKSKEY,
    JSON_LORA_FUNC_SET_APPSKEY,
    // -- MAC / RF (16-21) --------------------------------------------------------
    JSON_LORA_FUNC_SET_DR,
    JSON_LORA_FUNC_SET_ADR,
    JSON_LORA_FUNC_SET_TXP,
    JSON_LORA_FUNC_SET_CHANNEL,
    JSON_LORA_FUNC_SET_CONFIRM,
    JSON_LORA_FUNC_SET_PUBLIC_NET,
    // -- Data plane (22-24) ------------------------------------------------------
    JSON_LORA_FUNC_SEND_UNCONFIRMED,
    JSON_LORA_FUNC_SEND_CONFIRMED,
    JSON_LORA_FUNC_READ_RECV,
    // sentinel
    JSON_LORA_FUNC_MAX
} json_lora_function_id_t;

/* ============================================================================
 * Structures
 * ========================================================================== */

/**
 * @brief LoRa function configuration (parsed from JSON)
 */
typedef struct {
    bool available;
    json_lora_function_id_t function_id;
    char command[LORA_COMMAND_LEN];
    bool is_prefix;
    gpio_control_t gpio_start[MAX_GPIO_ACTIONS];
    uint8_t gpio_start_count;
    uint16_t delay_start_ms;
    char expect_response[LORA_RESPONSE_LEN];
    uint16_t timeout_ms;
    gpio_control_t gpio_end[MAX_GPIO_ACTIONS];
    uint8_t gpio_end_count;
    uint16_t delay_end_ms;
} json_lora_function_config_t;

/**
 * @brief Complete LoRa module configuration (parsed from JSON)
 */
typedef struct {
    module_metadata_t metadata;
    json_lora_function_config_t functions[LORA_MAX_FUNCTIONS];
    uint8_t function_count;
} json_lora_module_config_t;

/* ============================================================================
 * Public API
 * ========================================================================== */

/**
 * @brief Parse complete LoRa module configuration from JSON string
 *
 * Parses metadata via the common parser, then iterates the "functions" array
 * validating each entry against the hardcoded LORA_FUNCTION_NAMES table.
 *
 * @param json_str  NULL-terminated JSON string to parse
 * @param config    Output LoRa configuration structure (must be caller-allocated)
 * @return ESP_OK on success, ESP_ERR_INVALID_ARG / ESP_ERR_NO_MEM on failure
 */
esp_err_t json_lora_config_parse(const char *json_str,
                                 json_lora_module_config_t *config);

#ifdef __cplusplus
}
#endif

#endif // JSON_LORA_CONFIG_PARSER_H
