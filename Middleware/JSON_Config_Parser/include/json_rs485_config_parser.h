/**
 * @file json_rs485_config_parser.h
 * @brief RS485-specific JSON configuration parser
 *
 * Parses RS485 GPIO mode config JSON:
 *   {
 *     "module_id": "RS485",
 *     "module_type": "RS485",
 *     "stack_id": 0,
 *     "functions": [
 *       { "function_name": "RS485_SEND_MODE",
 *         "gpio_start_control": [{"pin":"03","state":"HIGH"},{"pin":"02","state":"HIGH"}],
 *         "delay_start": 1, "gpio_end_control": [], "delay_end": 0 },
 *       { "function_name": "RS485_RECEIVE_MODE",
 *         "gpio_start_control": [{"pin":"03","state":"LOW"},{"pin":"02","state":"LOW"}],
 *         "delay_start": 1, "gpio_end_control": [], "delay_end": 0 }
 *     ]
 *   }
 *
 * Pin format: "XY" where X = stack port (0/1), Y = pin 1-9.
 *   Stack 0 default: pin "03" = DE (STACK_GPIO_PIN_2),
 *                    pin "02" = RE (STACK_GPIO_PIN_1)
 */

#ifndef JSON_RS485_CONFIG_PARSER_H
#define JSON_RS485_CONFIG_PARSER_H

#include "json_config_parser.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================================
 * Constants
 * ========================================================================== */

#define RS485_MAX_GPIO_ACTIONS  5

/* ============================================================================
 * Enums
 * ========================================================================== */

typedef enum {
    JSON_RS485_FUNC_SEND_MODE    = 0,
    JSON_RS485_FUNC_RECEIVE_MODE = 1,
    JSON_RS485_FUNC_MAX
} json_rs485_function_id_t;

/* ============================================================================
 * Structures
 * ========================================================================== */

/**
 * @brief RS485 function configuration (GPIO-only, no AT command strings)
 */
typedef struct {
    bool available;
    json_rs485_function_id_t function_id;
    gpio_control_t gpio_start[RS485_MAX_GPIO_ACTIONS];
    uint8_t        gpio_start_count;
    uint16_t       delay_start_ms;
    gpio_control_t gpio_end[RS485_MAX_GPIO_ACTIONS];
    uint8_t        gpio_end_count;
    uint16_t       delay_end_ms;
} json_rs485_function_config_t;

/**
 * @brief Complete RS485 module configuration
 */
typedef struct {
    char     module_id[MAX_MODULE_ID_LEN];
    uint8_t  stack_id;
    json_rs485_function_config_t functions[JSON_RS485_FUNC_MAX];
} json_rs485_module_config_t;

/* ============================================================================
 * API
 * ========================================================================== */

/**
 * @brief Parse RS485 JSON configuration string
 *
 * @param json_str  JSON string buffer
 * @param json_len  Length of JSON string
 * @param out_config Output config structure
 * @return ESP_OK on success
 */
esp_err_t json_rs485_config_parse(const char *json_str, uint16_t json_len,
                                  json_rs485_module_config_t *out_config);

#ifdef __cplusplus
}
#endif

#endif /* JSON_RS485_CONFIG_PARSER_H */
