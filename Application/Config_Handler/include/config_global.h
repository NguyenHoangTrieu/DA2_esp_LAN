/**
 * @file config_global.h
 * @brief Global configuration variables for LAN MCU
 * 
 * Stores configuration that will be sent to WAN MCU when requested.
 * For Module Base Setting architecture (BLE-only trial version).
 */

#ifndef CONFIG_GLOBAL_H
#define CONFIG_GLOBAL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Constants ===== */
#define MAX_JSON_CONFIG_SIZE 2048
#define MODULE_ID_NONE "none"

/* ===== Global Configuration Variables ===== */

/**
 * @brief Stack 1 module ID
 * "none" = TCA not accessible (no adapter or I2C failure)
 * "000"  = Zigbee E18
 * "009"  = RS485
 * "015"  = LoRa WIO E5
 */
extern char g_stack_1_id[5];

/**
 * @brief Stack 2 module ID
 * "none" = TCA not accessible (no adapter or I2C failure)
 */
extern char g_stack_2_id[5];

/**
 * @brief RS485 baudrate configuration
 * Only used when a stack is configured as RS485
 */
extern uint32_t g_rs485_baudrate;

/**
 * @brief JSON config for Stack 1 (if configured via BL:JSON command)
 */
extern char *g_stack_1_json_config;
extern uint16_t g_stack_1_json_len;

extern char *g_stack_2_json_config;
extern uint16_t g_stack_2_json_len;

/* ===== Getter Functions ===== */

/**
 * @brief Get Stack 1 module ID
 * @return Pointer to module ID string (e.g., "000", "002")
 */
const char* config_get_stack_1_id(void);

/**
 * @brief Get Stack 2 module ID
 * @return Pointer to module ID string
 */
const char* config_get_stack_2_id(void);

/**
 * @brief Get RS485 baudrate
 * @return Baudrate value
 */
uint32_t config_get_rs485_baudrate(void);

/**
 * @brief Get Stack 1 JSON config
 * @param len Output parameter for config length
 * @return Pointer to JSON config string (may be empty)
 */
const char* config_get_stack_1_json(uint16_t *len);

/**
 * @brief Get Stack 2 JSON config
 * @param len Output parameter for config length
 * @return Pointer to JSON config string (may be empty)
 */
const char* config_get_stack_2_json(uint16_t *len);

/* ===== Setter Functions ===== */

/**
 * @brief Set Stack 1 module ID
 * @param module_id Module ID string (e.g., "002")
 */
void config_set_stack_1_id(const char *module_id);

/**
 * @brief Set Stack 2 module ID
 * @param module_id Module ID string
 */
void config_set_stack_2_id(const char *module_id);

/**
 * @brief Set RS485 baudrate
 * @param baudrate Baudrate value
 */
void config_set_rs485_baudrate(uint32_t baudrate);

/**
 * @brief Set Stack 1 JSON config
 * @param json_str JSON config string
 * @param len JSON config length
 */
void config_set_stack_1_json(const char *json_str, uint16_t len);

/**
 * @brief Set Stack 2 JSON config
 * @param json_str JSON config string
 * @param len JSON config length
 */
void config_set_stack_2_json(const char *json_str, uint16_t len);

/**
 * @brief Initialize config globals with default values
 */
void config_global_init(void);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_GLOBAL_H
