/**
 * @file config_handler.h
 * @brief Configuration handler for LAN MCU (Master side)
 */

#ifndef CONFIG_HANDLER_H
#define CONFIG_HANDLER_H

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "stack_handler.h"
#include <stdbool.h>
#include <stdint.h>

// Command buffer size
#define CONFIG_CMD_MAX_LEN 256
#define CONFIG_QUEUE_SIZE 10

/**
 * @brief Command type codes for LAN MCU
 */
typedef enum {
  CONFIG_UPDATE_FIRMWARE = 0,  // "CFFW" - Firmware update command
  CONFIG_UPDATE_LORA = 1,      // "CFLR" - LoRa config command
  CONFIG_UPDATE_CAN = 2,       // "CFCB" or "CFCM" - CAN config command
  CONFIG_UPDATE_SCAN = 3,      // "CFSC" - Config query command1
  CONFIG_UPDATE_STACK = 4,    // "CFST" - Stack config command
  CONFIG_UPDATE_RS485 = 5, // "CFRS" - RS485 config command
  CONFIG_TYPE_UNKNOWN = 0xFF
} config_type_t;

/**
 * @brief Firmware update command structure
 */
typedef struct {
  char url[256];     // Firmware URL (optional, can use default)
  bool force_update; // Force update even if same version
} fota_lan_command_t;

/**
 * @brief Generic configuration command for LAN
 */
typedef struct {
  config_type_t type;
  char raw_data[CONFIG_CMD_MAX_LEN];
  uint16_t data_len;
} config_command_t;

// Main config handler queue (receives commands from LAN comm)
// Declared in config_handler.c, used by mcu_lan_handler
extern QueueHandle_t g_config_handler_queue;

/**
 * @brief Start config handler task
 */
void config_handler_task_start(void);

/**
 * @brief Stop config handler task
 */
void config_handler_task_stop(void);

/**
 * @brief Parse command type from raw data
 *
 * @param cmd Command string
 * @param len Command length
 * @return config_type_t Command type
 */
config_type_t config_parse_type(const char *cmd, uint16_t len);

/**
 * @brief Save CAN configuration to NVS
 */
esp_err_t config_save_can_config_to_nvs(void);

/**
 * @brief Save LoRa TDMA configuration (g_lora_handler_cfg and crypto key) to
 * NVS
 */
esp_err_t config_save_lora_handler_config_to_nvs(void);

/**
 * @brief Save LoRa E32 radio configuration (g_lora_e32_params and baud rate) to
 * NVS
 */
esp_err_t config_save_lora_e32_config_to_nvs(void);

/**
 * @brief Save stack type to NVS
 */
esp_err_t config_save_stack_type(uint8_t stack_id, stack_comm_type_t type);

/**
 * @brief Save RS485 baud rate to NVS
 */
esp_err_t config_save_rs485_baud(uint32_t baud_rate);

/**
 * @brief Erase all gateway configurations from NVS
 */
esp_err_t erase_all_configs_from_nvs(void);

/**
 * @brief Initialize configuration system (load or save defaults)
 */
esp_err_t config_init(void);

#endif // CONFIG_HANDLER_H
