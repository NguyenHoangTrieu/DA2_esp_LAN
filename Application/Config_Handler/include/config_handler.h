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
#include <stdbool.h>
#include <stdint.h>

// Command buffer size
#define CONFIG_CMD_MAX_LEN 256
#define CONFIG_QUEUE_SIZE 10

/**
 * @brief Command type codes for LAN MCU
 */
typedef enum {
  CONFIG_UPDATE_FIRMWARE = 0, // "FW" - Firmware update command
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
 * @brief Parse firmware update command
 *
 * Format: "FW" or "FW:URL" or "FW:URL:FORCE"
 *
 * @param data Raw command data
 * @param len Command length
 * @param cfg Output FOTA config structure
 * @return esp_err_t ESP_OK on success
 */
esp_err_t config_parse_fota(const char *data, uint16_t len,
                                fota_lan_command_t *cfg);

#endif // CONFIG_HANDLER_H
