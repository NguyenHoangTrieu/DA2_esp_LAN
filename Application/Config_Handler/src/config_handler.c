/**
 * @file config_handler.c
 * @brief Configuration handler implementation for LAN MCU
 */

#include "config_handler.h"
#include "DA2_esp_LAN.h"
#include "fota_lan_config.h"
#include "fota_lan_handler.h"
#include <string.h>

static const char *TAG = "config_handler";

// Queue handle
QueueHandle_t g_config_handler_queue = NULL;

static bool config_handler_running = false;
static TaskHandle_t config_handler_task_handle = NULL;

/**
 * @brief Parse command type from 2-character prefix
 */
config_type_t config_parse_type(const char *cmd, uint16_t len) {
  if (len < 4 && (cmd[0] != 'C' || cmd[1] != 'F')) {
    return CONFIG_TYPE_UNKNOWN;
  }

  // Check first 2 characters
  if (cmd[2] == 'F' && cmd[3] == 'W') {
    return CONFIG_UPDATE_FIRMWARE;
  }
  if (strncmp(cmd + 2, "START_MCU", 10) == 0) {
    return CONFIG_SYNC_DATA;
  }
  return CONFIG_TYPE_UNKNOWN;
}

/**
 * @brief Parse FOTA command
 *
 * Format examples:
 *   "FW" - Use default URL from config
 *   "FW:https://example.com/firmware.bin" - Use custom URL
 *   "FW:https://example.com/firmware.bin:FORCE" - Force update
 */
esp_err_t config_parse_fota(const char *data, uint16_t len,
                                fota_lan_command_t *cfg) {
  if (!data || !cfg || len < 2) {
    return ESP_FAIL;
  }

  // Initialize with defaults
  memset(cfg, 0, sizeof(fota_lan_command_t));
  strncpy(cfg->url, FOTA_CONFIG_LAN_FIRMWARE_UPGRADE_URL, sizeof(cfg->url) - 1);
  cfg->force_update = false;

  // Check if just "FW" command (use defaults)
  if (len == 4) {
    ESP_LOGI(TAG, "FOTA command: use default URL");
    return ESP_OK;
  }

  // Parse format: "FW:url" or "FW:url:FORCE"
  const char *ptr = data + 4; // Skip "CF" and "FW"

  if (*ptr != ':') {
    ESP_LOGE(TAG, "Invalid FOTA format: missing ':'");
    return ESP_FAIL;
  }

  ptr++; // Skip ':'
  const char *end = data + len;

  // Find next ':' or end
  const char *colon = strchr(ptr, ':');

  if (colon && colon < end) {
    // Has URL and possibly FORCE flag
    int url_len = colon - ptr;
    if (url_len > 0 && url_len < sizeof(cfg->url)) {
      memset(cfg->url, 0, sizeof(cfg->url));
      memcpy(cfg->url, ptr, url_len);

      // Check for FORCE flag
      ptr = colon + 1;
      if ((end - ptr) >= 5 && strncmp(ptr, "FORCE", 5) == 0) {
        cfg->force_update = true;
      }
    }
  } else {
    // Just URL, no FORCE flag
    int url_len = end - ptr;
    if (url_len > 0 && url_len < sizeof(cfg->url)) {
      memset(cfg->url, 0, sizeof(cfg->url));
      memcpy(cfg->url, ptr, url_len);
    }
  }

  ESP_LOGI(TAG, "Parsed FOTA command - URL: '%s', Force: %d", cfg->url,
           cfg->force_update);

  return ESP_OK;
}

/**
 * @brief Config handler task - processes commands from queue
 */
static void config_handler_task(void *arg) {
  config_command_t cmd;

  ESP_LOGI(TAG, "Config LAN handler task started");

  while (config_handler_running) {
    // Wait for command from MCU LAN handler
    if (xQueueReceive(g_config_handler_queue, &cmd, pdMS_TO_TICKS(100)) ==
        pdTRUE) {
      ESP_LOGI(TAG, "Received config command, type: %d, len: %d", cmd.type,
               cmd.data_len);

      // Route based on command type
      switch (cmd.type) {
      case CONFIG_UPDATE_FIRMWARE: {
        fota_lan_command_t fota_cfg;

        if (config_parse_fota(cmd.raw_data, cmd.data_len, &fota_cfg) ==
            ESP_OK) {
          ESP_LOGI(TAG, "Starting FOTA process...");
          // Start FOTA handler task
          lan_ppp_connect();
          fota_lan_handler_task_start();
        } else {
          ESP_LOGE(TAG, "Failed to parse FOTA command");
        }
        break;
      }

      default:
        ESP_LOGW(TAG, "Unknown config type: %d", cmd.type);
        break;
      }
    }
  }

  ESP_LOGI(TAG, "Config LAN handler task exiting");
  vTaskDelete(NULL);
}

/**
 * @brief Start config handler task
 */
void config_handler_task_start(void) {
  if (config_handler_running) {
    ESP_LOGW(TAG, "Config LAN handler already running");
    return;
  }

  // Create queue if not exists
  if (!g_config_handler_queue) {
    g_config_handler_queue =
        xQueueCreate(CONFIG_QUEUE_SIZE, sizeof(config_command_t));
    if (!g_config_handler_queue) {
      ESP_LOGE(TAG, "Failed to create config queue");
      return;
    }
  }

  config_handler_running = true;

  BaseType_t ret = xTaskCreate(config_handler_task, "config_handler",
                               4096, NULL, 5, &config_handler_task_handle);

  if (ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create config handler task");
    config_handler_running = false;
    return;
  }

  ESP_LOGI(TAG, "Config LAN handler task created");
}

/**
 * @brief Stop config handler task
 */
void config_handler_task_stop(void) {
  if (!config_handler_running) {
    return;
  }

  config_handler_running = false;

  // Wait for task to exit
  if (config_handler_task_handle) {
    vTaskDelay(pdMS_TO_TICKS(200)); // Give time to exit gracefully
    config_handler_task_handle = NULL;
  }

  ESP_LOGI(TAG, "Config LAN handler task stopped");
}
