/**
 * @file config_handler.c
 * @brief Configuration handler implementation for LAN MCU
 */

#include "config_handler.h"
#include "DA2_esp_LAN.h"
#include "ble_handler.h"
#include "ble_gatt_handler.h"
#include "ble_native_handler.h"
#include "config_handler_ble_commands.h"
#include "config_handler_lora_commands.h"
#include "config_handler_zigbee_commands.h"
#include "config_handler_ble_native_commands.h"
#include "config_handler_ble_gatt_commands.h"
#include "config_handler_rs485_commands.h"
#include "config_ble_mode.h"
#include "config_global.h"
#include "fota_lan_config.h"
#include "fota_lan_handler.h"
#include "led_strip.h"
#include "mcu_wan_handler.h"
#include "rs485_handler.h"
#include "stack_handler.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "config_handler";

// Queue handle
QueueHandle_t g_config_handler_queue = NULL;

static bool config_handler_running = false;
static TaskHandle_t config_handler_task_handle = NULL;
static esp_err_t config_parse_fota(const char *data, uint16_t len,
                                   fota_lan_command_t *cfg);
static void mcu_wan_config_callback(const uint8_t *data, uint16_t len,
                                    bool is_fota);

config_type_t config_parse_type(const char *cmd, uint16_t len) {
  if (len < 4 || cmd[0] != 'C' || cmd[1] != 'F') {
    ESP_LOGW(TAG, "config_parse_type: INVALID PREFIX");
    ESP_LOGW(TAG, "  len=%u (need >=4), cmd[0-3]=%02X %02X %02X %02X ('%c%c%c%c')",
             len, (unsigned char)cmd[0], (unsigned char)cmd[1], (unsigned char)cmd[2], (unsigned char)cmd[3],
             (cmd[0] >= 32 && cmd[0] <= 126) ? cmd[0] : '.',
             (cmd[1] >= 32 && cmd[1] <= 126) ? cmd[1] : '.',
             (cmd[2] >= 32 && cmd[2] <= 126) ? cmd[2] : '.',
             (cmd[3] >= 32 && cmd[3] <= 126) ? cmd[3] : '.');
    return CONFIG_TYPE_UNKNOWN;
  }

  // Check first 2 characters after "CF"
  if (cmd[2] == 'F' && cmd[3] == 'W') {
    return CONFIG_UPDATE_FIRMWARE;
  } else if (cmd[2] == 'F' && cmd[3] == 'U') {
    return CONFIG_SET_FIRMWARE_URL;
  } else if (cmd[2] == 'R' && cmd[3] == 'S') {
    // RS485 commands - check subcommand
    if (len >= 10 && strncmp(cmd + 5, "JSON:", 5) == 0) {
      return CONFIG_UPDATE_RS485_JSON;
    } else {
      return CONFIG_UPDATE_RS485; // CFRS:BR:<baud>
    }
  } else if (cmd[2] == 'B' && cmd[3] == 'L') {
    // BLE AT commands (CFBL = CF + BLE) - check subcommand
    if (len >= 10 && strncmp(cmd + 5, "JSON:", 5) == 0) {
      return CONFIG_UPDATE_BLE_JSON;
    } else {
      // All other BLE AT commands use unified parser
      return CONFIG_UPDATE_BLE_CMD;
    }
  } else if (cmd[2] == 'L' && cmd[3] == 'R') {
    // LoRa commands - check subcommand
    if (len >= 10 && strncmp(cmd + 5, "JSON:", 5) == 0) {
      return CONFIG_UPDATE_LORA_JSON;
    } else {
      return CONFIG_UPDATE_LORA_CMD;
    }
  } else if (cmd[2] == 'Z' && cmd[3] == 'B') {
    // Zigbee commands - check subcommand
    if (len >= 10 && strncmp(cmd + 5, "JSON:", 5) == 0) {
      return CONFIG_UPDATE_ZIGBEE_JSON;
    } else {
      return CONFIG_UPDATE_ZIGBEE_CMD;
    }
  } else if (cmd[2] == 'B' && cmd[3] == 'N') {
    // BLE Native (ESP32 direct BLE Mesh) commands
    if (len >= 10 && strncmp(cmd + 5, "JSON:", 5) == 0) {
      return CONFIG_UPDATE_BLE_NATIVE_JSON;
    } else {
      return CONFIG_UPDATE_BLE_NATIVE_CMD;
    }
  } else if (cmd[2] == 'B' && cmd[3] == 'G') {
    // BLE GATT Central (ESP32 native GATT Central) commands
    if (len >= 10 && strncmp(cmd + 5, "JSON:", 5) == 0) {
      return CONFIG_UPDATE_BLE_GATT_JSON;
    } else {
      return CONFIG_UPDATE_BLE_GATT_CMD;
    }
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
static esp_err_t config_parse_fota(const char *data, uint16_t len,
                                   fota_lan_command_t *cfg) {
  if (!data || !cfg || len < 2) {
    return ESP_FAIL;
  }

  // Initialize with defaults
  memset(cfg, 0, sizeof(fota_lan_command_t));
  strncpy(cfg->url, fota_lan_handler_get_url(), sizeof(cfg->url) - 1);
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
 * @brief Parse RS485 baud rate configuration
 *
 * Format: CFRS:BR:9600 or CFRS:BR:115200
 *
 * Valid baud rates: 9600, 19200, 38400, 57600, 115200
 *
 * @param data Command data buffer
 * @param len Length of data
 * @return esp_err_t ESP_OK on success
 */
static esp_err_t config_parse_rs485_baud(const uint8_t *data, uint16_t len) {
  if (data == NULL || len < 8) { // Minimum: "CFRS:BR:"
    ESP_LOGE(TAG, "RS485 baud: invalid buffer");
    return ESP_ERR_INVALID_ARG;
  }

  // Check CFRS:BR: prefix
  if (memcmp(data, "CFRS:BR:", 8) != 0) {
    ESP_LOGE(TAG, "RS485 baud: missing CFRS:BR: prefix");
    return ESP_FAIL;
  }

  // Parse baud rate value
  const char *ptr = (const char *)(data + 8);
  int value_len = len - 8;

  if (value_len > 0 && value_len < 16) {
    char baud_str[16] = {0};
    memcpy(baud_str, ptr, value_len);
    uint32_t baud_rate = atoi(baud_str);

    // Validate RS485 baud rates (max 115200)
    if (baud_rate != 9600 && baud_rate != 19200 && baud_rate != 38400 &&
        baud_rate != 57600 && baud_rate != 115200) {
      ESP_LOGE(TAG,
               "RS485: Invalid baud rate %lu (valid: 9600, 19200, 38400, "
               "57600, 115200)",
               (unsigned long)baud_rate);
      return ESP_FAIL;
    }

    g_rs485_baud_rate = baud_rate;
    ESP_LOGI(TAG, "RS485 baud rate updated: %lu", (unsigned long)baud_rate);

    // Save to NVS
    esp_err_t err = config_save_rs485_baud(baud_rate);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save RS485 baud to NVS: %s",
               esp_err_to_name(err));
      return err;
    }

    return ESP_OK;
  }

  ESP_LOGE(TAG, "RS485 baud: invalid value format");
  return ESP_FAIL;
}

/**
 * @brief Config callback from MCU WAN handler
 * @param data    Pointer to config data buffer (starts with "CF...")
 * @param len     Length of config data
 * @param is_fota True if this config packet is identified as a FOTA command
 */
static void mcu_wan_config_callback(const uint8_t *data, uint16_t len,
                                    bool is_fota) {
  if (data == NULL || len == 0) {
    ESP_LOGW(TAG, "Config callback: invalid data (NULL or len=0)");
    return;
  }

  if (g_config_handler_queue == NULL) {
    ESP_LOGW(TAG, "Config callback: config queue not initialized");
    return;
  }

  // CRITICAL: Reject configs that exceed maximum size
  if (len > CONFIG_CMD_MAX_LEN) {
    ESP_LOGE(TAG, "Config too large: %u > %d bytes - REJECTED", len, CONFIG_CMD_MAX_LEN);
    return;
  }

  // Allocate on heap to avoid stack overflow (4KB+ structure)
  config_command_t *cmd = (config_command_t *)malloc(sizeof(config_command_t));
  if (cmd == NULL) {
    ESP_LOGE(TAG, "Config callback: failed to allocate command buffer (%u bytes)", sizeof(config_command_t));
    return;
  }

  memset(cmd, 0, sizeof(config_command_t));

  // Parse command type from raw data
  cmd->type = config_parse_type((const char *)data, len);
  cmd->source = CONFIG_SOURCE_WAN_MCU;  // All commands via WAN callback are from WAN MCU
  cmd->data_len = len;

  // Copy raw config payload
  memcpy(cmd->raw_data, data, cmd->data_len);

  // Enqueue command pointer to the main config handler queue (queue takes ownership)
  if (xQueueSend(g_config_handler_queue, &cmd, pdMS_TO_TICKS(50)) != pdTRUE) {
    ESP_LOGW(TAG, "Config callback: queue full, dropping command");
    free(cmd); // Queue full, must free memory
  } else {
    ESP_LOGI(TAG,
             "Config callback: queued config command, type=%d, len=%u, "
             "is_fota=%d",
             cmd->type, cmd->data_len, is_fota);
    // Queue now owns the memory, task will free it after processing
  }
}

/**
 * @brief Config handler task - processes commands from queue
 */
static void config_handler_task(void *arg) {
  config_command_t *cmd = NULL;

  ESP_LOGI(TAG, "Config LAN handler task started");

  while (config_handler_running) {
    // Wait for command pointer from MCU WAN handler callback
    if (xQueueReceive(g_config_handler_queue, &cmd, pdMS_TO_TICKS(100)) ==
        pdTRUE) {
      if (cmd == NULL) {
        ESP_LOGE(TAG, "Received NULL command pointer from queue!");
        continue;
      }

      ESP_LOGI(TAG, "Received config command, type: %d, len: %d", cmd->type,
               cmd->data_len);

      // Route based on command type
      switch (cmd->type) {
      case CONFIG_SET_FIRMWARE_URL: {
        /* CFFU:<url> — save LAN firmware URL to NVS only, no OTA trigger */
        if (cmd->data_len > 5 && cmd->raw_data[4] == ':') {
          const char *url = cmd->raw_data + 5;
          if (url[0] != '\0') {
            fota_lan_handler_set_url(url); /* set_url saves to NVS internally */
            ESP_LOGI(TAG, "LAN firmware URL saved: %s", url);
          }
        }
        break;
      }
      case CONFIG_UPDATE_FIRMWARE: {
        fota_lan_command_t fota_cfg;

        if (config_parse_fota(cmd->raw_data, cmd->data_len, &fota_cfg) ==
            ESP_OK) {
          ESP_LOGI(TAG, "Starting FOTA process...");
          /* Apply the URL parsed from the command (may be default or overridden). */
          fota_lan_handler_set_url(fota_cfg.url);
          // Start FOTA handler task — WiFi AP connect happens inside the task
          led_show_blue();
          mcu_wan_handler_stop();
          fota_lan_handler_task_start();
        } else {
          ESP_LOGE(TAG, "Failed to parse FOTA command");
        }
        break;
      }
      case CONFIG_UPDATE_RS485: {
        if (config_parse_rs485_baud((const uint8_t *)cmd->raw_data,
                                    cmd->data_len) == ESP_OK) {
          ESP_LOGI(TAG, "RS485 baud rate updated from MCU WAN");
        } else {
          ESP_LOGE(TAG, "Failed to parse RS485 baud rate command");
        }
        break;
      }
      case CONFIG_UPDATE_RS485_JSON: {
        if (config_parse_rs485_json((const uint8_t *)cmd->raw_data,
                                    cmd->data_len) == ESP_OK) {
          ESP_LOGI(TAG, "RS485 JSON GPIO config applied");
        } else {
          ESP_LOGE(TAG, "Failed to parse RS485 JSON config");
        }
        break;
      }
      case CONFIG_UPDATE_BLE_JSON: {
        if (config_parse_ble_json((const uint8_t *)cmd->raw_data,
                                  cmd->data_len) == ESP_OK) {
          ESP_LOGI(TAG, "BLE JSON config loaded from WAN MCU");
        } else {
          ESP_LOGE(TAG, "Failed to parse BLE JSON config");
        }
        break;
      }
      case CONFIG_UPDATE_BLE_CMD: {
        if (config_parse_ble_command((const uint8_t *)cmd->raw_data,
                                     cmd->data_len) == ESP_OK) {
          ESP_LOGI(TAG, "BLE command executed successfully");
        } else {
          ESP_LOGE(TAG, "Failed to execute BLE command");
        }
        break;
      }
      case CONFIG_UPDATE_LORA_JSON: {
        if (config_parse_lora_json((const uint8_t *)cmd->raw_data,
                                    cmd->data_len) == ESP_OK) {
          ESP_LOGI(TAG, "LoRa JSON config loaded from WAN MCU");
        } else {
          ESP_LOGE(TAG, "Failed to parse LoRa JSON config");
        }
        break;
      }
      case CONFIG_UPDATE_LORA_CMD: {
        if (config_parse_lora_command((const uint8_t *)cmd->raw_data,
                                       cmd->data_len) == ESP_OK) {
          ESP_LOGI(TAG, "LoRa command executed successfully");
        } else {
          ESP_LOGE(TAG, "Failed to execute LoRa command");
        }
        break;
      }
      case CONFIG_UPDATE_ZIGBEE_JSON: {
        if (config_parse_zigbee_json((const uint8_t *)cmd->raw_data,
                                      cmd->data_len) == ESP_OK) {
          ESP_LOGI(TAG, "Zigbee JSON config loaded from WAN MCU");
        } else {
          ESP_LOGE(TAG, "Failed to parse Zigbee JSON config");
        }
        break;
      }
      case CONFIG_UPDATE_ZIGBEE_CMD: {
        if (config_parse_zigbee_command((const uint8_t *)cmd->raw_data,
                                         cmd->data_len) == ESP_OK) {
          ESP_LOGI(TAG, "Zigbee command executed successfully");
        } else {
          ESP_LOGE(TAG, "Failed to execute Zigbee command");
        }
        break;
      }
      case CONFIG_UPDATE_BLE_NATIVE_JSON: {
        if (config_ble_mode_get() != BLE_MODE_NATIVE) {
          /* Cleanup previous BLE mode before switching */
          if (config_ble_mode_get() == BLE_MODE_GATT) {
            ESP_LOGI(TAG, "Deinitializing BLE GATT handler before switching to Native");
            ble_gatt_handler_deinit();
          }
          config_ble_mode_set(BLE_MODE_NATIVE);
          /* Ensure BLE Native handler is initialized when switching to NATIVE mode */
          esp_err_t init_ret = ble_native_handler_init();
          if (init_ret != ESP_OK) {
            ESP_LOGW(TAG, "BLE Native handler init failed: %s (may already be initialized)", 
                     esp_err_to_name(init_ret));
          }
        }
        if (config_parse_ble_native_json((const uint8_t *)cmd->raw_data,
                                          cmd->data_len) == ESP_OK) {
          /* Save JSON to NVS so it can be restored on next boot */
          if (cmd->data_len > 10) {
            config_save_ble_json_to_nvs(BLE_MODE_NATIVE,
                                        cmd->raw_data + 10,
                                        cmd->data_len - 10);
          }
          ESP_LOGI(TAG, "BLE Native JSON config loaded and saved to NVS");
        } else {
          ESP_LOGE(TAG, "Failed to parse BLE Native JSON config");
        }
        break;
      }
      case CONFIG_UPDATE_BLE_NATIVE_CMD: {
        if (config_ble_mode_get() != BLE_MODE_NATIVE) {
          /* Cleanup previous BLE mode before switching */
          if (config_ble_mode_get() == BLE_MODE_GATT) {
            ESP_LOGI(TAG, "Deinitializing BLE GATT handler before switching to Native");
            ble_gatt_handler_deinit();
          }
          config_ble_mode_set(BLE_MODE_NATIVE);
          esp_err_t init_ret = ble_native_handler_init();
          if (init_ret != ESP_OK) {
            ESP_LOGW(TAG, "BLE Native handler init failed: %s", esp_err_to_name(init_ret));
          }
        }
        if (config_parse_ble_native_command((const uint8_t *)cmd->raw_data,
                                             cmd->data_len) == ESP_OK) {
          ESP_LOGI(TAG, "BLE Native command executed");
        } else {
          ESP_LOGE(TAG, "Failed to execute BLE Native command");
        }
        break;
      }
      case CONFIG_UPDATE_BLE_GATT_JSON: {
        if (config_ble_mode_get() != BLE_MODE_GATT) {
          /* NOTE: Do NOT deinit BLE Native — keep Mesh stack running independent */
          config_ble_mode_set(BLE_MODE_GATT);
        }
        /* Always attempt init — safe no-op if already initialized.
         * Also retries if a previous attempt failed (e.g. NO_MEM). */
        esp_err_t init_ret = ble_gatt_handler_init();
        if (init_ret != ESP_OK) {
          ESP_LOGW(TAG, "BLE GATT handler init failed: %s", esp_err_to_name(init_ret));
        }
        config_parse_ble_gatt_json(cmd->raw_data, cmd->data_len);
        /* Save JSON to NVS so it can be restored on next boot */
        if (cmd->data_len > 10) {
          config_save_ble_json_to_nvs(BLE_MODE_GATT,
                                      cmd->raw_data + 10,
                                      cmd->data_len - 10);
        }
        break;
      }
      case CONFIG_UPDATE_BLE_GATT_CMD: {
        if (config_ble_mode_get() != BLE_MODE_GATT) {
          /* NOTE: Do NOT deinit BLE Native — keep Mesh stack running independent */
          config_ble_mode_set(BLE_MODE_GATT);
        }
        /* Always attempt init — retries if previous attempt failed */
        esp_err_t init_ret = ble_gatt_handler_init();
        if (init_ret != ESP_OK) {
          ESP_LOGW(TAG, "BLE GATT handler init failed: %s", esp_err_to_name(init_ret));
        }
        config_parse_ble_gatt_command(cmd->raw_data, cmd->data_len);
        break;
      }
      default:
        ESP_LOGW(TAG, "Unknown config type: %d", cmd->type);
        break;
      }

      // Free command buffer after processing
      free(cmd);
      cmd = NULL;
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
  // Queue holds pointers to avoid large memory consumption (4KB+ per item)
  if (!g_config_handler_queue) {
    g_config_handler_queue =
        xQueueCreate(CONFIG_QUEUE_SIZE, sizeof(config_command_t*));
    if (!g_config_handler_queue) {
      ESP_LOGE(TAG, "Failed to create config queue");
      return;
    }
  }

  // Register config callback from MCU WAN handler
  mcu_wan_handler_register_config_callback(mcu_wan_config_callback);

  config_handler_running = true;

  // Stack size increased from 4KB to 16KB to handle large config structures (4KB+ each)
  BaseType_t ret = xTaskCreate(config_handler_task, "config_handler", 1024 * 16,
                               NULL, 5, &config_handler_task_handle);

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

/**
 * @brief Restore BLE mode and JSON config from NVS on boot.
 *
 * Called once from app_main() after all peripheral init is done.
 * Initializes the appropriate BLE handler (GATT or Native) and applies the
 * previously saved JSON config so the device is immediately operational
 * without waiting for a new JSON from the WAN MCU.
 *
 * If no BLE config has been saved to NVS yet this is a no-op.
 */
void config_restore_ble_from_nvs(void) {
    uint8_t mode = 0;
    char *json = NULL;
    uint16_t json_len = 0;

    esp_err_t ret = config_load_ble_json_from_nvs(&mode, &json, &json_len);
    if (ret == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "No BLE config in NVS — skipping BLE restore");
        return;
    }
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "BLE NVS load failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Restoring BLE config from NVS (mode=%u, %u bytes)", mode, json_len);

    if (mode == BLE_MODE_NATIVE) {
        config_ble_mode_set(BLE_MODE_NATIVE);
        ret = ble_native_handler_init();
        if (ret == ESP_OK || ret == ESP_ERR_INVALID_STATE /* already init */) {
            ble_native_handler_load_config(0, json, json_len);
            ESP_LOGI(TAG, "BLE Native config restored from NVS");
        } else {
            ESP_LOGE(TAG, "BLE Native init failed during NVS restore: %s", esp_err_to_name(ret));
        }
    } else if (mode == BLE_MODE_GATT) {
        config_ble_mode_set(BLE_MODE_GATT);
        ret = ble_gatt_handler_init();
        if (ret == ESP_OK) {
            ble_gatt_handler_load_config(0, json, json_len);
            ESP_LOGI(TAG, "BLE GATT config restored from NVS");
        } else {
            ESP_LOGE(TAG, "BLE GATT init failed during NVS restore: %s", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGW(TAG, "Unknown BLE mode %u in NVS — ignoring", mode);
    }

    free(json);
}

