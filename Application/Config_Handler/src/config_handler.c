/**
 * @file config_handler.c
 * @brief Configuration handler implementation for LAN MCU
 */

#include "config_handler.h"
#include "DA2_esp_LAN.h"
#include "fota_lan_config.h"
#include "fota_lan_handler.h"
#include "lora_e32_comm.h"
#include "lora_tdma_handler.h"
#include "mcu_wan_handler.h"
#include <string.h>

static const char *TAG = "config_handler";

extern lora_e32_comm_handle_t g_lora_e32_handle;
// Queue handle
QueueHandle_t g_config_handler_queue = NULL;

static bool config_handler_running = false;
static TaskHandle_t config_handler_task_handle = NULL;
static esp_err_t config_parse_fota(const char *data, uint16_t len,
                                   fota_lan_command_t *cfg);
static void mcu_wan_config_callback(const uint8_t *data, uint16_t len,
                                    bool is_fota);
/**
 * @brief Parse command type from 2-character prefix
 */
config_type_t config_parse_type(const char *cmd, uint16_t len) {
  if (len < 4 || cmd[0] != 'C' || cmd[1] != 'F') {
    return CONFIG_TYPE_UNKNOWN;
  }

  // Check first 2 characters
  if (cmd[2] == 'F' && cmd[3] == 'W') {
    return CONFIG_UPDATE_FIRMWARE;
  } else if (cmd[2] == 'L' && cmd[3] == 'R') {
    return CONFIG_UPDATE_LORA;
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
 * @brief Parse LoRa/E32 configuration frames coming from MCU WAN.
 *
 * Supported frame formats (ASCII prefix + binary payload):
 *
 *   1) CFLR:MODEM:<6 bytes>
 *      - 6 bytes follow the prefix and map directly to e32_params_t:
 *          [0] head   (0xC0/0xC2)
 *          [1] addh   (address high)
 *          [2] addl   (address low)
 *          [3] sped   (UART & air rate)
 *          [4] chan   (RF channel)
 *          [5] option (options)
 *      Action:
 *        - Copy into g_lora_e32_params
 *        - Save to NVS via save_lora_e32_config_to_nvs()
 *        - If LoRa E32 driver is initialized (g_lora_e32_handle != NULL),
 *          push params to radio via lora_e32_comm_write_params().
 *
 *   2) CFLR:HDLCF:<11 bytes>
 *      - 11 bytes carry LoRa TDMA handler configuration:
 *          [0]  role              (lora_handler_role_t)
 *          [1]  node_id high
 *          [2]  node_id low
 *          [3]  gateway_id high
 *          [4]  gateway_id low
 *          [5]  num_slots
 *          [6]  my_slot
 *          [7]  slot_duration_ms byte3 (MSB)
 *          [8]  slot_duration_ms byte2
 *          [9]  slot_duration_ms byte1
 *          [10] slot_duration_ms byte0 (LSB)
 *      Action:
 *        - Update g_lora_handler_cfg
 *        - Save to NVS via save_lora_handler_config_to_nvs()
 *
 *   3) CFLR:CRYPT:<1 + N bytes>
 *      - Crypto key frame:
 *          [0] key_len (number of bytes that follow)
 *          [1..N] key bytes
 *      Action:
 *        - Update g_lora_handler_crypto_key_len and g_lora_handler_crypto_key[]
 *        - Save to NVS via save_lora_handler_config_to_nvs()
 *
 * All frames are expected to be passed *without* the outer [CF][length(2)]
 * WAN header. The buffer here must start at 'C' of "CFLR:...".
 */
esp_err_t config_parse_lora(const uint8_t *data, uint16_t len) {
  if (data == NULL || len < 5) {
    ESP_LOGE(TAG, "LoRa config: invalid buffer");
    return ESP_ERR_INVALID_ARG;
  }

  // Common prefix check
  if (len < 5 || memcmp(data, "CFLR:", 5) != 0) {
    ESP_LOGE(TAG, "LoRa config: missing CFLR prefix");
    return ESP_FAIL;
  }

  // MODEM config: CFLR:MODEM:<6 bytes>
  const char *modem_prefix = "CFLR:MODEM:";
  size_t modem_prefix_len = strlen(modem_prefix);

  if (len >= modem_prefix_len + sizeof(e32_params_t) &&
      strncmp((const char *)data, modem_prefix, modem_prefix_len) == 0) {

    const uint8_t *p = data + modem_prefix_len;

    e32_params_t params = {0};
    // e32_params_t is exactly 6 bytes packed (head, addh, addl, sped, chan,
    // option)
    memcpy(&params, p, sizeof(e32_params_t));

    g_lora_e32_params = params;

    ESP_LOGI(TAG,
             "LoRa MODEM config: head=0x%02X, addr=0x%02X%02X, sped=0x%02X, "
             "chan=0x%02X, option=0x%02X",
             params.head, params.addh, params.addl, params.sped, params.chan,
             params.option);

    // Persist to NVS
    esp_err_t err = save_lora_e32_config_to_nvs();
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save E32 config to NVS: %s",
               esp_err_to_name(err));
      return err;
    }

    // Push parameters to the E32 module if radio handle is available
    if (g_lora_e32_handle != NULL) {
      lora_e32_comm_status_t st =
          lora_e32_comm_write_params(g_lora_e32_handle, &g_lora_e32_params);
      if (st != LORA_E32_COMM_OK) {
        ESP_LOGE(TAG, "Failed to write params to E32 module (status=%d)", st);
        return ESP_FAIL;
      }
      ESP_LOGI(TAG, "E32 module parameters updated from WAN");
    } else {
      ESP_LOGW(TAG, "E32 handle not initialized, skipping write to module");
    }

    return ESP_OK;
  }

  // HDLCF (LoRa handler) config: CFLR:HDLCF:<11 bytes>
  const char *hdlc_prefix = "CFLR:HDLCF:";
  size_t hdlc_prefix_len = strlen(hdlc_prefix);

  if (len >= hdlc_prefix_len + 11 &&
      strncmp((const char *)data, hdlc_prefix, hdlc_prefix_len) == 0) {

    const uint8_t *p = data + hdlc_prefix_len;

    uint8_t role = p[0];
    uint16_t node_id = ((uint16_t)p[1] << 8) | p[2];
    uint16_t gateway_id = ((uint16_t)p[3] << 8) | p[4];
    uint8_t num_slots = p[5];
    uint8_t my_slot = p[6];
    uint32_t slot_ms = ((uint32_t)p[7] << 24) | ((uint32_t)p[8] << 16) |
                       ((uint32_t)p[9] << 8) | (uint32_t)p[10];

    g_lora_handler_cfg.role = (lora_handler_role_t)role;
    g_lora_handler_cfg.node_id = node_id;
    g_lora_handler_cfg.gateway_id = gateway_id;
    g_lora_handler_cfg.num_slots = num_slots;
    g_lora_handler_cfg.my_slot = my_slot;
    g_lora_handler_cfg.slot_duration_ms = slot_ms;

    ESP_LOGI(TAG,
             "LoRa TDMA config: role=%u, node=0x%04X, gw=0x%04X, slots=%u, "
             "my_slot=%u, slot_ms=%lu",
             (unsigned)role, node_id, gateway_id, num_slots, my_slot,
             (unsigned long)slot_ms);

    esp_err_t err = save_lora_handler_config_to_nvs();
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save LoRa TDMA config to NVS: %s",
               esp_err_to_name(err));
      return err;
    }

    return ESP_OK;
  }

  // CRYPT config: CFLR:CRYPT:<len+key>
  const char *crypt_prefix = "CFLR:CRYPT:";
  size_t crypt_prefix_len = strlen(crypt_prefix);

  if (len >= crypt_prefix_len + 1 &&
      strncmp((const char *)data, crypt_prefix, crypt_prefix_len) == 0) {

    const uint8_t *p = data + crypt_prefix_len;
    uint16_t payload_len = len - crypt_prefix_len;

    uint8_t key_len = p[0];
    if (key_len == 0 || key_len > LORA_HANDLER_CRYPTO_KEY_MAX_LEN) {
      ESP_LOGE(TAG, "LoRa CRYPT: invalid key_len=%u", key_len);
      return ESP_FAIL;
    }

    if (payload_len < (uint16_t)(1 + key_len)) {
      ESP_LOGE(TAG, "LoRa CRYPT: buffer too short for key_len=%u", key_len);
      return ESP_FAIL;
    }

    g_lora_handler_crypto_key_len = key_len;
    memcpy(g_lora_handler_crypto_key, &p[1], key_len);

    ESP_LOGI(TAG, "LoRa crypto key updated, len=%u", key_len);

    esp_err_t err = save_lora_handler_config_to_nvs();
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to save LoRa crypto config to NVS: %s",
               esp_err_to_name(err));
      return err;
    }

    return ESP_OK;
  }

  ESP_LOGW(TAG, "Unknown CFLR frame (len=%u)", (unsigned)len);
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
    ESP_LOGW(TAG, "Config callback: invalid data");
    return;
  }

  if (g_config_handler_queue == NULL) {
    ESP_LOGW(TAG, "Config callback: config queue not initialized");
    return;
  }

  config_command_t cmd;
  memset(&cmd, 0, sizeof(cmd));

  // Parse command type from raw data
  cmd.type = config_parse_type((const char *)data, len);

  // Clamp raw data length to CONFIG_CMD_MAX_LEN
  if (len > CONFIG_CMD_MAX_LEN) {
    ESP_LOGW(TAG, "Config callback: input length %u truncated to max %d bytes",
             len, CONFIG_CMD_MAX_LEN);
    cmd.data_len = CONFIG_CMD_MAX_LEN;
  } else {
    cmd.data_len = len;
  }

  // Copy raw config payload
  memcpy(cmd.raw_data, data, cmd.data_len);

  // Enqueue command to the main config handler queue
  if (xQueueSend(g_config_handler_queue, &cmd, pdMS_TO_TICKS(50)) != pdTRUE) {
    ESP_LOGW(TAG, "Config callback: queue full, dropping command");
  } else {
    ESP_LOGI(TAG,
             "Config callback: queued config command, type=%d, len=%u, "
             "is_fota=%d",
             cmd.type, cmd.data_len, is_fota);
  }
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
      case CONFIG_UPDATE_LORA: {
        if (config_parse_lora((const uint8_t *)cmd.raw_data, cmd.data_len) ==
            ESP_OK) {
          ESP_LOGI(TAG, "LoRa config updated from MCU WAN");
        } else {
          ESP_LOGE(TAG, "Failed to parse LoRa config frame");
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

  // Register config callback from MCU WAN handler
  mcu_wan_handler_register_config_callback(mcu_wan_config_callback);

  config_handler_running = true;

  BaseType_t ret = xTaskCreate(config_handler_task, "config_handler", 4096,
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
