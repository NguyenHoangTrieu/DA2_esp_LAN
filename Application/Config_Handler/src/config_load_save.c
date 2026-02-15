/**
 * @file config_load_save.c
 * @brief Configuration Load/Save for MCU LAN
 */

#include "config_handler.h"
#include "config_global.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "rs485_handler.h"
#include <string.h>

static const char *TAG = "CONFIG_NVS";

/* NVS Namespace */
#define NVS_NAMESPACE "lan_gateway"

/* NVS Keys */
#define NVS_KEY_INITIALIZED "initialized"
#define NVS_KEY_RS485_BAUD "rs485_baud"
#define NVS_KEY_STACK_1_ID "stack1_id"
#define NVS_KEY_STACK_2_ID "stack2_id"

/* Module JSON Config NVS Keys */
#define NVS_NAMESPACE_MODULE_CONFIG "mod_config"
#define NVS_KEY_STACK0_JSON "stack0_json"
#define NVS_KEY_STACK1_JSON "stack1_json"

/**
 * @brief Open NVS handle
 */
static esp_err_t nvs_open_handle(nvs_handle_t *handle) {
  esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
  }
  return err;
}

/* ===== Module JSON Config NVS Functions ===== */

/**
 * @brief Save module JSON config to NVS
 */
esp_err_t config_save_module_json_to_nvs(uint8_t stack_id, const char *json_str, uint16_t json_len) {
  if (stack_id > 1 || !json_str || json_len == 0) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE_MODULE_CONFIG, NVS_READWRITE, &handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
    return ret;
  }

  const char *key = (stack_id == 0) ? NVS_KEY_STACK0_JSON : NVS_KEY_STACK1_JSON;

  // Store JSON string
  ret = nvs_set_blob(handle, key, (const void *)json_str, json_len);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to write JSON to NVS: %s", esp_err_to_name(ret));
    nvs_close(handle);
    return ret;
  }

  // Commit
  ret = nvs_commit(handle);
  nvs_close(handle);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Module JSON saved to NVS for Stack %d (%u bytes)", stack_id, json_len);
  } else {
    ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
  }

  return ret;
}

/**
 * @brief Load module JSON config from NVS
 */
esp_err_t config_load_module_json_from_nvs(uint8_t stack_id, char **json_str, uint16_t *json_len) {
  if (stack_id > 1 || !json_str || !json_len) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE_MODULE_CONFIG, NVS_READWRITE, &handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
    return ret;
  }

  const char *key = (stack_id == 0) ? NVS_KEY_STACK0_JSON : NVS_KEY_STACK1_JSON;
  uint32_t required_size = 0;

  // Get size first
  ret = nvs_get_blob(handle, key, NULL, (size_t *)&required_size);
  if (ret == ESP_ERR_NVS_NOT_FOUND) {
    nvs_close(handle);
    return ESP_ERR_NOT_FOUND;
  }
  if (ret != ESP_OK) {
    nvs_close(handle);
    return ret;
  }

  if (required_size == 0 || required_size > 8192) { // Sanity check
    nvs_close(handle);
    ESP_LOGE(TAG, "Invalid JSON size: %lu", (unsigned long)required_size);
    return ESP_ERR_INVALID_ARG;
  }

  // Allocate buffer
  char *buffer = (char *)malloc(required_size);
  if (!buffer) {
    nvs_close(handle);
    return ESP_ERR_NO_MEM;
  }

  // Read blob
  ret = nvs_get_blob(handle, key, buffer, (size_t *)&required_size);

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NVS blob read error: %s", esp_err_to_name(ret));

    // Handle NVS corruption - erase corrupted data
    if (ret == ESP_ERR_NVS_INVALID_LENGTH || ret == ESP_ERR_NVS_INVALID_NAME) {
      ESP_LOGW(TAG, "Detected NVS corruption for Stack %d, erasing corrupted config", stack_id);
      nvs_erase_key(handle, key);
      nvs_commit(handle);
    }

    nvs_close(handle);
    free(buffer);
    return ret;
  }

  nvs_close(handle);

  *json_str = buffer;
  *json_len = (uint16_t)required_size;

  ESP_LOGI(TAG, "Module JSON loaded from NVS for Stack %d (%u bytes)", stack_id, *json_len);
  return ESP_OK;
}

/* ===== Global Config Variables NVS Functions ===== */

/**
 * @brief Save global config variables to NVS
 */
esp_err_t config_save_global_vars_to_nvs(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  ESP_LOGI(TAG, "Saving global config to NVS...");

  err = nvs_open_handle(&nvs_handle);
  if (err != ESP_OK) {
    return err;
  }

  // Save stack IDs
  err = nvs_set_str(nvs_handle, NVS_KEY_STACK_1_ID, config_get_stack_1_id());
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to save stack_1_id");
    nvs_close(nvs_handle);
    return err;
  }

  err = nvs_set_str(nvs_handle, NVS_KEY_STACK_2_ID, config_get_stack_2_id());
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to save stack_2_id");
    nvs_close(nvs_handle);
    return err;
  }

  // Commit
  err = nvs_commit(nvs_handle);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "Global config saved: stack1_id=%s, stack2_id=%s",
             config_get_stack_1_id(), config_get_stack_2_id());
  }

  nvs_close(nvs_handle);
  return err;
}

/**
 * @brief Load global config variables from NVS
 */
esp_err_t config_load_global_vars_from_nvs(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err;
  char buffer[8];
  size_t len;

  ESP_LOGI(TAG, "Loading global config from NVS...");

  err = nvs_open_handle(&nvs_handle);
  if (err != ESP_OK) {
    return err;
  }

  // Load stack_1_id
  len = sizeof(buffer);
  err = nvs_get_str(nvs_handle, NVS_KEY_STACK_1_ID, buffer, &len);
  if (err == ESP_OK) {
    config_set_stack_1_id(buffer);
  } else if (err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "Failed to load stack_1_id: %s", esp_err_to_name(err));
  }

  // Load stack_2_id
  len = sizeof(buffer);
  err = nvs_get_str(nvs_handle, NVS_KEY_STACK_2_ID, buffer, &len);
  if (err == ESP_OK) {
    config_set_stack_2_id(buffer);
  } else if (err != ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGW(TAG, "Failed to load stack_2_id: %s", esp_err_to_name(err));
  }

  nvs_close(nvs_handle);
  ESP_LOGI(TAG, "Global config loaded: stack1_id=%s, stack2_id=%s",
           config_get_stack_1_id(), config_get_stack_2_id());
  return ESP_OK;
}

/**
 * @brief Load RS485 baud rate from NVS
 */
esp_err_t config_load_rs485_baud(uint32_t *baud_rate) {
  if (baud_rate == NULL) {
    ESP_LOGE(TAG, "Invalid argument");
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t nvs_handle;
  esp_err_t err;

  ESP_LOGI(TAG, "Loading RS485 baud rate from NVS...");

  err = nvs_open_handle(&nvs_handle);
  if (err != ESP_OK) {
    return err;
  }

  // Read baud rate as uint32_t
  err = nvs_get_u32(nvs_handle, NVS_KEY_RS485_BAUD, baud_rate);
  if (err == ESP_OK) {
    ESP_LOGI(TAG, "RS485 baud rate loaded: %lu", (unsigned long)*baud_rate);
  } else if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGI(TAG, "RS485 baud rate not found in NVS, using default");
    err = ESP_OK; // Not an error, use default
  } else {
    ESP_LOGE(TAG, "Error reading RS485 baud: %s", esp_err_to_name(err));
  }

  nvs_close(nvs_handle);
  return err;
}

/**
 * @brief Save RS485 baud rate to NVS
 */
esp_err_t config_save_rs485_baud(uint32_t baud_rate) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  ESP_LOGI(TAG, "Saving RS485 baud rate to NVS...");

  err = nvs_open_handle(&nvs_handle);
  if (err != ESP_OK) {
    return err;
  }

  // Write baud rate as uint32_t
  err = nvs_set_u32(nvs_handle, NVS_KEY_RS485_BAUD, baud_rate);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error writing RS485 baud: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  err = nvs_commit(nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error committing RS485 baud to NVS: %s",
             esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "RS485 baud rate saved: %lu", (unsigned long)baud_rate);
  }

  nvs_close(nvs_handle);
  return err;
}

/**
 * @brief Load all configurations from NVS (call at startup)
 * Simple version for Module Base Setting trial
 */
static esp_err_t config_loadall_configs_from_nvs(void) {
  esp_err_t err;

  ESP_LOGI(TAG, "Loading configurations from NVS...");

  // Load RS485 baud rate
  err = config_load_rs485_baud(&g_rs485_baud_rate);
  if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
    ESP_LOGW(TAG, "Failed to load RS485 baud rate");
  }

  // Load global config variables (stack IDs)
  err = config_load_global_vars_from_nvs();
  if (err != ESP_OK && err != ESP_ERR_NOT_FOUND) {
    ESP_LOGW(TAG, "Failed to load global config variables");
  }

  ESP_LOGI(TAG, "Configuration loading complete");
  return ESP_OK;
}

/**
 * @brief Erase all gateway configurations from NVS
 */
esp_err_t erase_all_configs_from_nvs(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  ESP_LOGW(TAG, "Erasing all configurations from NVS...");

  err = nvs_open_handle(&nvs_handle);
  if (err != ESP_OK) {
    return err;
  }

  // Erase all keys in the namespace
  err = nvs_erase_all(nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error erasing NVS: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  // Commit changes
  err = nvs_commit(nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error committing NVS erase: %s", esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "All configurations erased from NVS");
  }

  nvs_close(nvs_handle);
  return err;
}

/**
 * @brief Check if this is first boot
 */
static bool is_first_boot(void) {
  nvs_handle_t handle;
  uint8_t initialized = 0;

  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
    esp_err_t err = nvs_get_u8(handle, NVS_KEY_INITIALIZED, &initialized);
    nvs_close(handle);
    return (err != ESP_OK || initialized == 0);
  }

  return true; // Namespace doesn't exist = first boot
}

/**
 * @brief Mark system as initialized
 */
static void mark_initialized(void) {
  nvs_handle_t handle;

  if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) == ESP_OK) {
    nvs_set_u8(handle, NVS_KEY_INITIALIZED, 1);
    nvs_commit(handle);
    nvs_close(handle);
  }
}

/**
 * @brief Initialize configuration - auto-saves defaults on first boot
 * Module Base Setting trial version (BLE + RS485 only)
 */
esp_err_t config_init(void) {
  ESP_LOGI(TAG, "Initializing LAN MCU configuration system (Module Base Setting trial)...");

  if (is_first_boot()) {
    ESP_LOGI(TAG, "First boot detected - saving default configuration");

    // Save RS485 baud rate default
    config_save_rs485_baud(g_rs485_baud_rate);

    // Save default global config variables
    config_save_global_vars_to_nvs();

    mark_initialized();
    ESP_LOGI(TAG, "Default configuration saved");
  } else {
    ESP_LOGI(TAG, "Loading existing configuration");
    config_loadall_configs_from_nvs();
  }

  return ESP_OK;
}
