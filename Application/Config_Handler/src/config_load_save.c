/**
 * @file config_load_save.c
 * @brief Configuration Load/Save for MCU LAN
 */

#include "config_handler.h"
#include "fota_lan_handler.h"
#include "config_global.h"
#include "config_ble_mode.h"
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
#define NVS_KEY_FOTA_LAN_URL "fota_lan_url"  /* LAN MCU firmware OTA URL */

/* Module JSON Config NVS Keys */
#define NVS_NAMESPACE_MODULE_CONFIG "mod_config"
#define NVS_KEY_STACK0_JSON "stack0_json"
#define NVS_KEY_STACK1_JSON "stack1_json"

/* BLE (GATT / Native) config NVS Keys */
#define NVS_NAMESPACE_BLE_CFG "ble_cfg"
#define NVS_KEY_BLE_MODE      "mode"
#define NVS_KEY_BLE_GATT_JSON "gatt_json"
#define NVS_KEY_BLE_NATIVE_JSON "native_json"

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

/**
 * @brief Save LAN MCU firmware OTA URL to NVS.
 */
esp_err_t config_save_fota_lan_url_to_nvs(void) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open_handle(&handle);
  if (err != ESP_OK) return err;
  err = nvs_set_str(handle, NVS_KEY_FOTA_LAN_URL, fota_lan_handler_get_url());
  if (err == ESP_OK) err = nvs_commit(handle);
  nvs_close(handle);
  if (err == ESP_OK)
    ESP_LOGI(TAG, "FOTA LAN URL saved: %s", fota_lan_handler_get_url());
  return err;
}

/**
 * @brief Load LAN MCU firmware OTA URL from NVS (call at startup).
 */
static esp_err_t load_fota_lan_url_from_nvs(void) {
  nvs_handle_t handle;
  esp_err_t err = nvs_open_handle(&handle);
  if (err != ESP_OK) return err;
  char buf[FOTA_CONFIG_LAN_FIRMWARE_URL_MAX_LEN];
  size_t len = sizeof(buf);
  err = nvs_get_str(handle, NVS_KEY_FOTA_LAN_URL, buf, &len);
  nvs_close(handle);
  if (err == ESP_OK && len > 1) {
    fota_lan_handler_set_url(buf);
    ESP_LOGI(TAG, "FOTA LAN URL loaded: %s", buf);
  } else if (err == ESP_ERR_NVS_NOT_FOUND) {
    err = ESP_OK; /* use default from fota_lan_config.h */
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

  if (required_size == 0 || required_size > 16384) { // Sanity check
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

/**
 * @brief Delete module JSON config from NVS for a specific stack
 *
 * Used when a module swap is detected on boot so that a stale config
 * belonging to the previous module is not applied to the new one.
 *
 * @param stack_id Stack ID (0 or 1)
 * @return esp_err_t ESP_OK on success or if key was already absent
 */
esp_err_t config_delete_module_json_from_nvs(uint8_t stack_id) {
  if (stack_id >= 2) {
    ESP_LOGE(TAG, "Invalid stack_id %d for JSON delete", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE_MODULE_CONFIG, NVS_READWRITE, &handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS namespace for JSON delete: %s", esp_err_to_name(ret));
    return ret;
  }

  const char *key = (stack_id == 0) ? NVS_KEY_STACK0_JSON : NVS_KEY_STACK1_JSON;
  ret = nvs_erase_key(handle, key);

  if (ret == ESP_ERR_NVS_NOT_FOUND) {
    /* Key was already absent — not an error */
    ret = ESP_OK;
  } else if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to erase NVS key for Stack %d: %s", stack_id, esp_err_to_name(ret));
    nvs_close(handle);
    return ret;
  } else {
    ret = nvs_commit(handle);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to commit NVS after JSON erase for Stack %d: %s",
               stack_id, esp_err_to_name(ret));
    } else {
      ESP_LOGI(TAG, "Module JSON erased from NVS for Stack %d", stack_id);
    }
  }

  nvs_close(handle);
  return ret;
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

  // Load FOTA LAN URL
  err = load_fota_lan_url_from_nvs();
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to load FOTA LAN URL");
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
 * @brief Save BLE JSON config (GATT or Native) to NVS.
 *        Also saves the BLE mode so restore knows which handler to start.
 * @param mode  BLE_MODE_GATT or BLE_MODE_NATIVE
 * @param json_str JSON config string
 * @param json_len JSON length
 */
esp_err_t config_save_ble_json_to_nvs(uint8_t mode, const char *json_str, uint16_t json_len) {
  if (!json_str || json_len == 0) return ESP_ERR_INVALID_ARG;

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE_BLE_CFG, NVS_READWRITE, &handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BLE cfg NVS open failed: %s", esp_err_to_name(ret));
    return ret;
  }

  /* Save mode byte */
  ret = nvs_set_u8(handle, NVS_KEY_BLE_MODE, mode);
  if (ret != ESP_OK) { nvs_close(handle); return ret; }

  /* Save JSON blob under the appropriate key */
  const char *key = (mode == BLE_MODE_NATIVE) ? NVS_KEY_BLE_NATIVE_JSON : NVS_KEY_BLE_GATT_JSON;
  ret = nvs_set_blob(handle, key, json_str, json_len);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "BLE JSON NVS write failed: %s", esp_err_to_name(ret));
    nvs_close(handle);
    return ret;
  }

  ret = nvs_commit(handle);
  nvs_close(handle);
  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "BLE JSON saved to NVS (mode=%u, %u bytes)", mode, json_len);
  }
  return ret;
}

/**
 * @brief Load BLE JSON config from NVS.
 * @param[out] mode     BLE mode stored (caller receives BLE_MODE_GATT or BLE_MODE_NATIVE)
 * @param[out] json_str malloc'd buffer containing JSON — caller must free()
 * @param[out] json_len JSON length
 * @return ESP_OK, ESP_ERR_NOT_FOUND if never saved, or error code
 */
esp_err_t config_load_ble_json_from_nvs(uint8_t *mode, char **json_str, uint16_t *json_len) {
  if (!mode || !json_str || !json_len) return ESP_ERR_INVALID_ARG;

  nvs_handle_t handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE_BLE_CFG, NVS_READONLY, &handle);
  if (ret == ESP_ERR_NVS_NOT_FOUND) return ESP_ERR_NOT_FOUND;
  if (ret != ESP_OK) return ret;

  uint8_t saved_mode = 0;
  ret = nvs_get_u8(handle, NVS_KEY_BLE_MODE, &saved_mode);
  if (ret != ESP_OK) { nvs_close(handle); return (ret == ESP_ERR_NVS_NOT_FOUND) ? ESP_ERR_NOT_FOUND : ret; }

  const char *key = (saved_mode == BLE_MODE_NATIVE) ? NVS_KEY_BLE_NATIVE_JSON : NVS_KEY_BLE_GATT_JSON;
  uint32_t size = 0;
  ret = nvs_get_blob(handle, key, NULL, (size_t *)&size);
  if (ret == ESP_ERR_NVS_NOT_FOUND) { nvs_close(handle); return ESP_ERR_NOT_FOUND; }
  if (ret != ESP_OK || size == 0 || size > CONFIG_CMD_MAX_LEN) { nvs_close(handle); return ESP_FAIL; }

  char *buf = malloc(size);
  if (!buf) { nvs_close(handle); return ESP_ERR_NO_MEM; }

  ret = nvs_get_blob(handle, key, buf, (size_t *)&size);
  nvs_close(handle);
  if (ret != ESP_OK) { free(buf); return ret; }

  *mode = saved_mode;
  *json_str = buf;
  *json_len = (uint16_t)size;
  ESP_LOGI(TAG, "BLE JSON loaded from NVS (mode=%u, %u bytes)", saved_mode, (uint16_t)size);
  return ESP_OK;
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
