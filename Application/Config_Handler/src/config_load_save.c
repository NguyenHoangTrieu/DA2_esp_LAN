/**
 * @file config_load_save.c
 * @brief Configuration Load/Save for MCU LAN (CAN, LoRa, Thread, Zigbee)
 */

#include "can_driver.h"
#include "config_handler.h"
#include "esp_log.h"
#include "lora_e32_comm.h"
#include "lora_tdma_connect.h"
#include "lora_tdma_handler.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "rs485_handler.h"
#include <string.h>

static const char *TAG = "CONFIG_NVS";

/* NVS Namespace */
#define NVS_NAMESPACE "lan_gateway"

/* NVS Keys */
#define NVS_KEY_INITIALIZED "initialized"
#define NVS_KEY_CAN_CONFIG "can_cfg"
#define NVS_KEY_CAN_WHITELIST "can_wlist"
#define NVS_KEY_LORA_CONFIG "lora_cfg"
#define NVS_KEY_LORA_CRYPTO "lora_crypto"
#define NVS_KEY_LORA_E32_PARAMS "lora_e32_params"
#define NVS_KEY_LORA_E32_BAUD "lora_e32_baud"
#define NVS_KEY_STACK_1_TYPE "stack1_type"
#define NVS_KEY_STACK_2_TYPE "stack2_type"
#define NVS_KEY_RS485_BAUD "rs485_baud"

/* CAN Whitelist Size */
#define CAN_MAX_WHITELIST_SIZE MAX_WHITELISTED_IDS

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
 * @brief Load CAN configuration from NVS
 */
static esp_err_t config_loadcan_config_from_nvs(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  ESP_LOGI(TAG, "Loading CAN config from NVS...");

  err = nvs_open_handle(&nvs_handle);
  if (err != ESP_OK) {
    return err;
  }

  // Read CAN config (baud_rate and operating_mode)
  typedef struct {
    uint32_t baud_rate;
    can_operating_mode_t operating_mode;
  } can_config_persistent_t;

  can_config_persistent_t can_cfg;
  size_t required_size = sizeof(can_config_persistent_t);

  err = nvs_get_blob(nvs_handle, NVS_KEY_CAN_CONFIG, &can_cfg, &required_size);

  if (err == ESP_OK) {
    // Copy to global config
    g_can_config.baud_rate = can_cfg.baud_rate;
    g_can_config.operating_mode = can_cfg.operating_mode;
    ESP_LOGI(TAG, "CAN config loaded - Baud: %lu, Mode: %d",
             g_can_config.baud_rate, g_can_config.operating_mode);
  } else if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGI(TAG, "CAN config not found in NVS, using defaults");
    err = ESP_OK; // Not an error, use defaults
  } else {
    ESP_LOGE(TAG, "Error reading CAN config: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  // Read CAN whitelist
  typedef struct {
    uint16_t ids[CAN_MAX_WHITELIST_SIZE];
    uint16_t count;
  } can_whitelist_persistent_t;

  can_whitelist_persistent_t whitelist;
  required_size = sizeof(can_whitelist_persistent_t);

  err = nvs_get_blob(nvs_handle, NVS_KEY_CAN_WHITELIST, &whitelist,
                     &required_size);

  if (err == ESP_OK) {
    // Copy to global whitelist
    g_can_whitelist_count = whitelist.count;
    if (g_can_whitelist_count > CAN_MAX_WHITELIST_SIZE) {
      ESP_LOGW(TAG, "Whitelist count %d exceeds max %d, truncating",
               g_can_whitelist_count, CAN_MAX_WHITELIST_SIZE);
      g_can_whitelist_count = CAN_MAX_WHITELIST_SIZE;
    }

    memcpy(g_can_whitelist, whitelist.ids,
           g_can_whitelist_count * sizeof(uint16_t));

    ESP_LOGI(TAG, "CAN whitelist loaded - Count: %d", g_can_whitelist_count);
    for (uint8_t i = 0; i < g_can_whitelist_count; i++) {
      ESP_LOGI(TAG, "  ID[%d]: 0x%03X", i, g_can_whitelist[i]);
    }
  } else if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGI(TAG,
             "CAN whitelist not found in NVS, using empty whitelist (accept "
             "all)");
    g_can_whitelist_count = 0;
    err = ESP_OK; // Not an error, use empty whitelist
  } else {
    ESP_LOGE(TAG, "Error reading CAN whitelist: %s", esp_err_to_name(err));
  }

  nvs_close(nvs_handle);
  return err;
}

/**
 * @brief Load LoRa TDMA configuration from NVS
 *
 * Loads:
 *  - g_lora_handler_cfg (role, IDs, TDMA params)
 *  - g_lora_handler_crypto_key & g_lora_handler_crypto_key_len
 */
static esp_err_t config_load_lora_handler_config_from_nvs(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  ESP_LOGI(TAG, "Loading LoRa TDMA config from NVS...");

  err = nvs_open_handle(&nvs_handle);
  if (err != ESP_OK) {
    return err;
  }

  typedef struct {
    uint8_t role; /* lora_handler_role_t */
    uint16_t node_id;
    uint16_t gateway_id;
    uint8_t num_slots;
    uint8_t my_slot;
    uint32_t slot_duration_ms;
  } lora_config_persistent_t;

  lora_config_persistent_t lora_cfg;
  size_t required_size = sizeof(lora_config_persistent_t);

  err =
      nvs_get_blob(nvs_handle, NVS_KEY_LORA_CONFIG, &lora_cfg, &required_size);

  if (err == ESP_OK) {
    g_lora_handler_cfg.role = (lora_handler_role_t)lora_cfg.role;
    g_lora_handler_cfg.node_id = lora_cfg.node_id;
    g_lora_handler_cfg.gateway_id = lora_cfg.gateway_id;
    g_lora_handler_cfg.num_slots = lora_cfg.num_slots;
    g_lora_handler_cfg.my_slot = lora_cfg.my_slot;
    g_lora_handler_cfg.slot_duration_ms = lora_cfg.slot_duration_ms;

    ESP_LOGI(TAG,
             "LoRa config loaded - role: %d, node: 0x%04X, gw: 0x%04X, "
             "slots: %u, my_slot: %u, slot_ms: %lu",
             g_lora_handler_cfg.role, g_lora_handler_cfg.node_id,
             g_lora_handler_cfg.gateway_id, g_lora_handler_cfg.num_slots,
             g_lora_handler_cfg.my_slot,
             (unsigned long)g_lora_handler_cfg.slot_duration_ms);
  } else if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGI(TAG, "LoRa TDMA config not found in NVS, using defaults in "
                  "g_lora_handler_cfg");
    err = ESP_OK; // Not an error, use defaults
  } else {
    ESP_LOGE(TAG, "Error reading LoRa config: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  /* Load crypto key */
  typedef struct {
    uint8_t key_len;
    uint8_t key[LORA_HANDLER_CRYPTO_KEY_MAX_LEN];
  } lora_crypto_persistent_t;

  lora_crypto_persistent_t crypto;
  required_size = sizeof(lora_crypto_persistent_t);

  err = nvs_get_blob(nvs_handle, NVS_KEY_LORA_CRYPTO, &crypto, &required_size);

  if (err == ESP_OK) {
    if (crypto.key_len > LORA_HANDLER_CRYPTO_KEY_MAX_LEN) {
      ESP_LOGW(TAG, "LoRa crypto key_len %u exceeds max %u, truncating",
               crypto.key_len, (unsigned)LORA_HANDLER_CRYPTO_KEY_MAX_LEN);
      crypto.key_len = LORA_HANDLER_CRYPTO_KEY_MAX_LEN;
    }

    g_lora_handler_crypto_key_len = crypto.key_len;
    if (g_lora_handler_crypto_key_len > 0) {
      memcpy(g_lora_handler_crypto_key, crypto.key,
             g_lora_handler_crypto_key_len);
    }

    ESP_LOGI(TAG, "LoRa crypto key loaded, len=%u",
             g_lora_handler_crypto_key_len);
  } else if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGI(TAG, "LoRa crypto key not found in NVS, using default key");
    err = ESP_OK; // Not an error, use defaults
  } else {
    ESP_LOGE(TAG, "Error reading LoRa crypto key: %s", esp_err_to_name(err));
  }

  nvs_close(nvs_handle);
  return err;
}

/**
 * @brief Save CAN configuration to NVS
 */
esp_err_t config_save_can_config_to_nvs(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  ESP_LOGI(TAG, "Saving CAN config to NVS...");

  err = nvs_open_handle(&nvs_handle);
  if (err != ESP_OK) {
    return err;
  }

  // Prepare CAN config data
  typedef struct {
    uint32_t baud_rate;
    can_operating_mode_t operating_mode;
  } can_config_persistent_t;

  can_config_persistent_t can_cfg = {.baud_rate = g_can_config.baud_rate,
                                     .operating_mode =
                                         g_can_config.operating_mode};

  // Write CAN config blob
  err = nvs_set_blob(nvs_handle, NVS_KEY_CAN_CONFIG, &can_cfg,
                     sizeof(can_config_persistent_t));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error writing CAN config: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  // Prepare whitelist data
  typedef struct {
    uint16_t ids[CAN_MAX_WHITELIST_SIZE];
    uint16_t count;
  } can_whitelist_persistent_t;

  can_whitelist_persistent_t whitelist = {0};
  whitelist.count = g_can_whitelist_count;

  if (whitelist.count > CAN_MAX_WHITELIST_SIZE) {
    ESP_LOGW(TAG, "Whitelist count %d exceeds max %d, truncating",
             whitelist.count, CAN_MAX_WHITELIST_SIZE);
    whitelist.count = CAN_MAX_WHITELIST_SIZE;
  }

  memcpy(whitelist.ids, g_can_whitelist, whitelist.count * sizeof(uint16_t));

  // Write whitelist blob
  err = nvs_set_blob(nvs_handle, NVS_KEY_CAN_WHITELIST, &whitelist,
                     sizeof(can_whitelist_persistent_t));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error writing CAN whitelist: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  // Commit changes
  err = nvs_commit(nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error committing NVS: %s", esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "CAN config saved - Baud: %lu, Mode: %d, Whitelist count: %d",
             g_can_config.baud_rate, g_can_config.operating_mode,
             g_can_whitelist_count);
  }

  nvs_close(nvs_handle);
  return err;
}

/**
 * @brief Load E32 radio configuration (module params + UART baud) from NVS.
 *
 * Only the global context g_lora_e32_params and g_lora_e32_baud_rate is
 * persisted. Other driver settings are compile-time defaults.
 */
static esp_err_t config_loadlora_e32_config_from_nvs(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  ESP_LOGI(TAG, "Loading E32 radio config from NVS...");

  err = nvs_open_handle(&nvs_handle);
  if (err != ESP_OK) {
    return err;
  }

  /* Load module params (e32_params_t) */
  size_t required_size = sizeof(e32_params_t);
  err = nvs_get_blob(nvs_handle, NVS_KEY_LORA_E32_PARAMS, &g_lora_e32_params,
                     &required_size);

  if (err == ESP_OK) {
    ESP_LOGI(TAG, "E32 params loaded from NVS");
  } else if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGI(
        TAG,
        "E32 params not found in NVS, using defaults in g_lora_e32_params");
    err = ESP_OK; /* Not an error, keep defaults */
  } else {
    ESP_LOGE(TAG, "Error reading E32 params: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  /* Load UART baud rate */
  int32_t baud = 0;
  err = nvs_get_i32(nvs_handle, NVS_KEY_LORA_E32_BAUD, &baud);
  if (err == ESP_OK && baud > 0) {
    g_lora_e32_baud_rate = baud;
    ESP_LOGI(TAG, "E32 baud rate loaded from NVS: %ld", (long)baud);
  } else if (err == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGI(TAG, "E32 baud rate not found in NVS, using default %d",
             g_lora_e32_baud_rate);
    err = ESP_OK;
  } else if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error reading E32 baud: %s", esp_err_to_name(err));
  }

  nvs_close(nvs_handle);
  return err;
}

/**
 * @brief Save E32 radio configuration (module params + UART baud) to NVS.
 *
 * Only the minimal set of runtime-adjustable parameters is stored so that
 * the application can change them at runtime and keep them across reboots.
 */
static esp_err_t config_save_lora_e32_config_to_nvs_internal(void) {
  nvs_handle_t nvs_handle;
  esp_err_t err;

  ESP_LOGI(TAG, "Saving E32 radio config to NVS...");

  err = nvs_open_handle(&nvs_handle);
  if (err != ESP_OK) {
    return err;
  }

  /* Store module params blob */
  err = nvs_set_blob(nvs_handle, NVS_KEY_LORA_E32_PARAMS, &g_lora_e32_params,
                     sizeof(e32_params_t));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error writing E32 params: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  /* Store baud rate as signed 32-bit integer */
  err = nvs_set_i32(nvs_handle, NVS_KEY_LORA_E32_BAUD,
                    (int32_t)g_lora_e32_baud_rate);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error writing E32 baud: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  err = nvs_commit(nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error committing E32 config to NVS: %s",
             esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG, "E32 config saved (baud=%d)", g_lora_e32_baud_rate);
  }

  nvs_close(nvs_handle);
  return err;
}

/**
 * @brief Save LoRa TDMA configuration (handler config + crypto key) to NVS
 */
esp_err_t config_save_lora_handler_config_to_nvs(void) {
  /* Save LoRa TDMA handler configuration */
  nvs_handle_t nvs_handle;
  esp_err_t err;

  ESP_LOGI(TAG, "Saving LoRa TDMA config to NVS...");

  err = nvs_open_handle(&nvs_handle);
  if (err != ESP_OK) {
    return err;
  }

  typedef struct {
    uint8_t role; /* lora_handler_role_t */
    uint16_t node_id;
    uint16_t gateway_id;
    uint8_t num_slots;
    uint8_t my_slot;
    uint32_t slot_duration_ms;
  } lora_config_persistent_t;

  lora_config_persistent_t lora_cfg = {
      .role = (uint8_t)g_lora_handler_cfg.role,
      .node_id = g_lora_handler_cfg.node_id,
      .gateway_id = g_lora_handler_cfg.gateway_id,
      .num_slots = g_lora_handler_cfg.num_slots,
      .my_slot = g_lora_handler_cfg.my_slot,
      .slot_duration_ms = g_lora_handler_cfg.slot_duration_ms};

  err = nvs_set_blob(nvs_handle, NVS_KEY_LORA_CONFIG, &lora_cfg,
                     sizeof(lora_config_persistent_t));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error writing LoRa config: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  typedef struct {
    uint8_t key_len;
    uint8_t key[LORA_HANDLER_CRYPTO_KEY_MAX_LEN];
  } lora_crypto_persistent_t;

  lora_crypto_persistent_t crypto = {0};
  crypto.key_len = g_lora_handler_crypto_key_len;

  if (crypto.key_len > LORA_HANDLER_CRYPTO_KEY_MAX_LEN) {
    ESP_LOGW(TAG, "LoRa crypto key_len %u exceeds max %u, truncating",
             crypto.key_len, (unsigned)LORA_HANDLER_CRYPTO_KEY_MAX_LEN);
    crypto.key_len = LORA_HANDLER_CRYPTO_KEY_MAX_LEN;
  }

  if (crypto.key_len > 0) {
    memcpy(crypto.key, g_lora_handler_crypto_key, crypto.key_len);
  }

  err = nvs_set_blob(nvs_handle, NVS_KEY_LORA_CRYPTO, &crypto,
                     sizeof(lora_crypto_persistent_t));
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error writing LoRa crypto key: %s", esp_err_to_name(err));
    nvs_close(nvs_handle);
    return err;
  }

  err = nvs_commit(nvs_handle);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Error committing NVS (LoRa): %s", esp_err_to_name(err));
  } else {
    ESP_LOGI(TAG,
             "LoRa TDMA config saved - role: %d, node: 0x%04X, gw: 0x%04X, "
             "slots: %u, my_slot: %u, slot_ms: %lu, key_len: %u",
             g_lora_handler_cfg.role, g_lora_handler_cfg.node_id,
             g_lora_handler_cfg.gateway_id, g_lora_handler_cfg.num_slots,
             g_lora_handler_cfg.my_slot,
             (unsigned long)g_lora_handler_cfg.slot_duration_ms,
             g_lora_handler_crypto_key_len);
  }

  nvs_close(nvs_handle);
  return err;
}

/**
 * @brief Save only the E32 radio configuration (g_lora_e32_params + baud) to
 * NVS.
 */
esp_err_t config_save_lora_e32_config_to_nvs(void) {
  return config_save_lora_e32_config_to_nvs_internal();
}

/**
 * @brief Load stack type from NVS
 */
esp_err_t config_load_stack_type(uint8_t stack_id, stack_comm_type_t *type) {
  if (stack_id >= STACK_HANDLER_MAX_STACKS || type == NULL) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t nvs_handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
  if (ret != ESP_OK) {
    ESP_LOGW(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
    *type = STACK_COMM_TYPE_NONE;
    return ret;
  }

  const char *key =
      (stack_id == 0) ? NVS_KEY_STACK_1_TYPE : NVS_KEY_STACK_2_TYPE;
  uint8_t value = 0;
  ret = nvs_get_u8(nvs_handle, key, &value);

  if (ret == ESP_OK) {
    *type = (stack_comm_type_t)value;
    ESP_LOGI(TAG, "Stack %d type loaded: %d", stack_id + 1, *type);
  } else if (ret == ESP_ERR_NVS_NOT_FOUND) {
    ESP_LOGI(TAG, "Stack %d type not found, using DEFAULT", stack_id + 1);
    *type = (stack_id == 0) ? g_stack_1_type : g_stack_2_type;
    ret = ESP_OK;
  } else {
    ESP_LOGE(TAG, "Failed to load stack type: %s", esp_err_to_name(ret));
    *type = (stack_id == 0) ? g_stack_1_type : g_stack_2_type;
  }

  nvs_close(nvs_handle);
  return ret;
}

/**
 * @brief Save stack type to NVS
 */
esp_err_t config_save_stack_type(uint8_t stack_id, stack_comm_type_t type) {
  if (stack_id >= STACK_HANDLER_MAX_STACKS) {
    ESP_LOGE(TAG, "Invalid stack ID: %d", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t nvs_handle;
  esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
    return ret;
  }

  const char *key =
      (stack_id == 0) ? NVS_KEY_STACK_1_TYPE : NVS_KEY_STACK_2_TYPE;
  ret = nvs_set_u8(nvs_handle, key, (uint8_t)type);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to save stack type: %s", esp_err_to_name(ret));
    nvs_close(nvs_handle);
    return ret;
  }

  ret = nvs_commit(nvs_handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
  } else {
    ESP_LOGI(TAG, "Stack %d type saved: %d", stack_id + 1, type);
  }

  nvs_close(nvs_handle);
  return ret;
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
 */
static esp_err_t config_loadall_configs_from_nvs(void) {
  esp_err_t err;

  ESP_LOGI(TAG, "Loading all configurations from NVS...");

  // Load CAN config
  err = config_loadcan_config_from_nvs();
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to load CAN config");
  }

  // Load LoRa TDMA config
  err = config_load_lora_handler_config_from_nvs();
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to load LoRa TDMA config");
  }

  // Load E32 radio config (module params + baud)
  err = config_loadlora_e32_config_from_nvs();
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to load LoRa TDMA config");
  }

  esp_err_t ret1 = config_load_stack_type(0, &g_stack_1_type);
  esp_err_t ret2 = config_load_stack_type(1, &g_stack_2_type);

  if (ret1 == ESP_OK && ret2 == ESP_OK) {
    ESP_LOGI(TAG, "Stack types loaded: ST1=%d, ST2=%d", g_stack_1_type,
             g_stack_2_type);
  }

  // Load RS485 baud rate
  err = config_load_rs485_baud(&g_rs485_baud_rate);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Failed to load RS485 baud rate");
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
 */
esp_err_t config_init(void) {
  ESP_LOGI(TAG, "Initializing LAN MCU configuration system...");

  if (is_first_boot()) {
    ESP_LOGI(TAG, "First boot detected - saving default configuration");

    // Save default CAN config to NVS
    config_save_can_config_to_nvs();

    // Save default LoRa TDMA config to NVS
    config_save_lora_handler_config_to_nvs();
    config_save_lora_e32_config_to_nvs();

    // Save stack config to NVS
    config_save_stack_type(0, g_stack_1_type);
    config_save_stack_type(1, g_stack_2_type);

    config_save_rs485_baud(g_rs485_baud_rate);

    mark_initialized();
    ESP_LOGI(TAG, "Default configuration saved");
  } else {
    ESP_LOGI(TAG, "Loading existing configuration");
    config_loadall_configs_from_nvs();
  }

  return ESP_OK;
}
