/**
 * @file module_monitor_task.c
 * @brief Module Monitor Task Implementation
 */

#include "module_monitor_task.h"
#include "cJSON.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "MODULE_MONITOR";

/* ===== Configuration ===== */

#define MODULE_MONITOR_TASK_STACK_SIZE 4096
#define MODULE_MONITOR_TASK_PRIORITY 3
#define MODULE_MONITOR_CONFIG_QUEUE_SIZE 10
#define MODULE_MONITOR_MAX_STACKS 2 // Stack 0 and Stack 1
#define NVS_NAMESPACE_MODULE_CONFIG "mod_config"
#define NVS_KEY_STACK0_JSON "stack0_json"
#define NVS_KEY_STACK1_JSON "stack1_json"

/* ===== Global State ===== */

static struct {
  bool initialized;
  TaskHandle_t monitor_task_handle;
  QueueHandle_t config_queue;
  SemaphoreHandle_t mutex;
  module_info_t module_info[MODULE_MONITOR_MAX_STACKS]; // Stack 0 and Stack 1
} g_monitor_state = {0};

/* ===== Module Config Queue Types ===== */

typedef struct {
  uint8_t stack_id;
  char *json_str; // Allocated string
  uint16_t json_len;
} module_config_msg_t;

/* ===== Forward Declarations ===== */

static void module_monitor_task_impl(void *pvParameters);
static esp_err_t module_parse_json_config(uint8_t stack_id,
                                          const char *json_str,
                                          uint16_t json_len);
static esp_err_t module_detect_type_from_json(const char *json_str,
                                              module_type_t *module_type);
static esp_err_t module_start_handler_task(uint8_t stack_id,
                                           module_type_t module_type);
static esp_err_t module_stop_handler_task(uint8_t stack_id);

/* ===== NVS Helper Functions ===== */

/**
 * @brief Open NVS handle
 */
static esp_err_t nvs_open_config(nvs_handle_t *handle) {
  return nvs_open(NVS_NAMESPACE_MODULE_CONFIG, NVS_READWRITE, handle);
}

/**
 * @brief Get NVS key for stack
 */
static const char *nvs_get_key(uint8_t stack_id) {
  return (stack_id == 0) ? NVS_KEY_STACK0_JSON : NVS_KEY_STACK1_JSON;
}

/* ===== Implementation ===== */

esp_err_t module_monitor_task_start(void) {
  if (g_monitor_state.initialized) {
    ESP_LOGW(TAG, "Monitor task already initialized");
    return ESP_OK;
  }

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_LOGI(TAG, "NVS partition needs erasing");
    nvs_flash_erase();
    ret = nvs_flash_init();
  }
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
    return ret;
  }

  // Create mutex
  g_monitor_state.mutex = xSemaphoreCreateMutex();
  if (!g_monitor_state.mutex) {
    ESP_LOGE(TAG, "Failed to create mutex");
    return ESP_ERR_NO_MEM;
  }

  // Create config queue
  g_monitor_state.config_queue = xQueueCreate(MODULE_MONITOR_CONFIG_QUEUE_SIZE,
                                              sizeof(module_config_msg_t));
  if (!g_monitor_state.config_queue) {
    ESP_LOGE(TAG, "Failed to create config queue");
    vSemaphoreDelete(g_monitor_state.mutex);
    return ESP_ERR_NO_MEM;
  }

  // Initialize module info for both stacks
  for (int i = 0; i < MODULE_MONITOR_MAX_STACKS; i++) {
    g_monitor_state.module_info[i].stack_id = i;
    g_monitor_state.module_info[i].module_type = MODULE_TYPE_NONE;
    g_monitor_state.module_info[i].is_configured = false;
    g_monitor_state.module_info[i].is_running = false;
    g_monitor_state.module_info[i].handler_status = HANDLER_STATUS_STOPPED;
    g_monitor_state.module_info[i].config_data = NULL;
    g_monitor_state.module_info[i].json_config_str = NULL;
    g_monitor_state.module_info[i].json_config_len = 0;
  }

  g_monitor_state.initialized = true;

  // Try to load saved configs from NVS
  for (int i = 0; i < MODULE_MONITOR_MAX_STACKS; i++) {
    char *json_str = NULL;
    uint16_t json_len = 0;

    if (module_monitor_load_config_from_nvs(i, &json_str, &json_len) ==
        ESP_OK) {
      ESP_LOGI(TAG, "Loaded saved config for Stack %d from NVS", i);
      if (module_parse_json_config(i, json_str, json_len) == ESP_OK) {
        g_monitor_state.module_info[i].is_configured = true;
      } else {
        ESP_LOGW(TAG, "Failed to parse saved config for Stack %d", i);
        free(json_str);
      }
    }
  }

  // Create and start monitor task
  BaseType_t task_ret = xTaskCreate(module_monitor_task_impl, "module_monitor",
                                    MODULE_MONITOR_TASK_STACK_SIZE, NULL,
                                    MODULE_MONITOR_TASK_PRIORITY,
                                    &g_monitor_state.monitor_task_handle);

  if (task_ret != pdPASS) {
    ESP_LOGE(TAG, "Failed to create monitor task");
    vQueueDelete(g_monitor_state.config_queue);
    vSemaphoreDelete(g_monitor_state.mutex);
    g_monitor_state.initialized = false;
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Module monitor task started");
  return ESP_OK;
}

esp_err_t module_monitor_task_stop(void) {
  if (!g_monitor_state.initialized) {
    return ESP_ERR_INVALID_STATE;
  }

  // Stop all handler tasks
  for (int i = 0; i < MODULE_MONITOR_MAX_STACKS; i++) {
    if (g_monitor_state.module_info[i].is_running) {
      module_stop_handler_task(i);
    }
  }

  // Delete task
  if (g_monitor_state.monitor_task_handle) {
    vTaskDelete(g_monitor_state.monitor_task_handle);
    g_monitor_state.monitor_task_handle = NULL;
  }

  // Cleanup queues and mutex
  if (g_monitor_state.config_queue) {
    vQueueDelete(g_monitor_state.config_queue);
    g_monitor_state.config_queue = NULL;
  }

  if (g_monitor_state.mutex) {
    vSemaphoreDelete(g_monitor_state.mutex);
    g_monitor_state.mutex = NULL;
  }

  // Cleanup module configs
  for (int i = 0; i < MODULE_MONITOR_MAX_STACKS; i++) {
    if (g_monitor_state.module_info[i].json_config_str) {
      free(g_monitor_state.module_info[i].json_config_str);
      g_monitor_state.module_info[i].json_config_str = NULL;
    }
    if (g_monitor_state.module_info[i].config_data) {
      free(g_monitor_state.module_info[i].config_data);
      g_monitor_state.module_info[i].config_data = NULL;
    }
  }

  g_monitor_state.initialized = false;
  ESP_LOGI(TAG, "Module monitor task stopped");
  return ESP_OK;
}

esp_err_t module_monitor_load_config(uint8_t stack_id, const char *json_str,
                                     uint16_t json_len) {
  if (!g_monitor_state.initialized || !json_str || json_len == 0 ||
      stack_id > 1) {
    return ESP_ERR_INVALID_ARG;
  }

  // Allocate and copy JSON string
  char *json_copy = (char *)malloc(json_len + 1);
  if (!json_copy) {
    return ESP_ERR_NO_MEM;
  }

  memcpy(json_copy, json_str, json_len);
  json_copy[json_len] = '\0';

  // Create message
  module_config_msg_t msg = {
      .stack_id = stack_id, .json_str = json_copy, .json_len = json_len};

  // Send to monitor task queue
  if (xQueueSend(g_monitor_state.config_queue, &msg, pdMS_TO_TICKS(1000)) !=
      pdTRUE) {
    ESP_LOGE(TAG, "Failed to send config to monitor queue");
    free(json_copy);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Config queued for Stack %d (%u bytes)", stack_id, json_len);
  return ESP_OK;
}

module_type_t module_monitor_get_stack_type(uint8_t stack_id) {
  if (stack_id > 1 || !g_monitor_state.initialized) {
    return MODULE_TYPE_UNKNOWN;
  }

  if (xSemaphoreTake(g_monitor_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    module_type_t type = g_monitor_state.module_info[stack_id].module_type;
    xSemaphoreGive(g_monitor_state.mutex);
    return type;
  }

  return MODULE_TYPE_UNKNOWN;
}

bool module_monitor_is_configured(uint8_t stack_id) {
  if (stack_id > 1 || !g_monitor_state.initialized) {
    return false;
  }

  if (xSemaphoreTake(g_monitor_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    bool configured = g_monitor_state.module_info[stack_id].is_configured;
    xSemaphoreGive(g_monitor_state.mutex);
    return configured;
  }

  return false;
}

esp_err_t module_monitor_start_handler(uint8_t stack_id) {
  if (stack_id > 1 || !g_monitor_state.initialized) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xSemaphoreTake(g_monitor_state.mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }

  module_info_t *info = &g_monitor_state.module_info[stack_id];

  if (!info->is_configured || info->is_running) {
    xSemaphoreGive(g_monitor_state.mutex);
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t ret = module_start_handler_task(stack_id, info->module_type);
  if (ret == ESP_OK) {
    info->is_running = true;
    info->handler_status = HANDLER_STATUS_RUNNING;
  }

  xSemaphoreGive(g_monitor_state.mutex);
  return ret;
}

esp_err_t module_monitor_stop_handler(uint8_t stack_id) {
  if (stack_id > 1 || !g_monitor_state.initialized) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xSemaphoreTake(g_monitor_state.mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }

  module_info_t *info = &g_monitor_state.module_info[stack_id];

  if (!info->is_running) {
    xSemaphoreGive(g_monitor_state.mutex);
    return ESP_ERR_INVALID_STATE;
  }

  esp_err_t ret = module_stop_handler_task(stack_id);
  if (ret == ESP_OK) {
    info->is_running = false;
    info->handler_status = HANDLER_STATUS_STOPPED;
  }

  xSemaphoreGive(g_monitor_state.mutex);
  return ret;
}

handler_status_t module_monitor_get_handler_status(uint8_t stack_id) {
  if (stack_id > 1 || !g_monitor_state.initialized) {
    return HANDLER_STATUS_ERROR;
  }

  if (xSemaphoreTake(g_monitor_state.mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    handler_status_t status =
        g_monitor_state.module_info[stack_id].handler_status;
    xSemaphoreGive(g_monitor_state.mutex);
    return status;
  }

  return HANDLER_STATUS_ERROR;
}

const module_info_t *module_monitor_get_info(uint8_t stack_id) {
  if (stack_id > 1 || !g_monitor_state.initialized) {
    return NULL;
  }

  return &g_monitor_state.module_info[stack_id];
}

esp_err_t module_monitor_save_config_to_nvs(uint8_t stack_id,
                                            const char *json_str,
                                            uint16_t json_len) {
  if (stack_id > 1 || !json_str || json_len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle;
  esp_err_t ret = nvs_open_config(&handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
    return ret;
  }

  const char *key = nvs_get_key(stack_id);

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
    ESP_LOGI(TAG, "Config saved to NVS for Stack %d (%u bytes)", stack_id,
             json_len);
  } else {
    ESP_LOGE(TAG, "Failed to commit NVS: %s", esp_err_to_name(ret));
  }

  return ret;
}

esp_err_t module_monitor_load_config_from_nvs(uint8_t stack_id, char **json_str,
                                              uint16_t *json_len) {
  if (stack_id > 1 || !json_str || !json_len) {
    return ESP_ERR_INVALID_ARG;
  }

  nvs_handle_t handle;
  esp_err_t ret = nvs_open_config(&handle);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(ret));
    return ret;
  }

  const char *key = nvs_get_key(stack_id);
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
      ESP_LOGW(TAG,
               "Detected NVS corruption for Stack %d, erasing corrupted config",
               stack_id);
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

  ESP_LOGI(TAG, "Config loaded from NVS for Stack %d (%u bytes)", stack_id,
           *json_len);
  return ESP_OK;
}

/* ===== Internal Helper Functions ===== */

/**
 * @brief Parse JSON and validate
 */
static esp_err_t module_parse_json_config(uint8_t stack_id,
                                          const char *json_str,
                                          uint16_t json_len) {
  if (!json_str || json_len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  // Detect module type
  module_type_t module_type = MODULE_TYPE_NONE;
  esp_err_t ret = module_detect_type_from_json(json_str, &module_type);
  if (ret != ESP_OK || module_type == MODULE_TYPE_NONE) {
    ESP_LOGE(TAG, "Failed to detect module type from JSON");
    return ESP_FAIL;
  }

  // Update module info
  if (xSemaphoreTake(g_monitor_state.mutex, pdMS_TO_TICKS(500)) != pdTRUE) {
    return ESP_ERR_TIMEOUT;
  }

  module_info_t *info = &g_monitor_state.module_info[stack_id];

  // Free old config
  if (info->json_config_str) {
    free(info->json_config_str);
  }

  // Store new config
  info->json_config_str = (char *)malloc(json_len + 1);
  if (!info->json_config_str) {
    xSemaphoreGive(g_monitor_state.mutex);
    return ESP_ERR_NO_MEM;
  }

  memcpy(info->json_config_str, json_str, json_len);
  info->json_config_str[json_len] = '\0';
  info->json_config_len = json_len;
  info->module_type = module_type;

  ESP_LOGI(TAG, "Config parsed for Stack %d: type=%d", stack_id, module_type);

  xSemaphoreGive(g_monitor_state.mutex);
  return ESP_OK;
}

/**
 * @brief Detect module type from JSON (parse "module_type" field)
 */
static esp_err_t module_detect_type_from_json(const char *json_str,
                                              module_type_t *module_type) {
  if (!json_str || !module_type) {
    return ESP_ERR_INVALID_ARG;
  }

  cJSON *root = cJSON_Parse(json_str);
  if (!root) {
    ESP_LOGE(TAG, "Failed to parse JSON");
    return ESP_FAIL;
  }

  cJSON *type_item = cJSON_GetObjectItem(root, "module_type");
  if (!type_item || !type_item->valuestring) {
    ESP_LOGE(TAG, "Missing module_type in JSON");
    cJSON_Delete(root);
    return ESP_FAIL;
  }

  const char *type_str = type_item->valuestring;
  if (strcmp(type_str, "BLE") == 0) {
    *module_type = MODULE_TYPE_BLE;
  } else if (strcmp(type_str, "ZIGBEE") == 0) {
    *module_type = MODULE_TYPE_ZIGBEE;
  } else if (strcmp(type_str, "LORA") == 0) {
    *module_type = MODULE_TYPE_LORA;
  } else {
    ESP_LOGW(TAG, "Unknown module_type: %s", type_str);
    *module_type = MODULE_TYPE_UNKNOWN;
    cJSON_Delete(root);
    return ESP_FAIL;
  }

  cJSON_Delete(root);
  return ESP_OK;
}

/**
 * @brief Start appropriate handler task based on module type
 */
static esp_err_t module_start_handler_task(uint8_t stack_id,
                                           module_type_t module_type) {
  switch (module_type) {
  case MODULE_TYPE_BLE:
    ESP_LOGI(TAG, "Starting BLE handler for Stack %d", stack_id);
    // Call to ble_handler_task_start() - will be implemented in Task 1.2
    // return ble_handler_task_start(stack_id);
    return ESP_OK; // TODO: Implement when BLE task is ready

  case MODULE_TYPE_ZIGBEE:
    ESP_LOGI(TAG, "Starting Zigbee handler for Stack %d", stack_id);
    // Call to zigbee_handler_task_start() - for future
    return ESP_OK;

  case MODULE_TYPE_LORA:
    ESP_LOGI(TAG, "Starting LoRa handler for Stack %d", stack_id);
    // Call to lora_handler_task_start() - for future
    return ESP_OK;

  default:
    ESP_LOGE(TAG, "Unknown module type: %d", module_type);
    return ESP_FAIL;
  }
}

/**
 * @brief Stop handler task for a stack
 */
static esp_err_t module_stop_handler_task(uint8_t stack_id) {
  module_info_t *info = &g_monitor_state.module_info[stack_id];

  ESP_LOGI(TAG, "Stopping handler for Stack %d (type=%d)", stack_id,
           info->module_type);

  switch (info->module_type) {
  case MODULE_TYPE_BLE:
    // Call ble_handler_task_stop() - will be implemented
    return ESP_OK;

  case MODULE_TYPE_ZIGBEE:
    // Call zigbee_handler_task_stop() - for future
    return ESP_OK;

  case MODULE_TYPE_LORA:
    // Call lora_handler_task_stop() - for future
    return ESP_OK;

  default:
    return ESP_FAIL;
  }
}

/* ===== Main Monitor Task ===== */

/**
 * @brief Main monitor task implementation
 *
 * Waits for config messages and processes them
 */
static void module_monitor_task_impl(void *pvParameters) {
  ESP_LOGI(TAG, "Monitor task running");

  module_config_msg_t msg;

  while (g_monitor_state.initialized) {
    // Wait for config message (timeout 5 seconds for periodic checks)
    if (xQueueReceive(g_monitor_state.config_queue, &msg,
                      pdMS_TO_TICKS(5000)) == pdTRUE) {
      ESP_LOGI(TAG, "Received config for Stack %d", msg.stack_id);

      // Parse and apply config
      esp_err_t ret =
          module_parse_json_config(msg.stack_id, msg.json_str, msg.json_len);
      if (ret == ESP_OK) {
        // Save to NVS for persistence
        module_monitor_save_config_to_nvs(msg.stack_id, msg.json_str,
                                          msg.json_len);

        // Auto-start handler task
        ret = module_monitor_start_handler(msg.stack_id);
        if (ret != ESP_OK) {
          ESP_LOGE(TAG, "Failed to start handler for Stack %d", msg.stack_id);
        }
      } else {
        ESP_LOGE(TAG, "Failed to parse config for Stack %d", msg.stack_id);
      }

      free(msg.json_str);
    }

    // Periodic checks (e.g., handler task health, config updates)
    // Could add LED status indication here
  }

  ESP_LOGI(TAG, "Monitor task exiting");
  vTaskDelete(NULL);
}
