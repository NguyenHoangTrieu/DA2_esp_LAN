/**
 * @file module_monitor_task.c
 * @brief Module Monitor Task Implementation
 */

#include "module_monitor_task.h"
#include "config_handler.h"
#include "config_global.h"
#include "stack_handler.h"
#include "ble_handler_task.h"
#include "mcu_wan_handler.h"
#include "cJSON.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "MODULE_MONITOR";

/* ===== Configuration ===== */

#define MODULE_MONITOR_TASK_STACK_SIZE 4096
#define MODULE_MONITOR_TASK_PRIORITY 3
#define MODULE_MONITOR_CONFIG_QUEUE_SIZE 10
#define MODULE_MONITOR_MAX_STACKS 2

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
                                              uint16_t json_len,
                                              module_type_t *module_type);
static esp_err_t module_start_handler_task(uint8_t stack_id,
                                           module_type_t module_type);
static esp_err_t module_stop_handler_task(uint8_t stack_id);
static esp_err_t module_monitor_start_handler(uint8_t stack_id);

/* ===== Implementation ===== */

esp_err_t module_monitor_task_start(void) {
  if (g_monitor_state.initialized) {
    ESP_LOGW(TAG, "Monitor task already initialized");
    return ESP_OK;
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

  // 1. Read current hardware module IDs
  const char *cur_id[MODULE_MONITOR_MAX_STACKS] = {
    stack_handler_get_module_id(0),
    stack_handler_get_module_id(1),
  };

  // 2. Load previously saved IDs from NVS into config globals.
  //    If no NVS entry exists (first boot) the globals keep their default
  //    "000" value, so every stack will be treated as newly inserted.
  config_load_global_vars_from_nvs();

  const char *old_id[MODULE_MONITOR_MAX_STACKS] = {
    config_get_stack_1_id(),  // Stack 0 is stored as stack_1_id
    config_get_stack_2_id(),  // Stack 1 is stored as stack_2_id
  };

  // 3. Compare IDs: invalidate NVS JSON config for any stack whose module
  //    has been swapped since the last boot.
  bool id_changed[MODULE_MONITOR_MAX_STACKS] = {false, false};
  for (int i = 0; i < MODULE_MONITOR_MAX_STACKS; i++) {
    if (strcmp(cur_id[i], old_id[i]) != 0) {
      ESP_LOGW(TAG, "Stack %d module changed: '%s' -> '%s', clearing NVS config",
               i, old_id[i], cur_id[i]);
      config_delete_module_json_from_nvs(i);
      id_changed[i] = true;
    } else {
      ESP_LOGI(TAG, "Stack %d module unchanged: '%s'", i, cur_id[i]);
    }
  }

  // 4. Persist the current (new) IDs to NVS for comparison on the next boot.
  config_set_stack_1_id(cur_id[0]);
  config_set_stack_2_id(cur_id[1]);
  config_save_global_vars_to_nvs();

  ESP_LOGI(TAG, "Module IDs: Stack_1=%s, Stack_2=%s",
           config_get_stack_1_id(), config_get_stack_2_id());

  // 5. Load saved JSON only for stacks whose module ID has not changed.
  for (int i = 0; i < MODULE_MONITOR_MAX_STACKS; i++) {
    if (id_changed[i]) {
      ESP_LOGI(TAG, "Stack %d: module was replaced, waiting for new config", i);
      continue;
    }

    char *json_str = NULL;
    uint16_t json_len = 0;

    if (config_load_module_json_from_nvs(i, &json_str, &json_len) == ESP_OK) {
      ESP_LOGI(TAG, "Loaded saved config for Stack %d from NVS", i);
      if (module_parse_json_config(i, json_str, json_len) == ESP_OK) {
        g_monitor_state.module_info[i].is_configured = true;
        free(json_str); // module_parse_json_config makes its own copy

        // Auto-start handler task for restored config (no queue message on boot)
        esp_err_t start_ret = module_monitor_start_handler(i);
        if (start_ret == ESP_OK) {
          ESP_LOGI(TAG, "Handler auto-started for Stack %d (NVS restore)", i);

          // Load config into handler → triggers HW reset + enter CMD mode
          // (same sequence as when receiving JSON from WAN MCU)
          module_info_t *info = &g_monitor_state.module_info[i];
          if (info->module_type == MODULE_TYPE_BLE) {
            esp_err_t cfg_ret = ble_handler_task_load_config(i,
                                                             info->json_config_str,
                                                             info->json_config_len);
            if (cfg_ret != ESP_OK) {
              ESP_LOGE(TAG, "Failed to load config into BLE handler for Stack %d", i);
            } else {
              ESP_LOGI(TAG, "BLE handler config loaded after NVS restore (Stack %d)", i);
            }
          }
        } else {
          ESP_LOGW(TAG, "Handler start failed for Stack %d: %s", i,
                   esp_err_to_name(start_ret));
        }
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

esp_err_t module_monitor_send_config(uint8_t stack_id, const char *json_str, uint16_t json_len) {
  if (!json_str || json_len == 0) {
    ESP_LOGE(TAG, "Invalid config parameters");
    return ESP_ERR_INVALID_ARG;
  }

  if (stack_id >= MODULE_MONITOR_MAX_STACKS) {
    ESP_LOGE(TAG, "Invalid stack_id %u", stack_id);
    return ESP_ERR_INVALID_ARG;
  }

  if (!g_monitor_state.initialized || !g_monitor_state.config_queue) {
    ESP_LOGE(TAG, "Monitor task not running");
    return ESP_ERR_INVALID_STATE;
  }

  // Allocate JSON string copy
  char *json_copy = malloc(json_len + 1);
  if (!json_copy) {
    ESP_LOGE(TAG, "Failed to allocate memory for JSON config");
    return ESP_ERR_NO_MEM;
  }

  memcpy(json_copy, json_str, json_len);
  json_copy[json_len] = '\0';

  // Create config message
  module_config_msg_t msg = {
    .stack_id = stack_id,
    .json_str = json_copy,
    .json_len = json_len
  };

  // Send to queue (5 second timeout)
  if (xQueueSend(g_monitor_state.config_queue, &msg, pdMS_TO_TICKS(5000)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to enqueue config for Stack %u", stack_id);
    free(json_copy);
    return ESP_FAIL;
  }

  ESP_LOGI(TAG, "Config enqueued for Stack %u (%u bytes)", stack_id, json_len);
  return ESP_OK;
}

static esp_err_t module_monitor_start_handler(uint8_t stack_id) {
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
  esp_err_t ret = module_detect_type_from_json(json_str, json_len, &module_type);
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
  info->is_configured = true;

  // Also update config_global so mcu_wan_handler can report correct json_len
  if (stack_id == 0) {
    config_set_stack_1_json(json_str, json_len);
  } else if (stack_id == 1) {
    config_set_stack_2_json(json_str, json_len);
  }

  ESP_LOGI(TAG, "Config parsed for Stack %d: type=%d", stack_id, module_type);

  xSemaphoreGive(g_monitor_state.mutex);
  return ESP_OK;
}

/**
 * @brief Detect module type from JSON (parse "module_type" field)
 */
static esp_err_t module_detect_type_from_json(const char *json_str,
                                              uint16_t json_len,
                                              module_type_t *module_type) {
  if (!json_str || json_len == 0 || !module_type) {
    return ESP_ERR_INVALID_ARG;
  }

  // CRITICAL: Create null-terminated copy for cJSON_Parse
  // cJSON requires null-terminated string
  char *json_copy = malloc(json_len + 1);
  if (!json_copy) {
    ESP_LOGE(TAG, "Failed to allocate memory for JSON copy");
    return ESP_ERR_NO_MEM;
  }
  memcpy(json_copy, json_str, json_len);
  json_copy[json_len] = '\0';

  cJSON *root = cJSON_Parse(json_copy);
  if (!root) {
    // Check for NULL bytes in original data
    size_t actual_len = strnlen(json_str, json_len);
    if (actual_len < json_len) {
      ESP_LOGE(TAG, "JSON contains NULL byte at position %zu (expected %u) - DATA CORRUPTION!", actual_len, json_len);
    }
  }
  free(json_copy);
  if (!root) {
    const char *error_ptr = cJSON_GetErrorPtr();
    if (error_ptr != NULL) {
      // Find position in original string
      size_t error_pos = error_ptr - json_str;
      ESP_LOGE(TAG, "JSON parse error at position %zu", error_pos);
      ESP_LOGE(TAG, "JSON parse error before: %s", error_ptr);
      
      // Log context around error (50 chars before and after)
      if (error_pos > 50) {
        ESP_LOGE(TAG, "Context: ...%.50s >>> ERROR >>> %.50s...", 
                 json_str + error_pos - 50, error_ptr);
      } else {
        ESP_LOGE(TAG, "Context: %.50s >>> ERROR >>> %.50s...", 
                 json_str, error_ptr);
      }
    } else {
      ESP_LOGE(TAG, "JSON parse error (no error pointer)");
    }
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
    return ble_handler_task_start(stack_id);

  case MODULE_TYPE_ZIGBEE:
    ESP_LOGI(TAG, "Starting Zigbee handler for Stack %d", stack_id);
    // TODO: Call zigbee_handler_task_start(stack_id) when implemented
    ESP_LOGW(TAG, "Zigbee handler not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;

  case MODULE_TYPE_LORA:
    ESP_LOGI(TAG, "Starting LoRa handler for Stack %d", stack_id);
    // TODO: Call lora_handler_task_start(stack_id) when implemented
    ESP_LOGW(TAG, "LoRa handler not yet implemented");
    return ESP_ERR_NOT_SUPPORTED;

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
    return ble_handler_task_stop(stack_id);

  case MODULE_TYPE_ZIGBEE:
    // TODO: Call zigbee_handler_task_stop(stack_id) when implemented
    return ESP_OK;

  case MODULE_TYPE_LORA:
    // TODO: Call lora_handler_task_stop(stack_id) when implemented
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
      ESP_LOGI(TAG, "Received config for Stack %d (%u bytes)", msg.stack_id, msg.json_len);

      // Parse and apply config
      esp_err_t ret =
          module_parse_json_config(msg.stack_id, msg.json_str, msg.json_len);
      if (ret == ESP_OK) {
        // Save to NVS for persistence
        config_save_module_json_to_nvs(msg.stack_id, msg.json_str,
                                       msg.json_len);

        // Start handler task if not already running
        module_info_t *info = &g_monitor_state.module_info[msg.stack_id];
        bool handler_already_running = info->is_running;

        if (!handler_already_running) {
          ret = module_monitor_start_handler(msg.stack_id);
          if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start handler for Stack %d", msg.stack_id);
            uint8_t error_resp[] = "CFBL:JSON:FAIL:START";
            mcu_wan_enqueue_uplink(HANDLER_BLE, error_resp, sizeof(error_resp) - 1);
            free(msg.json_str);
            continue;
          }
        } else {
          ESP_LOGI(TAG, "Handler already running for Stack %d, reloading config", msg.stack_id);
        }

        // Load config into handler task (works for both fresh start and reload)
        if (info->module_type == MODULE_TYPE_BLE) {
          esp_err_t cfg_ret = ble_handler_task_load_config(msg.stack_id, 
                                                            info->json_config_str, 
                                                            info->json_config_len);
          if (cfg_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to load config into BLE handler for Stack %d", msg.stack_id);
            uint8_t error_resp[] = "CFBL:JSON:FAIL:LOAD";
            mcu_wan_enqueue_uplink(HANDLER_BLE, error_resp, sizeof(error_resp) - 1);
          } else {
            ESP_LOGI(TAG, "%s config loaded for Stack %d",
                     handler_already_running ? "Reloaded" : "Handler started and", msg.stack_id);
            uint8_t ok_resp[] = "CFBL:JSON:OK";
            mcu_wan_enqueue_uplink(HANDLER_BLE, ok_resp, sizeof(ok_resp) - 1);
          }
        } else {
          ESP_LOGI(TAG, "Handler %s for Stack %d",
                   handler_already_running ? "config reloaded" : "started successfully", msg.stack_id);
          uint8_t ok_resp[] = "CFBL:JSON:OK";
          mcu_wan_enqueue_uplink(HANDLER_BLE, ok_resp, sizeof(ok_resp) - 1);
        }
      } else {
        ESP_LOGE(TAG, "Failed to parse config for Stack %d", msg.stack_id);
        // Send failure response
        uint8_t error_resp[] = "CFBL:JSON:FAIL:PARSE";
        mcu_wan_enqueue_uplink(HANDLER_BLE, error_resp, sizeof(error_resp) - 1);
      }

      free(msg.json_str);
    }

    // Periodic checks (e.g., handler task health, config updates)
    // Could add LED status indication here
  }

  ESP_LOGI(TAG, "Monitor task exiting");
  vTaskDelete(NULL);
}
