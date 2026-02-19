/**
 * @file ble_handler.c
 * @brief BLE Handler Middleware Implementation
 */

#include "ble_handler.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "json_ble_config_parser.h"
#include "module_config_controller.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "BLE_HANDLER";

/* ===== Configuration Constants ===== */

#define BLE_BINARY_CMD_MARKER 0xC0   // Binary protocol marker
#define BLE_CMD_MAX_LEN 128          // Max command string length
#define BLE_RESPONSE_MAX_LEN 256     // Max response buffer size
#define BLE_HEX_DATA_MAX_LEN 512     // Max hex data buffer size
#define BLE_MAX_STACKS 2             // Number of stacks (0 and 1)

/* ===== Static Data ===== */

static struct {
  bool initialized;
  ble_module_config_t config[BLE_MAX_STACKS]; // Stack 0 and Stack 1
} g_ble_handler = {0};

static bool g_module_ctrl_initialized = false;

// Mutex to protect g_ble_handler from multi-stack race conditions (Fix Issue
// #2)
static SemaphoreHandle_t g_ble_handler_mutex = NULL;

/* ===== Helper Functions ===== */

/**
 * @brief Validate stack ID
 */
static bool ble_is_valid_stack_id(uint8_t stack_id) {
  return (stack_id == 0 || stack_id == 1);
}

static comm_port_type_t ble_get_comm_port(uint8_t stack_id) {
  if (!ble_is_valid_stack_id(stack_id)) {
    return COMM_PORT_MAX;
  }

  const char *port = g_ble_handler.config[stack_id].comm_port_type;
  if (strcmp(port, "uart") == 0) {
    return COMM_PORT_UART;
  }
  if (strcmp(port, "spi") == 0) {
    return COMM_PORT_SPI;
  }
  if (strcmp(port, "i2c") == 0) {
    return COMM_PORT_I2C;
  }
  if (strcmp(port, "usb") == 0) {
    return COMM_PORT_USB;
  }
  return COMM_PORT_MAX;
}

static bool ble_parse_pin_id(const char *pin_str, uint8_t *pin_out) {
  if (!pin_str || !pin_out) {
    return false;
  }

  const char *digits = pin_str;
  if (strncmp(pin_str, "GPIO", 4) == 0) {
    digits = pin_str + 4;
  }

  if (*digits == '\0') {
    return false;
  }

  char *end_ptr = NULL;
  long pin_val = strtol(digits, &end_ptr, 10);
  if (end_ptr == digits || pin_val < 0 || pin_val > 8) {
    return false;
  }

  *pin_out = (uint8_t)pin_val;
  return true;
}

/**
 * @brief Get function config by ID
 */
static ble_function_config_t *
ble_get_function_config(uint8_t stack_id, ble_function_id_t func_id) {
  if (!ble_is_valid_stack_id(stack_id) || func_id >= BLE_FUNC_COUNT) {
    return NULL;
  }
  return &g_ble_handler.config[stack_id].functions[func_id];
}

/**
 * @brief Validate command string for buffer overflow protection
 *
 * Checks:
 * - Command length within limits
 * - ASCII printable characters (for AT commands)
 * - Binary format detection (0xC0 prefix)
 *
 * @return true if valid, false if suspicious
 */
static bool ble_validate_command_string(const char *cmd, size_t max_len) {
  if (!cmd) {
    return false;
  }

  size_t cmd_len = strlen(cmd);

  // Length check
  if (cmd_len == 0 || cmd_len > max_len) {
    ESP_LOGW(TAG, "Command length invalid: %zu (max: %zu)", cmd_len, max_len);
    return false;
  }

  // Check for binary format marker (0xC0 0xC0 ...)
  if (cmd[0] == (char)BLE_BINARY_CMD_MARKER) {
    ESP_LOGD(TAG, "Binary format command detected (0x%02X prefix)",
             BLE_BINARY_CMD_MARKER);
    return true; // Binary commands always valid if length OK
  }

  // ASCII validation for AT commands
  if (strncmp(cmd, "AT", 2) == 0) {
    // Check for printable ASCII characters
    for (size_t i = 0; i < cmd_len; i++) {
      if (cmd[i] < 0x20 || cmd[i] > 0x7E) {
        if (cmd[i] != '\r' && cmd[i] != '\n') {
          ESP_LOGW(TAG,
                   "Non-printable character in AT command at index %zu: 0x%02X",
                   i, (uint8_t)cmd[i]);
          return false;
        }
      }
    }
    return true;
  }

  // Generic string validation (allow most printable chars)
  for (size_t i = 0; i < cmd_len; i++) {
    if (cmd[i] < 0x20 || cmd[i] > 0x7E) {
      if (cmd[i] != '\r' && cmd[i] != '\n' && cmd[i] != '\t') {
        ESP_LOGW(TAG, "Suspicious character in command at index %zu: 0x%02X", i,
                 (uint8_t)cmd[i]);
        return false;
      }
    }
  }

  return true;
}

/**
 * @brief Execute a BLE function with optional parameter
 *
 * Uses Module_Config_Controller wrapper layer for:
 * 1. GPIO control (via module_gpio_write)
 * 2. UART communication (via module_bus_write/read)
 * 3. Proper error handling and initialization checks
 *
 * @param stack_id Stack ID
 * @param func_id Function ID
 * @param param Optional parameter (NULL if not needed)
 * @param result Output result
 * @return ESP_OK on success
 */
static esp_err_t ble_execute_function_internal(uint8_t stack_id,
                                               ble_function_id_t func_id,
                                               const char *param,
                                               ble_exec_result_t *result) {
  if (!ble_is_valid_stack_id(stack_id) || func_id >= BLE_FUNC_COUNT) {
    if (result)
      result->status = ESP_ERR_INVALID_ARG;
    return ESP_ERR_INVALID_ARG;
  }

  ble_function_config_t *func_cfg = ble_get_function_config(stack_id, func_id);
  if (!func_cfg || !func_cfg->available) {
    ESP_LOGW(TAG, "Function %d not configured for stack %d", func_id, stack_id);
    if (result)
      result->status = ESP_ERR_NOT_SUPPORTED;
    return ESP_ERR_NOT_SUPPORTED;
  }

  ESP_LOGD(TAG, "Executing BLE function %d on stack %d", func_id, stack_id);

  esp_err_t ret = ESP_OK;
  TickType_t start_tick = xTaskGetTickCount();

  // Step 1: Execute GPIO start sequences via Module_Config_Controller wrapper
  for (uint8_t i = 0; i < func_cfg->gpio_start_count; i++) {
    char pin_str[8]; // Format: "0X" where 0 is stack, X is pin (increased to
                     // prevent truncation)
    snprintf(pin_str, sizeof(pin_str), "%d%d", stack_id,
             func_cfg->gpio_start[i]);
    bool state = func_cfg->gpio_start_state[i];

    ret = module_gpio_write(stack_id, pin_str, state);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to control GPIO pin %s: %s", pin_str,
               esp_err_to_name(ret));
      if (result)
        result->status = ret;
      return ret;
    }
    ESP_LOGD(TAG, "GPIO pin %s set to %d", pin_str, state);
  }

  // Step 2: Wait delay_start_ms using FreeRTOS
  if (func_cfg->delay_start_ms > 0) {
    ESP_LOGD(TAG, "Waiting %lu ms before command", func_cfg->delay_start_ms);
    vTaskDelay(pdMS_TO_TICKS(func_cfg->delay_start_ms));
  }

  // Build command with parameter if provided
  char final_command[BLE_CMD_MAX_LEN] = {0};
  if (param && strstr(func_cfg->command, "{PARAM}")) {
    // Replace {PARAM} placeholder with actual parameter
    char *src = func_cfg->command;
    char *dest = final_command;
    size_t dest_remaining = sizeof(final_command) - 1;

    while (*src && dest_remaining > 0) {
      if (strncmp(src, "{PARAM}", 7) == 0) {
        size_t param_len = strlen(param);
        if (param_len > dest_remaining) {
          ESP_LOGE(TAG, "Parameter too long");
          if (result)
            result->status = ESP_ERR_INVALID_SIZE;
          return ESP_ERR_INVALID_SIZE;
        }
        memcpy(dest, param, param_len);
        dest += param_len;
        dest_remaining -= param_len;
        src += 7;
      } else {
        *dest++ = *src++;
        dest_remaining--;
      }
    }
    *dest = '\0';
  } else {
    strncpy(final_command, func_cfg->command, sizeof(final_command) - 1);
  }

  // Check for GPIO-only functions (no command, no response)
  size_t cmd_len = strlen(final_command);
  size_t expect_len = strlen(func_cfg->expect_response);
  bool is_gpio_only = (cmd_len == 0 && expect_len == 0);

  if (is_gpio_only) {
    // GPIO-only function: skip command sending, just execute sequences
    ESP_LOGI(TAG, "GPIO-only function %d - no command/response expected", func_id);
    
    // Execute GPIO end sequences
    for (uint8_t i = 0; i < func_cfg->gpio_end_count; i++) {
      char pin_str[8];
      snprintf(pin_str, sizeof(pin_str), "%d%d", stack_id, func_cfg->gpio_end[i]);
      bool state = func_cfg->gpio_end_state[i];

      ret = module_gpio_write(stack_id, pin_str, state);
      if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to control GPIO end pin %s: %s", pin_str,
                 esp_err_to_name(ret));
      }
    }

    // Wait delay_end_ms
    if (func_cfg->delay_end_ms > 0) {
      ESP_LOGD(TAG, "Waiting %lu ms after GPIO sequences", func_cfg->delay_end_ms);
      vTaskDelay(pdMS_TO_TICKS(func_cfg->delay_end_ms));
    }

    uint32_t exec_time = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
    if (result) {
      result->status = ESP_OK;
      snprintf(result->response, sizeof(result->response), "GPIO_OK");
      result->response_len = 7;
      result->execution_time_ms = exec_time;
    }

    ESP_LOGI(TAG, "GPIO-only function %d completed on stack %d (took %lu ms)",
             func_id, stack_id, exec_time);
    return ESP_OK;
  }

  // Normal command execution: validate command string
  if (!ble_validate_command_string(final_command, sizeof(final_command))) {
    ESP_LOGE(TAG, "Command validation failed for function %d", func_id);
    if (result)
      result->status = ESP_ERR_INVALID_ARG;
    return ESP_ERR_INVALID_ARG;
  }

  // Step 3: Send AT command via Module_Config_Controller wrapper
  ESP_LOGD(TAG, "Sending command: %s", final_command);
  comm_port_type_t port_type = ble_get_comm_port(stack_id);
  if (port_type == COMM_PORT_MAX) {
    ESP_LOGE(TAG, "Invalid comm port type for stack %d", stack_id);
    if (result)
      result->status = ESP_ERR_INVALID_STATE;
    return ESP_ERR_INVALID_STATE;
  }

  ret = module_bus_write(stack_id, port_type, (const uint8_t *)final_command,
                         strlen(final_command));
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send command: %s", esp_err_to_name(ret));
    if (result)
      result->status = ret;
    return ret;
  }

  // Skip module_bus_read if no response expected and timeout is 0
  uint8_t response_buffer[BLE_RESPONSE_MAX_LEN] = {0};
  size_t response_len = 0;
  bool skip_read = (expect_len == 0 && func_cfg->timeout_ms == 0);

  if (!skip_read) {
    // Step 4: Wait for response with timeout
    ret = module_bus_read(stack_id, port_type, response_buffer,
                          sizeof(response_buffer) - 1, func_cfg->timeout_ms,
                          &response_len);
    if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
      ESP_LOGE(TAG, "Failed to receive response: %s", esp_err_to_name(ret));
      if (result)
        result->status = ret;
      return ret;
    }

    // Step 5: Verify response matches expect_response
    bool response_valid = false;
    if (response_len > 0) {
      response_buffer[response_len] = '\0';
      ESP_LOGD(TAG, "Received response: %s", (char *)response_buffer);

      // Check if response contains expected string
      if (expect_len == 0 ||
          strstr((const char *)response_buffer, func_cfg->expect_response) !=
              NULL) {
        response_valid = true;
      }
    }

    if (!response_valid && expect_len > 0) {
      ESP_LOGW(TAG, "Response validation failed: expected '%s'",
               func_cfg->expect_response);
      if (result) {
        result->status = ESP_ERR_INVALID_RESPONSE;
        snprintf(result->response, sizeof(result->response), "%s",
                 response_len > 0 ? (const char *)response_buffer : "TIMEOUT");
        result->response_len = response_len;
      }
      return ESP_ERR_INVALID_RESPONSE;
    }
  } else {
    ESP_LOGD(TAG, "Skipping response read (no response expected, timeout=0)");
  }

  // Step 6: Execute GPIO end sequences via Module_Config_Controller wrapper
  for (uint8_t i = 0; i < func_cfg->gpio_end_count; i++) {
    char pin_str[8];
    snprintf(pin_str, sizeof(pin_str), "%d%d", stack_id, func_cfg->gpio_end[i]);
    bool state = func_cfg->gpio_end_state[i];

    ret = module_gpio_write(stack_id, pin_str, state);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to control GPIO end pin %s: %s", pin_str,
               esp_err_to_name(ret));
      // Don't fail here, GPIO control after command is less critical
    }
  }

  // Step 7: Wait delay_end_ms using FreeRTOS
  if (func_cfg->delay_end_ms > 0) {
    ESP_LOGD(TAG, "Waiting %lu ms after command", func_cfg->delay_end_ms);
    vTaskDelay(pdMS_TO_TICKS(func_cfg->delay_end_ms));
  }

  uint32_t exec_time = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
  if (result) {
    result->status = ESP_OK;
    snprintf(result->response, sizeof(result->response), "%s",
             response_len > 0 ? (const char *)response_buffer : "OK");
    result->response_len = response_len;
    result->execution_time_ms = exec_time;
  }

  ESP_LOGI(TAG, "Function %d executed successfully on stack %d (took %lu ms)",
           func_id, stack_id, exec_time);
  return ESP_OK;
}

/* ===== Public API Implementation ===== */

esp_err_t ble_handler_init(void) {
  if (g_ble_handler.initialized) {
    ESP_LOGW(TAG, "BLE handler already initialized");
    return ESP_OK;
  }

  // Create mutex for protecting g_ble_handler (Fix Issue #2)
  if (!g_ble_handler_mutex) {
    g_ble_handler_mutex = xSemaphoreCreateMutex();
    if (!g_ble_handler_mutex) {
      ESP_LOGE(TAG, "Failed to create BLE handler mutex");
      return ESP_ERR_NO_MEM;
    }
  }

  memset(&g_ble_handler, 0, sizeof(g_ble_handler));

  // Initialize UART communication for both stacks
  // This will be configured based on loaded JSON config
  // For now, use default BLE module settings (9600 baud for JDY-23)
  ESP_LOGI(TAG, "BLE handler initializing...");

  g_ble_handler.initialized = true;
  ESP_LOGI(TAG, "BLE handler initialized successfully");
  return ESP_OK;
}

esp_err_t ble_handler_load_config(uint8_t stack_id, const char *json_config,
                                  uint16_t json_len) {
  if (!g_ble_handler.initialized) {
    ESP_LOGE(TAG, "BLE handler not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  if (!ble_is_valid_stack_id(stack_id) || !json_config || json_len == 0) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Loading BLE config for stack %d (%d bytes)", stack_id,
           json_len);

  // Protect g_ble_handler access with mutex (Fix Issue #2)
  if (xSemaphoreTake(g_ble_handler_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
    ESP_LOGE(TAG, "Failed to acquire BLE handler mutex");
    return ESP_ERR_TIMEOUT;
  }

  // Heap-allocate parsed config to avoid stack overflow (~5.2KB struct)
  // This function may be called from tasks with small stacks (e.g., module_monitor_task: 4KB)
  json_ble_module_config_t *parsed = (json_ble_module_config_t *)calloc(1, sizeof(json_ble_module_config_t));
  if (!parsed) {
    ESP_LOGE(TAG, "Failed to allocate parsed config buffer");
    xSemaphoreGive(g_ble_handler_mutex);
    return ESP_ERR_NO_MEM;
  }

  esp_err_t ret = json_ble_config_parse(json_config, parsed);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to parse BLE JSON config: %s", esp_err_to_name(ret));
    free(parsed);
    xSemaphoreGive(g_ble_handler_mutex);
    return ret;
  }

  if (!g_module_ctrl_initialized) {
    ret = module_config_controller_init();
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to init module config controller");
      free(parsed);
      xSemaphoreGive(g_ble_handler_mutex);
      return ret;
    }
    g_module_ctrl_initialized = true;
  }

  memset(&g_ble_handler.config[stack_id], 0, sizeof(ble_module_config_t));
  g_ble_handler.config[stack_id].module_id = stack_id;
  strncpy(g_ble_handler.config[stack_id].module_type,
          parsed->metadata.module_type,
          sizeof(g_ble_handler.config[stack_id].module_type) - 1);
  strncpy(g_ble_handler.config[stack_id].module_name,
          parsed->metadata.module_name,
          sizeof(g_ble_handler.config[stack_id].module_name) - 1);

  switch (parsed->metadata.communication.port_type) {
  case COMM_PORT_UART:
    strncpy(g_ble_handler.config[stack_id].comm_port_type, "uart",
            sizeof(g_ble_handler.config[stack_id].comm_port_type) - 1);
    g_ble_handler.config[stack_id].baudrate =
        parsed->metadata.communication.params.uart.baudrate;
    ret = module_config_controller_init_uart(
        stack_id, &parsed->metadata.communication.params.uart);
    break;
  case COMM_PORT_SPI:
    strncpy(g_ble_handler.config[stack_id].comm_port_type, "spi",
            sizeof(g_ble_handler.config[stack_id].comm_port_type) - 1);
    ret = module_config_controller_init_spi(
        stack_id, &parsed->metadata.communication.params.spi);
    break;
  case COMM_PORT_I2C:
    strncpy(g_ble_handler.config[stack_id].comm_port_type, "i2c",
            sizeof(g_ble_handler.config[stack_id].comm_port_type) - 1);
    ret = module_config_controller_init_i2c(
        stack_id, &parsed->metadata.communication.params.i2c);
    break;
  case COMM_PORT_USB:
    strncpy(g_ble_handler.config[stack_id].comm_port_type, "usb",
            sizeof(g_ble_handler.config[stack_id].comm_port_type) - 1);
    ret = module_config_controller_init_usb(
        stack_id, &parsed->metadata.communication.params.usb);
    break;
  default:
    ret = ESP_ERR_NOT_SUPPORTED;
    break;
  }

  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init communication for stack %d: %s", stack_id,
             esp_err_to_name(ret));
    free(parsed);
    xSemaphoreGive(g_ble_handler_mutex);
    return ret;
  }

  for (int i = 0; i < BLE_FUNC_COUNT; i++) {
    g_ble_handler.config[stack_id].functions[i].available = false;
  }

  for (int i = 0; i < BLE_MAX_FUNCTIONS; i++) {
    json_ble_function_config_t *src = &parsed->functions[i];
    if (!src->available ||
        (ble_function_id_t)src->function_id >= BLE_FUNC_COUNT) {
      continue;
    }

    ble_function_config_t *dst =
        &g_ble_handler.config[stack_id].functions[src->function_id];
    dst->available = true;
    strncpy(dst->command, src->command, sizeof(dst->command) - 1);
    strncpy(dst->expect_response, src->expect_response,
            sizeof(dst->expect_response) - 1);
    dst->delay_start_ms = src->delay_start_ms;
    dst->delay_end_ms = src->delay_end_ms;
    dst->timeout_ms = src->timeout_ms;

    dst->gpio_start_count = 0;
    for (uint8_t j = 0; j < src->gpio_start_count; j++) {
      uint8_t pin_id = 0;
      if (!ble_parse_pin_id(src->gpio_start[j].pin, &pin_id)) {
        ESP_LOGE(TAG, "Invalid GPIO start pin: %s", src->gpio_start[j].pin);
        free(parsed);
        xSemaphoreGive(g_ble_handler_mutex);
        return ESP_ERR_INVALID_ARG;
      }
      dst->gpio_start[dst->gpio_start_count] = pin_id;
      dst->gpio_start_state[dst->gpio_start_count] = src->gpio_start[j].state;
      dst->gpio_start_count++;
    }

    dst->gpio_end_count = 0;
    for (uint8_t j = 0; j < src->gpio_end_count; j++) {
      uint8_t pin_id = 0;
      if (!ble_parse_pin_id(src->gpio_end[j].pin, &pin_id)) {
        ESP_LOGE(TAG, "Invalid GPIO end pin: %s", src->gpio_end[j].pin);
        free(parsed);
        xSemaphoreGive(g_ble_handler_mutex);
        return ESP_ERR_INVALID_ARG;
      }
      dst->gpio_end[dst->gpio_end_count] = pin_id;
      dst->gpio_end_state[dst->gpio_end_count] = src->gpio_end[j].state;
      dst->gpio_end_count++;
    }
  }

  free(parsed);
  xSemaphoreGive(g_ble_handler_mutex);

  ESP_LOGI(TAG, "BLE config loaded for stack %d", stack_id);
  return ESP_OK;
}

/* ===== Core Functions (0-14) ===== */

esp_err_t ble_handler_hw_reset(uint8_t stack_id) {
  if (!g_ble_handler.initialized) {
    ESP_LOGE(TAG, "BLE handler not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  ble_exec_result_t result = {0};
  esp_err_t ret =
      ble_execute_function_internal(stack_id, BLE_FUNC_HW_RESET, NULL, &result);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Hardware reset executed on stack %d", stack_id);
  } else {
    ESP_LOGE(TAG, "Hardware reset failed on stack %d: %s", stack_id,
             result.response);
  }

  return ret;
}

esp_err_t ble_handler_sw_reset(uint8_t stack_id) {
  if (!g_ble_handler.initialized) {
    ESP_LOGE(TAG, "BLE handler not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  ble_exec_result_t result = {0};
  esp_err_t ret =
      ble_execute_function_internal(stack_id, BLE_FUNC_SW_RESET, NULL, &result);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Software reset executed on stack %d", stack_id);
  } else {
    ESP_LOGE(TAG, "Software reset failed on stack %d: %s", stack_id,
             result.response);
  }

  return ret;
}

esp_err_t ble_handler_factory_reset(uint8_t stack_id) {
  if (!g_ble_handler.initialized) {
    ESP_LOGE(TAG, "BLE handler not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  ble_exec_result_t result = {0};
  esp_err_t ret = ble_execute_function_internal(
      stack_id, BLE_FUNC_FACTORY_RESET, NULL, &result);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Factory reset executed on stack %d", stack_id);
  } else {
    ESP_LOGE(TAG, "Factory reset failed on stack %d: %s", stack_id,
             result.response);
  }

  return ret;
}

esp_err_t ble_handler_get_info(uint8_t stack_id, char *buffer, size_t max_len) {
  if (!g_ble_handler.initialized || !buffer || max_len == 0) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  ble_exec_result_t result = {0};
  esp_err_t ret =
      ble_execute_function_internal(stack_id, BLE_FUNC_GET_INFO, NULL, &result);

  if (ret == ESP_OK) {
    strncpy(buffer, result.response, max_len - 1);
    buffer[max_len - 1] = '\0';
    ESP_LOGI(TAG, "Get info on stack %d: %s", stack_id, buffer);
  } else {
    ESP_LOGE(TAG, "Get info failed on stack %d", stack_id);
  }

  return ret;
}

esp_err_t ble_handler_enter_cmd_mode(uint8_t stack_id) {
  if (!g_ble_handler.initialized) {
    ESP_LOGE(TAG, "BLE handler not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  ble_exec_result_t result = {0};
  esp_err_t ret = ble_execute_function_internal(
      stack_id, BLE_FUNC_ENTER_CMD_MODE, NULL, &result);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Entered CMD mode on stack %d", stack_id);
  } else {
    ESP_LOGE(TAG, "Enter CMD mode failed on stack %d: %s", stack_id,
             result.response);
  }

  return ret;
}



esp_err_t ble_handler_get_connection_status(uint8_t stack_id, char *buffer,
                                            size_t max_len) {
  if (!g_ble_handler.initialized || !buffer || max_len == 0) {
    ESP_LOGE(TAG, "Invalid arguments");
    return ESP_ERR_INVALID_ARG;
  }

  ble_exec_result_t result = {0};
  esp_err_t ret = ble_execute_function_internal(
      stack_id, BLE_FUNC_GET_CONNECTION_STATUS, NULL, &result);

  if (ret == ESP_OK) {
    strncpy(buffer, result.response, max_len - 1);
    buffer[max_len - 1] = '\0';
    ESP_LOGI(TAG, "Connection status on stack %d: %s", stack_id, buffer);
  } else {
    ESP_LOGE(TAG, "Get connection status failed on stack %d", stack_id);
  }

  return ret;
}

esp_err_t ble_handler_enter_sleep(uint8_t stack_id) {
  if (!g_ble_handler.initialized) {
    ESP_LOGE(TAG, "BLE handler not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  ble_exec_result_t result = {0};
  esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_ENTER_SLEEP,
                                                NULL, &result);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Sleep mode entered on stack %d", stack_id);
  } else {
    ESP_LOGE(TAG, "Enter sleep failed on stack %d: %s", stack_id,
             result.response);
  }

  return ret;
}

esp_err_t ble_handler_wakeup(uint8_t stack_id) {
  if (!g_ble_handler.initialized) {
    ESP_LOGE(TAG, "BLE handler not initialized");
    return ESP_ERR_INVALID_STATE;
  }

  ble_exec_result_t result = {0};
  esp_err_t ret =
      ble_execute_function_internal(stack_id, BLE_FUNC_WAKEUP, NULL, &result);

  if (ret == ESP_OK) {
    ESP_LOGI(TAG, "Wakeup on stack %d", stack_id);
  } else {
    ESP_LOGE(TAG, "Wakeup failed on stack %d: %s", stack_id, result.response);
  }

  return ret;
}

/* ===== Optional Functions & Command Matching ===== */

esp_err_t ble_handler_get_function_by_command(uint8_t stack_id,
                                               const char *command,
                                               ble_function_config_t *func_config) {
  if (!g_ble_handler.initialized || !command || !func_config) {
    return ESP_ERR_INVALID_ARG;
  }

  if (!ble_is_valid_stack_id(stack_id)) {
    return ESP_ERR_INVALID_ARG;
  }

  size_t cmd_len = strlen(command);
  
  // Search all functions for prefix or exact match
  for (int func_id = 0; func_id < BLE_FUNC_COUNT; func_id++) {
    ble_function_config_t *cfg = &g_ble_handler.config[stack_id].functions[func_id];
    
    if (!cfg->available) {
      continue;
    }
    
    size_t cfg_cmd_len = strlen(cfg->command);
    
    // For commands starting with "AT+" - try prefix match
    if (strncmp(cfg->command, "AT+", 3) == 0) {
      // Prefix match: command must start with cfg->command
      if (cmd_len >= cfg_cmd_len && 
          strncmp(command, cfg->command, cfg_cmd_len) == 0) {
        memcpy(func_config, cfg, sizeof(ble_function_config_t));
        ESP_LOGI(TAG, "Matched prefix: %s (func_id=%d)", cfg->command, func_id);
        return ESP_OK;
      }
    } else {
      // Exact match for non-AT commands (HW_RESET, etc.)
      if (cmd_len == cfg_cmd_len && strcmp(command, cfg->command) == 0) {
        memcpy(func_config, cfg, sizeof(ble_function_config_t));
        ESP_LOGI(TAG, "Matched exact: %s (func_id=%d)", cfg->command, func_id);
        return ESP_OK;
      }
    }
  }
  
  ESP_LOGW(TAG, "No function match for command: %s", command);
  return ESP_ERR_NOT_FOUND;
}

/**
 * @brief Execute command with pre-matched function config (for task layer)
 * 
 * Flow:
 * 1. Apply GPIO start sequences from func_config
 * 2. Wait delay_start_ms from func_config
 * 3. Send command (full string from server: e.g., "AT+SCAN=5000")
 * 4. Wait for response with timeout from func_config
 * 5. Apply GPIO end sequences from func_config
 * 6. Wait delay_end_ms from func_config
 * 
 * @param stack_id Stack ID (0 or 1)
 * @param command Raw command string (e.g., "AT+SCAN=5000")
 * @param func_config Function config already matched from JSON
 * @param result Output execution result
 * @return ESP_OK on success
 */
esp_err_t ble_handler_execute_command_with_config(uint8_t stack_id,
                                                   const char *command,
                                                   const ble_function_config_t *func_config,
                                                   ble_exec_result_t *result) {
  if (!g_ble_handler.initialized || !command || !func_config) {
    if (result) result->status = ESP_ERR_INVALID_ARG;
    return ESP_ERR_INVALID_ARG;
  }

  if (!ble_is_valid_stack_id(stack_id)) {
    if (result) result->status = ESP_ERR_INVALID_ARG;
    return ESP_ERR_INVALID_ARG;
  }

  ESP_LOGI(TAG, "Executing command '%s' on stack %d with JSON config", command, stack_id);

  esp_err_t ret = ESP_OK;
  TickType_t start_tick = xTaskGetTickCount();

  // Step 1: Execute GPIO start sequences (from JSON config)
  for (uint8_t i = 0; i < func_config->gpio_start_count; i++) {
    char pin_str[8];
    snprintf(pin_str, sizeof(pin_str), "%d%d", stack_id, func_config->gpio_start[i]);
    bool state = func_config->gpio_start_state[i];

    ret = module_gpio_write(stack_id, pin_str, state);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to control GPIO start pin %s: %s", pin_str, esp_err_to_name(ret));
      if (result) result->status = ret;
      return ret;
    }
    ESP_LOGD(TAG, "GPIO pin %s set to %d", pin_str, state);
  }

  // Step 2: Wait delay_start_ms (from JSON config)
  if (func_config->delay_start_ms > 0) {
    ESP_LOGD(TAG, "Waiting %lu ms before command", func_config->delay_start_ms);
    vTaskDelay(pdMS_TO_TICKS(func_config->delay_start_ms));
  }

  // Step 3: Validate and send command
  size_t cmd_len = strlen(command);
  size_t expect_len = strlen(func_config->expect_response);
  bool is_gpio_only = (cmd_len == 0 && expect_len == 0);

  if (is_gpio_only) {
    // GPIO-only function: skip command sending
    ESP_LOGI(TAG, "GPIO-only function - no command/response expected");
  } else {
    // Validate command string
    if (!ble_validate_command_string(command, BLE_CMD_MAX_LEN)) {
      ESP_LOGE(TAG, "Command validation failed");
      if (result) result->status = ESP_ERR_INVALID_ARG;
      return ESP_ERR_INVALID_ARG;
    }

    // Send command via Module_Config_Controller
    ESP_LOGD(TAG, "Sending command: %s", command);
    comm_port_type_t port_type = ble_get_comm_port(stack_id);
    if (port_type == COMM_PORT_MAX) {
      ESP_LOGE(TAG, "Invalid comm port type for stack %d", stack_id);
      if (result) result->status = ESP_ERR_INVALID_STATE;
      return ESP_ERR_INVALID_STATE;
    }

    ret = module_bus_write(stack_id, port_type, (const uint8_t *)command, cmd_len);
    if (ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to send command: %s", esp_err_to_name(ret));
      if (result) result->status = ret;
      return ret;
    }

    // Step 4: Wait for response (from JSON config timeout)
    uint8_t response_buffer[BLE_RESPONSE_MAX_LEN] = {0};
    size_t response_len = 0;
    bool skip_read = (expect_len == 0 && func_config->timeout_ms == 0);

    if (!skip_read) {
      ret = module_bus_read(stack_id, port_type, response_buffer,
                            sizeof(response_buffer) - 1, func_config->timeout_ms,
                            &response_len);
      if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "Failed to receive response: %s", esp_err_to_name(ret));
        if (result) result->status = ret;
        return ret;
      }

      // Verify response matches expect_response (from JSON)
      bool response_valid = false;
      if (response_len > 0) {
        response_buffer[response_len] = '\0';
        ESP_LOGD(TAG, "Received response: %s", (char *)response_buffer);

        if (expect_len == 0 ||
            strstr((const char *)response_buffer, func_config->expect_response) != NULL) {
          response_valid = true;
        }
      }

      if (!response_valid && expect_len > 0) {
        ESP_LOGW(TAG, "Response validation failed: expected '%s'",
                 func_config->expect_response);
        if (result) {
          result->status = ESP_ERR_INVALID_RESPONSE;
          snprintf(result->response, sizeof(result->response), "%s",
                   response_len > 0 ? (const char *)response_buffer : "TIMEOUT");
          result->response_len = response_len;
        }
        return ESP_ERR_INVALID_RESPONSE;
      }

      // Copy response to result
      if (result && response_len > 0) {
        snprintf(result->response, sizeof(result->response), "%s",
                 (const char *)response_buffer);
        result->response_len = response_len;
      }
    } else {
      ESP_LOGD(TAG, "Skipping response read (no response expected, timeout=0)");
    }
  }

  // Step 5: Execute GPIO end sequences (from JSON config)
  for (uint8_t i = 0; i < func_config->gpio_end_count; i++) {
    char pin_str[8];
    snprintf(pin_str, sizeof(pin_str), "%d%d", stack_id, func_config->gpio_end[i]);
    bool state = func_config->gpio_end_state[i];

    ret = module_gpio_write(stack_id, pin_str, state);
    if (ret != ESP_OK) {
      ESP_LOGW(TAG, "Failed to control GPIO end pin %s: %s", pin_str,
               esp_err_to_name(ret));
    }
  }

  // Step 6: Wait delay_end_ms (from JSON config)
  if (func_config->delay_end_ms > 0) {
    ESP_LOGD(TAG, "Waiting %lu ms after command", func_config->delay_end_ms);
    vTaskDelay(pdMS_TO_TICKS(func_config->delay_end_ms));
  }

  uint32_t exec_time = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
  if (result) {
    result->status = ESP_OK;
    if (result->response[0] == '\0') {
      snprintf(result->response, sizeof(result->response), "OK");
    }
    result->execution_time_ms = exec_time;
  }

  ESP_LOGI(TAG, "Command executed successfully on stack %d (took %lu ms)",
           stack_id, exec_time);
  return ESP_OK;
}

esp_err_t ble_handler_send_binary_command(uint8_t stack_id,
                                          const uint8_t *cmd_bytes,
                                          uint16_t cmd_len, uint8_t *response,
                                          uint16_t resp_len,
                                          uint16_t timeout_ms) {
  if (!ble_is_valid_stack_id(stack_id) || !cmd_bytes || cmd_len == 0) {
    return ESP_ERR_INVALID_ARG;
  }

  comm_port_type_t port_type = ble_get_comm_port(stack_id);
  if (port_type == COMM_PORT_MAX) {
    ESP_LOGE(TAG, "Invalid comm port for stack %d", stack_id);
    return ESP_ERR_INVALID_STATE;
  }

  ESP_LOGD(TAG, "Sending binary command (%d bytes): 0x%02X 0x%02X ...", cmd_len,
           cmd_bytes[0], cmd_len > 1 ? cmd_bytes[1] : 0);

  // Send binary command
  esp_err_t ret = module_bus_write(stack_id, port_type, cmd_bytes, cmd_len);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to send binary command: %s", esp_err_to_name(ret));
    return ret;
  }

  // Read response if requested
  if (response && resp_len > 0) {
    size_t received_len = 0;
    ret = module_bus_read(stack_id, port_type, response, resp_len, timeout_ms,
                          &received_len);
    if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
      ESP_LOGE(TAG, "Failed to read binary response: %s", esp_err_to_name(ret));
      return ret;
    }

    ESP_LOGD(TAG, "Binary response received: %zu bytes", received_len);
  }

  return ESP_OK;
}
