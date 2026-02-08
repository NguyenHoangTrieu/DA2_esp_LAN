/**
 * @file ble_handler.c
 * @brief BLE Handler Middleware Implementation
 */

#include "ble_handler.h"
#include "json_ble_config_parser.h"
#include "module_config_controller.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "BLE_HANDLER";

/* ===== Configuration Constants ===== */

#define BLE_MAX_DEVICES_PER_STACK   16      // Max tracked devices per stack
#define BLE_CMD_MAX_RETRIES         3       // Auto-recovery retry count
#define BLE_BINARY_CMD_MARKER       0xC0    // Binary protocol marker
#define BLE_CMD_MAX_LEN             128     // Max command string length
#define BLE_RESPONSE_MAX_LEN        256     // Max response buffer size
#define BLE_HEX_DATA_MAX_LEN        512     // Max hex data buffer size
#define BLE_MAX_STACKS              2       // Number of stacks (0 and 1)

/* ===== Static Data ===== */

static struct {
    bool initialized;
    ble_module_config_t config[BLE_MAX_STACKS];  // Stack 0 and Stack 1
    ble_device_t devices[BLE_MAX_STACKS][BLE_MAX_DEVICES_PER_STACK];  // Device tracking
    uint8_t device_count[BLE_MAX_STACKS];        // Device count per stack
} g_ble_handler = {0};

static bool g_module_ctrl_initialized = false;

// Mutex to protect g_ble_handler from multi-stack race conditions (Fix Issue #2)
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
static ble_function_config_t* ble_get_function_config(uint8_t stack_id,
                                                        ble_function_id_t func_id) {
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
        ESP_LOGD(TAG, "Binary format command detected (0x%02X prefix)", BLE_BINARY_CMD_MARKER);
        return true;  // Binary commands always valid if length OK
    }

    // ASCII validation for AT commands
    if (strncmp(cmd, "AT", 2) == 0) {
        // Check for printable ASCII characters
        for (size_t i = 0; i < cmd_len; i++) {
            if (cmd[i] < 0x20 || cmd[i] > 0x7E) {
                if (cmd[i] != '\r' && cmd[i] != '\n') {
                    ESP_LOGW(TAG, "Non-printable character in AT command at index %zu: 0x%02X", 
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
                ESP_LOGW(TAG, "Suspicious character in command at index %zu: 0x%02X", 
                        i, (uint8_t)cmd[i]);
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
        if (result) result->status = ESP_ERR_INVALID_ARG;
        return ESP_ERR_INVALID_ARG;
    }

    ble_function_config_t *func_cfg = ble_get_function_config(stack_id, func_id);
    if (!func_cfg || !func_cfg->available) {
        ESP_LOGW(TAG, "Function %d not configured for stack %d", func_id, stack_id);
        if (result) result->status = ESP_ERR_NOT_SUPPORTED;
        return ESP_ERR_NOT_SUPPORTED;
    }

    ESP_LOGD(TAG, "Executing BLE function %d on stack %d", func_id, stack_id);

    esp_err_t ret = ESP_OK;
    TickType_t start_tick = xTaskGetTickCount();

    // Step 1: Execute GPIO start sequences via Module_Config_Controller wrapper
    for (uint8_t i = 0; i < func_cfg->gpio_start_count; i++) {
        char pin_str[3];  // Format: "0X" where 0 is stack, X is pin
        snprintf(pin_str, sizeof(pin_str), "%d%d", stack_id, func_cfg->gpio_start[i]);
        bool state = func_cfg->gpio_start_state[i];
        
        ret = module_gpio_write(stack_id, pin_str, state);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to control GPIO pin %s: %s", pin_str, esp_err_to_name(ret));
            if (result) result->status = ret;
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
                    if (result) result->status = ESP_ERR_INVALID_SIZE;
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

    // Step 2.5: Validate command string (APPROVED ENHANCEMENT)
    if (!ble_validate_command_string(final_command, sizeof(final_command))) {
        ESP_LOGE(TAG, "Command validation failed for function %d", func_id);
        if (result) result->status = ESP_ERR_INVALID_ARG;
        return ESP_ERR_INVALID_ARG;
    }

    // Step 3: Send AT command via Module_Config_Controller wrapper (module_bus_write)
    ESP_LOGD(TAG, "Sending command: %s", final_command);
    comm_port_type_t port_type = ble_get_comm_port(stack_id);
    if (port_type == COMM_PORT_MAX) {
        ESP_LOGE(TAG, "Invalid comm port type for stack %d", stack_id);
        if (result) result->status = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }

    ret = module_bus_write(stack_id, port_type,
                          (const uint8_t *)final_command,
                          strlen(final_command));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send command: %s", esp_err_to_name(ret));
        if (result) result->status = ret;
        return ret;
    }

    // Step 4: Wait for response with timeout via Module_Config_Controller wrapper (module_bus_read)
    uint8_t response_buffer[BLE_RESPONSE_MAX_LEN] = {0};
    size_t response_len = 0;
    ret = module_bus_read(stack_id, port_type,
                         response_buffer,
                         sizeof(response_buffer) - 1,
                         &response_len,
                         func_cfg->timeout_ms);
    if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "Failed to receive response: %s", esp_err_to_name(ret));
        if (result) result->status = ret;
        return ret;
    }

    // Step 5: Verify response matches expect_response
    bool response_valid = false;
    if (response_len > 0) {
        response_buffer[response_len] = '\0';
        ESP_LOGD(TAG, "Received response: %s", (char *)response_buffer);
        
        // Check if response contains expected string
        if (strlen(func_cfg->expect_response) == 0 || 
            strstr((const char *)response_buffer, func_cfg->expect_response) != NULL) {
            response_valid = true;
        }
    }

    if (!response_valid && strlen(func_cfg->expect_response) > 0) {
        ESP_LOGW(TAG, "Response validation failed: expected '%s'", func_cfg->expect_response);
        if (result) {
            result->status = ESP_ERR_INVALID_RESPONSE;
            snprintf(result->response, sizeof(result->response), "%s", 
                    response_len > 0 ? (const char *)response_buffer : "TIMEOUT");
            result->response_len = response_len;
        }
        return ESP_ERR_INVALID_RESPONSE;
    }

    // Step 6: Execute GPIO end sequences via Module_Config_Controller wrapper
    for (uint8_t i = 0; i < func_cfg->gpio_end_count; i++) {
        char pin_str[3];
        snprintf(pin_str, sizeof(pin_str), "%d%d", stack_id, func_cfg->gpio_end[i]);
        bool state = func_cfg->gpio_end_state[i];
        
        ret = module_gpio_write(stack_id, pin_str, state);
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to control GPIO end pin %s: %s", pin_str, esp_err_to_name(ret));
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

esp_err_t ble_handler_load_config(uint8_t stack_id,
                                   const char *json_config,
                                   uint16_t json_len) {
    if (!g_ble_handler.initialized) {
        ESP_LOGE(TAG, "BLE handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (!ble_is_valid_stack_id(stack_id) || !json_config || json_len == 0) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "Loading BLE config for stack %d (%d bytes)", stack_id, json_len);

    // Protect g_ble_handler access with mutex (Fix Issue #2)
    if (xSemaphoreTake(g_ble_handler_mutex, pdMS_TO_TICKS(1000)) != pdTRUE) {
        ESP_LOGE(TAG, "Failed to acquire BLE handler mutex");
        return ESP_ERR_TIMEOUT;
    }

    json_ble_module_config_t parsed = {0};
    esp_err_t ret = json_ble_config_parse(json_config, &parsed);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse BLE JSON config: %s", esp_err_to_name(ret));
        xSemaphoreGive(g_ble_handler_mutex);
        return ret;
    }

    if (!g_module_ctrl_initialized) {
        ret = module_config_controller_init();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to init module config controller");
            xSemaphoreGive(g_ble_handler_mutex);
            return ret;
        }
        g_module_ctrl_initialized = true;
    }

    memset(&g_ble_handler.config[stack_id], 0, sizeof(ble_module_config_t));
    g_ble_handler.config[stack_id].module_id = stack_id;
    strncpy(g_ble_handler.config[stack_id].module_type,
            parsed.metadata.module_type,
            sizeof(g_ble_handler.config[stack_id].module_type) - 1);
    strncpy(g_ble_handler.config[stack_id].module_name,
            parsed.metadata.module_name,
            sizeof(g_ble_handler.config[stack_id].module_name) - 1);

    switch (parsed.metadata.communication.port_type) {
    case COMM_PORT_UART:
        strncpy(g_ble_handler.config[stack_id].comm_port_type, "uart",
                sizeof(g_ble_handler.config[stack_id].comm_port_type) - 1);
        g_ble_handler.config[stack_id].baudrate =
            parsed.metadata.communication.params.uart.baudrate;
        ret = module_config_controller_init_uart(
            stack_id, &parsed.metadata.communication.params.uart);
        break;
    case COMM_PORT_SPI:
        strncpy(g_ble_handler.config[stack_id].comm_port_type, "spi",
                sizeof(g_ble_handler.config[stack_id].comm_port_type) - 1);
        ret = module_config_controller_init_spi(
            stack_id, &parsed.metadata.communication.params.spi);
        break;
    case COMM_PORT_I2C:
        strncpy(g_ble_handler.config[stack_id].comm_port_type, "i2c",
                sizeof(g_ble_handler.config[stack_id].comm_port_type) - 1);
        ret = module_config_controller_init_i2c(
            stack_id, &parsed.metadata.communication.params.i2c);
        break;
    case COMM_PORT_USB:
        strncpy(g_ble_handler.config[stack_id].comm_port_type, "usb",
                sizeof(g_ble_handler.config[stack_id].comm_port_type) - 1);
        ret = module_config_controller_init_usb(
            stack_id, &parsed.metadata.communication.params.usb);
        break;
    default:
        ret = ESP_ERR_NOT_SUPPORTED;
        break;
    }

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init communication for stack %d: %s",
                 stack_id, esp_err_to_name(ret));
        xSemaphoreGive(g_ble_handler_mutex);
        return ret;
    }

    for (int i = 0; i < BLE_FUNC_COUNT; i++) {
        g_ble_handler.config[stack_id].functions[i].available = false;
    }

    for (int i = 0; i < BLE_MAX_FUNCTIONS; i++) {
        json_ble_function_config_t *src = &parsed.functions[i];
        if (!src->available || src->function_id >= BLE_FUNC_COUNT) {
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
                ESP_LOGE(TAG, "Invalid GPIO start pin: %s",
                         src->gpio_start[j].pin);
                xSemaphoreGive(g_ble_handler_mutex);
                return ESP_ERR_INVALID_ARG;
            }
            dst->gpio_start[dst->gpio_start_count] = pin_id;
            dst->gpio_start_state[dst->gpio_start_count] =
                src->gpio_start[j].state;
            dst->gpio_start_count++;
        }

        dst->gpio_end_count = 0;
        for (uint8_t j = 0; j < src->gpio_end_count; j++) {
            uint8_t pin_id = 0;
            if (!ble_parse_pin_id(src->gpio_end[j].pin, &pin_id)) {
                ESP_LOGE(TAG, "Invalid GPIO end pin: %s",
                         src->gpio_end[j].pin);
                xSemaphoreGive(g_ble_handler_mutex);
                return ESP_ERR_INVALID_ARG;
            }
            dst->gpio_end[dst->gpio_end_count] = pin_id;
            dst->gpio_end_state[dst->gpio_end_count] = src->gpio_end[j].state;
            dst->gpio_end_count++;
        }
    }

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
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_HW_RESET, NULL, &result);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Hardware reset executed on stack %d", stack_id);
    } else {
        ESP_LOGE(TAG, "Hardware reset failed on stack %d: %s", stack_id, result.response);
    }

    return ret;
}

esp_err_t ble_handler_sw_reset(uint8_t stack_id) {
    if (!g_ble_handler.initialized) {
        ESP_LOGE(TAG, "BLE handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_SW_RESET, NULL, &result);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Software reset executed on stack %d", stack_id);
    } else {
        ESP_LOGE(TAG, "Software reset failed on stack %d: %s", stack_id, result.response);
    }

    return ret;
}

esp_err_t ble_handler_factory_reset(uint8_t stack_id) {
    if (!g_ble_handler.initialized) {
        ESP_LOGE(TAG, "BLE handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_FACTORY_RESET, NULL, &result);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Factory reset executed on stack %d", stack_id);
    } else {
        ESP_LOGE(TAG, "Factory reset failed on stack %d: %s", stack_id, result.response);
    }

    return ret;
}

esp_err_t ble_handler_get_info(uint8_t stack_id,
                                char *buffer,
                                size_t max_len) {
    if (!g_ble_handler.initialized || !buffer || max_len == 0) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_GET_INFO, NULL, &result);

    if (ret == ESP_OK) {
        strncpy(buffer, result.response, max_len - 1);
        buffer[max_len - 1] = '\0';
        ESP_LOGI(TAG, "Get info on stack %d: %s", stack_id, buffer);
    } else {
        ESP_LOGE(TAG, "Get info failed on stack %d", stack_id);
    }

    return ret;
}

esp_err_t ble_handler_set_name(uint8_t stack_id,
                                const char *name) {
    if (!g_ble_handler.initialized || !name || strlen(name) > 31) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_SET_NAME, name, &result);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Device name set to '%s' on stack %d", name, stack_id);
    } else {
        ESP_LOGE(TAG, "Set name failed on stack %d: %s", stack_id, result.response);
    }

    return ret;
}

esp_err_t ble_handler_set_comm_config(uint8_t stack_id,
                                       const char *config_param) {
    if (!g_ble_handler.initialized || !config_param) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_SET_COMM_CONFIG,
                                                   config_param, &result);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Comm config updated on stack %d", stack_id);
    } else {
        ESP_LOGE(TAG, "Set comm config failed on stack %d: %s", stack_id, result.response);
    }

    return ret;
}

esp_err_t ble_handler_set_rf_params(uint8_t stack_id,
                                     const char *rf_param) {
    if (!g_ble_handler.initialized || !rf_param) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_SET_RF_PARAMS,
                                                   rf_param, &result);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "RF params set on stack %d", stack_id);
    } else {
        ESP_LOGE(TAG, "Set RF params failed on stack %d: %s", stack_id, result.response);
    }

    return ret;
}

esp_err_t ble_handler_enter_cmd_mode(uint8_t stack_id) {
    if (!g_ble_handler.initialized) {
        ESP_LOGE(TAG, "BLE handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_ENTER_CMD_MODE, NULL, &result);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Entered CMD mode on stack %d", stack_id);
    } else {
        ESP_LOGE(TAG, "Enter CMD mode failed on stack %d: %s", stack_id, result.response);
    }

    return ret;
}

esp_err_t ble_handler_enter_data_mode(uint8_t stack_id) {
    if (!g_ble_handler.initialized) {
        ESP_LOGE(TAG, "BLE handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_ENTER_DATA_MODE, NULL, &result);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Entered DATA mode on stack %d", stack_id);
    } else {
        ESP_LOGE(TAG, "Enter DATA mode failed on stack %d: %s", stack_id, result.response);
    }

    return ret;
}

esp_err_t ble_handler_start_broadcast(uint8_t stack_id) {
    if (!g_ble_handler.initialized) {
        ESP_LOGE(TAG, "BLE handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_START_BROADCAST, NULL, &result);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Broadcasting started on stack %d", stack_id);
    } else {
        ESP_LOGE(TAG, "Start broadcast failed on stack %d: %s", stack_id, result.response);
    }

    return ret;
}

esp_err_t ble_handler_connect(uint8_t stack_id,
                               const char *address) {
    if (!g_ble_handler.initialized || !address || strlen(address) < 11) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_CONNECT, address, &result);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Connected to %s on stack %d", address, stack_id);
    } else {
        ESP_LOGE(TAG, "Connect failed on stack %d: %s", stack_id, result.response);
    }

    return ret;
}

esp_err_t ble_handler_disconnect(uint8_t stack_id) {
    if (!g_ble_handler.initialized) {
        ESP_LOGE(TAG, "BLE handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_DISCONNECT, NULL, &result);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Disconnected on stack %d", stack_id);
    } else {
        ESP_LOGE(TAG, "Disconnect failed on stack %d: %s", stack_id, result.response);
    }

    return ret;
}

esp_err_t ble_handler_get_connection_status(uint8_t stack_id,
                                             char *buffer,
                                             size_t max_len) {
    if (!g_ble_handler.initialized || !buffer || max_len == 0) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_GET_CONNECTION_STATUS,
                                                   NULL, &result);

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
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_ENTER_SLEEP, NULL, &result);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Sleep mode entered on stack %d", stack_id);
    } else {
        ESP_LOGE(TAG, "Enter sleep failed on stack %d: %s", stack_id, result.response);
    }

    return ret;
}

esp_err_t ble_handler_wakeup(uint8_t stack_id) {
    if (!g_ble_handler.initialized) {
        ESP_LOGE(TAG, "BLE handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_WAKEUP, NULL, &result);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Wakeup on stack %d", stack_id);
    } else {
        ESP_LOGE(TAG, "Wakeup failed on stack %d: %s", stack_id, result.response);
    }

    return ret;
}

/* ===== Optional Functions (15-19) ===== */

esp_err_t ble_handler_start_discovery(uint8_t stack_id) {
    if (!g_ble_handler.initialized) {
        ESP_LOGE(TAG, "BLE handler not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_START_DISCOVERY, NULL, &result);

    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Discovery not configured for stack %d", stack_id);
    } else if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Discovery started on stack %d", stack_id);
    }

    return ret;
}

esp_err_t ble_handler_send_data(uint8_t stack_id,
                                 const uint8_t *data,
                                 uint16_t len) {
    if (!g_ble_handler.initialized || !data || len == 0) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    // Convert binary data to hex string for AT command
    // Use heap allocation to avoid stack overflow (512 bytes is too large for stack)
    uint16_t max_data_len = (len < 256) ? len : 256;
    char *hex_data = (char *)malloc(max_data_len * 2 + 1);
    if (!hex_data) {
        ESP_LOGE(TAG, "Failed to allocate hex_data buffer");
        return ESP_ERR_NO_MEM;
    }
    
    for (uint16_t i = 0; i < max_data_len; i++) {
        snprintf(&hex_data[i * 2], 3, "%02X", data[i]);
    }
    hex_data[max_data_len * 2] = '\0';

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_SEND_DATA, hex_data, &result);
    
    free(hex_data);

    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Send data not configured for stack %d", stack_id);
    } else if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Data sent on stack %d (%d bytes)", stack_id, len);
    }

    return ret;
}

esp_err_t ble_handler_get_diagnostics(uint8_t stack_id,
                                       char *buffer,
                                       size_t max_len) {
    if (!g_ble_handler.initialized || !buffer || max_len == 0) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_GET_DIAGNOSTICS, NULL, &result);

    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Diagnostics not configured for stack %d", stack_id);
    } else if (ret == ESP_OK) {
        strncpy(buffer, result.response, max_len - 1);
        buffer[max_len - 1] = '\0';
        ESP_LOGI(TAG, "Diagnostics on stack %d: %s", stack_id, buffer);
    }

    return ret;
}

esp_err_t ble_handler_set_security(uint8_t stack_id,
                                    const char *security_param) {
    if (!g_ble_handler.initialized || !security_param) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_SET_SECURITY,
                                                   security_param, &result);

    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Security not configured for stack %d", stack_id);
    } else if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Security configured on stack %d", stack_id);
    }

    return ret;
}

esp_err_t ble_handler_manage_whitelist(uint8_t stack_id,
                                        const char *mac_address,
                                        bool add) {
    if (!g_ble_handler.initialized || !mac_address || strlen(mac_address) < 11) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    char param[64];
    snprintf(param, sizeof(param), "%s,%d", mac_address, add ? 1 : 0);

    ble_exec_result_t result = {0};
    esp_err_t ret = ble_execute_function_internal(stack_id, BLE_FUNC_MANAGE_WHITELIST,
                                                   param, &result);

    if (ret == ESP_ERR_NOT_SUPPORTED) {
        ESP_LOGW(TAG, "Whitelist management not configured for stack %d", stack_id);
    } else if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Whitelist updated on stack %d", stack_id);
    }

    return ret;
}

/* ===== Internal Helper for Task Layer ===== */

esp_err_t ble_handler_execute_function(uint8_t stack_id,
                                        ble_function_id_t func_id,
                                        const char *param,
                                        ble_exec_result_t *result) {
    if (!g_ble_handler.initialized) {
        if (result) result->status = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }

    return ble_execute_function_internal(stack_id, func_id, param, result);
}

/* ===== Device Management Implementation (NEW - Task 1.1) ===== */

esp_err_t ble_handler_add_device(uint8_t stack_id,
                                  const uint8_t *mac_address,
                                  const char *device_name) {
    if (!ble_is_valid_stack_id(stack_id) || !mac_address) {
        return ESP_ERR_INVALID_ARG;
    }

    // Check if device already exists
    for (uint8_t i = 0; i < g_ble_handler.device_count[stack_id]; i++) {
        if (memcmp(g_ble_handler.devices[stack_id][i].mac_address, 
                  mac_address, 6) == 0) {
            ESP_LOGD(TAG, "Device already tracked on stack %d", stack_id);
            // Update name if provided
            if (device_name) {
                strncpy(g_ble_handler.devices[stack_id][i].device_name,
                       device_name, sizeof(g_ble_handler.devices[stack_id][i].device_name) - 1);
            }
            return ESP_OK;
        }
    }

    // Check capacity
    if (g_ble_handler.device_count[stack_id] >= BLE_MAX_DEVICES_PER_STACK) {
        ESP_LOGW(TAG, "Device list full for stack %d", stack_id);
        return ESP_ERR_NO_MEM;
    }

    // Add new device
    uint8_t idx = g_ble_handler.device_count[stack_id];
    memcpy(g_ble_handler.devices[stack_id][idx].mac_address, mac_address, 6);
    if (device_name) {
        strncpy(g_ble_handler.devices[stack_id][idx].device_name,
               device_name, sizeof(g_ble_handler.devices[stack_id][idx].device_name) - 1);
    } else {
        g_ble_handler.devices[stack_id][idx].device_name[0] = '\0';
    }
    g_ble_handler.devices[stack_id][idx].connected = true;
    g_ble_handler.devices[stack_id][idx].rssi = 0;
    g_ble_handler.devices[stack_id][idx].last_activity_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    g_ble_handler.device_count[stack_id]++;
    
    ESP_LOGI(TAG, "Device added to stack %d (total: %d)", 
            stack_id, g_ble_handler.device_count[stack_id]);
    return ESP_OK;
}

esp_err_t ble_handler_remove_device(uint8_t stack_id,
                                     const uint8_t *mac_address) {
    if (!ble_is_valid_stack_id(stack_id) || !mac_address) {
        return ESP_ERR_INVALID_ARG;
    }

    // Find device
    for (uint8_t i = 0; i < g_ble_handler.device_count[stack_id]; i++) {
        if (memcmp(g_ble_handler.devices[stack_id][i].mac_address, 
                  mac_address, 6) == 0) {
            // Shift remaining devices down
            for (uint8_t j = i; j < g_ble_handler.device_count[stack_id] - 1; j++) {
                memcpy(&g_ble_handler.devices[stack_id][j],
                      &g_ble_handler.devices[stack_id][j + 1],
                      sizeof(ble_device_t));
            }
            g_ble_handler.device_count[stack_id]--;
            
            ESP_LOGI(TAG, "Device removed from stack %d (remaining: %d)", 
                    stack_id, g_ble_handler.device_count[stack_id]);
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

uint8_t ble_handler_get_device_count(uint8_t stack_id) {
    if (!ble_is_valid_stack_id(stack_id)) {
        return 0;
    }
    return g_ble_handler.device_count[stack_id];
}

esp_err_t ble_handler_get_device(uint8_t stack_id,
                                  const uint8_t *mac_address,
                                  ble_device_t *device_out) {
    if (!ble_is_valid_stack_id(stack_id) || !mac_address || !device_out) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint8_t i = 0; i < g_ble_handler.device_count[stack_id]; i++) {
        if (memcmp(g_ble_handler.devices[stack_id][i].mac_address, 
                  mac_address, 6) == 0) {
            memcpy(device_out, &g_ble_handler.devices[stack_id][i], sizeof(ble_device_t));
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t ble_handler_update_device_activity(uint8_t stack_id,
                                              const uint8_t *mac_address) {
    if (!ble_is_valid_stack_id(stack_id) || !mac_address) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint8_t i = 0; i < g_ble_handler.device_count[stack_id]; i++) {
        if (memcmp(g_ble_handler.devices[stack_id][i].mac_address, 
                  mac_address, 6) == 0) {
            g_ble_handler.devices[stack_id][i].last_activity_ms = 
                xTaskGetTickCount() * portTICK_PERIOD_MS;
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

/* ===== Enhanced Features Implementation (NEW - Task 1.1) ===== */

esp_err_t ble_handler_execute_with_recovery(uint8_t stack_id,
                                             ble_function_id_t func_id,
                                             const char *param,
                                             ble_exec_result_t *result) {
    if (!g_ble_handler.initialized) {
        if (result) result->status = ESP_ERR_INVALID_STATE;
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = ESP_FAIL;

    // Try executing function with retries
    for (int retry = 0; retry < BLE_CMD_MAX_RETRIES; retry++) {
        ret = ble_execute_function_internal(stack_id, func_id, param, result);
        
        if (ret == ESP_OK) {
            if (retry > 0) {
                ESP_LOGI(TAG, "Function %d succeeded after %d retries", func_id, retry);
            }
            return ESP_OK;
        }

        if (ret == ESP_ERR_TIMEOUT || ret == ESP_ERR_INVALID_RESPONSE) {
            ESP_LOGW(TAG, "Function %d failed (attempt %d/%d): %s", 
                    func_id, retry + 1, BLE_CMD_MAX_RETRIES, esp_err_to_name(ret));
            vTaskDelay(pdMS_TO_TICKS(100));  // Wait 100ms between retries
        } else {
            // Non-recoverable error
            return ret;
        }
    }

    // All retries failed - try SW reset recovery
    ESP_LOGW(TAG, "All retries failed for function %d, attempting SW reset", func_id);
    ret = ble_handler_sw_reset(stack_id);
    if (ret != ESP_OK) {
        // SW reset failed - try HW reset as last resort
        ESP_LOGE(TAG, "SW reset failed, attempting HW reset");
        ret = ble_handler_hw_reset(stack_id);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "HW reset failed - module may be unrecoverable");
            if (result) result->status = ESP_FAIL;
            return ESP_FAIL;
        }
    }

    // Wait for module to stabilize after reset
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Retry original function after reset
    ret = ble_execute_function_internal(stack_id, func_id, param, result);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Function %d succeeded after reset recovery", func_id);
    } else {
        ESP_LOGE(TAG, "Function %d failed even after reset recovery", func_id);
    }

    return ret;
}

esp_err_t ble_handler_parse_frame(const uint8_t *data,
                                   uint16_t len,
                                   uint8_t *mac_out,
                                   uint8_t *payload_out,
                                   uint16_t *payload_len_out) {
    if (!data || len < 8 || !mac_out || !payload_out || !payload_len_out) {
        return ESP_ERR_INVALID_ARG;
    }

    // Simple frame format: [MAC:6][PAYLOAD:N]
    // For more complex protocols, extend this logic
    
    memcpy(mac_out, data, 6);
    
    uint16_t payload_len = len - 6;
    memcpy(payload_out, data + 6, payload_len);
    *payload_len_out = payload_len;

    ESP_LOGD(TAG, "Frame parsed: MAC=%02X:%02X:%02X:%02X:%02X:%02X, payload_len=%d",
            mac_out[0], mac_out[1], mac_out[2], mac_out[3], mac_out[4], mac_out[5],
            payload_len);

    return ESP_OK;
}

esp_err_t ble_handler_send_binary_command(uint8_t stack_id,
                                           const uint8_t *cmd_bytes,
                                           uint16_t cmd_len,
                                           uint8_t *response,
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

    ESP_LOGD(TAG, "Sending binary command (%d bytes): 0x%02X 0x%02X ...", 
            cmd_len, cmd_bytes[0], cmd_len > 1 ? cmd_bytes[1] : 0);

    // Send binary command
    esp_err_t ret = module_bus_write(stack_id, port_type, cmd_bytes, cmd_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send binary command: %s", esp_err_to_name(ret));
        return ret;
    }

    // Read response if requested
    if (response && resp_len > 0) {
        size_t received_len = 0;
        ret = module_bus_read(stack_id, port_type, response, resp_len, 
                             &received_len, timeout_ms);
        if (ret != ESP_OK && ret != ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "Failed to read binary response: %s", esp_err_to_name(ret));
            return ret;
        }

        ESP_LOGD(TAG, "Binary response received: %zu bytes", received_len);
    }

    return ESP_OK;
}
