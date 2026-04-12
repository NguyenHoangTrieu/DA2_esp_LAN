/**
 * @file json_lora_config_parser.c
 * @brief LoRa-specific JSON configuration parser implementation
 *
 * Mirror of json_ble_config_parser.c with LoRaWAN-specific function names
 * and module_type check.  The parsing logic (cJSON iteration, GPIO array,
 * field extraction) is identical to the BLE counterpart.
 */

#include "json_lora_config_parser.h"
#include "cJSON.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "LORA_PARSER";

/* ============================================================================
 * Hardcoded LoRa Function Names
 * ========================================================================== */

/**
 * @brief Ordered name table – index MUST match json_lora_function_id_t enum value.
 */
static const char *LORA_FUNCTION_NAMES[JSON_LORA_FUNC_MAX] = {
    // Lifecycle (0-3)
    "MODULE_HW_RESET",              // 0
    "MODULE_SW_RESET",              // 1
    "MODULE_GET_INFO",              // 2
    "MODULE_FACTORY_RESET",         // 3
    // Region / Class (4-5)
    "MODULE_SET_REGION",            // 4
    "MODULE_SET_CLASS",             // 5
    // OTAA Provisioning (6-11)
    "MODULE_SET_JOIN_MODE",         // 6
    "MODULE_SET_DEVEUI",            // 7
    "MODULE_GET_DEVEUI",            // 8
    "MODULE_SET_APPEUI",            // 9
    "MODULE_SET_APPKEY",            // 10
    "MODULE_JOIN",                  // 11
    // Join Status / ABP (12-15)
    "MODULE_GET_JOIN_STATUS",       // 12
    "MODULE_SET_DEVADDR",           // 13
    "MODULE_SET_NWKSKEY",           // 14
    "MODULE_SET_APPSKEY",           // 15
    // MAC / RF (16-21)
    "MODULE_SET_DR",                // 16
    "MODULE_SET_ADR",               // 17
    "MODULE_SET_TXP",               // 18
    "MODULE_SET_CHANNEL",           // 19
    "MODULE_SET_CONFIRM",           // 20
    "MODULE_SET_PUBLIC_NET",        // 21
    // Data plane (22-24)
    "MODULE_SEND_UNCONFIRMED",      // 22
    "MODULE_SEND_CONFIRMED",        // 23
    "MODULE_READ_RECV",             // 24
    // Port (25)
    "MODULE_SET_PORT",              // 25
    // ABP extended (26)
    "MODULE_GET_DEVADDR",           // 26
    // MAC extended (27-30)
    "MODULE_SET_RETRY",             // 27
    "MODULE_SET_REPT",              // 28
    "MODULE_SET_RXWIN2",            // 29
    "MODULE_SET_DELAY",             // 30
    // Data plane extended (31-32)
    "MODULE_SEND_HEX",              // 31
    "MODULE_SEND_CONFIRMED_HEX",    // 32
    // Utility (33-34)
    "MODULE_CHECK_PAYLOAD_LEN",     // 33
    "MODULE_GET_VDD",               // 34
    // Power management (35-38)
    "MODULE_LOWPOWER",              // 35
    "MODULE_LOWPOWER_AUTO_ON",      // 36
    "MODULE_LOWPOWER_AUTO_OFF",     // 37
    "MODULE_WAKEUP_NOTIFY",         // 38
};

/* ============================================================================
 * Internal Helper Functions
 * ========================================================================== */

/**
 * @brief Look up function ID by name string
 */
static json_lora_function_id_t get_function_id(const char *function_name) {
    for (int i = 0; i < JSON_LORA_FUNC_MAX; i++) {
        if (LORA_FUNCTION_NAMES[i] == NULL) continue;
        if (strcmp(function_name, LORA_FUNCTION_NAMES[i]) == 0) {
            return (json_lora_function_id_t)i;
        }
    }
    return JSON_LORA_FUNC_MAX; // Not found
}

/**
 * @brief Parse GPIO control array from a JSON array node
 */
static esp_err_t parse_gpio_array(cJSON *gpio_array, gpio_control_t *gpio_out,
                                  uint8_t *count) {
    if (!cJSON_IsArray(gpio_array)) {
        *count = 0;
        return ESP_OK; // Empty / absent array is valid
    }

    int array_size = cJSON_GetArraySize(gpio_array);
    if (array_size > MAX_GPIO_ACTIONS) {
        ESP_LOGE(TAG, "Too many GPIO actions: %d (max %d)", array_size,
                 MAX_GPIO_ACTIONS);
        return ESP_ERR_INVALID_ARG;
    }

    *count = 0;
    cJSON *gpio_item = NULL;
    cJSON_ArrayForEach(gpio_item, gpio_array) {
        cJSON *pin   = cJSON_GetObjectItem(gpio_item, "pin");
        cJSON *state = cJSON_GetObjectItem(gpio_item, "state");

        if (!cJSON_IsString(pin)) {
            ESP_LOGE(TAG, "Invalid GPIO pin field");
            return ESP_ERR_INVALID_ARG;
        }
        if (!cJSON_IsString(state)) {
            ESP_LOGE(TAG, "Invalid GPIO state field");
            return ESP_ERR_INVALID_ARG;
        }

        strncpy(gpio_out[*count].pin, pin->valuestring, MAX_PIN_ID_LEN - 1);
        gpio_out[*count].pin[MAX_PIN_ID_LEN - 1] = '\0';

        if (strcmp(state->valuestring, "HIGH") == 0) {
            gpio_out[*count].state = true;
        } else if (strcmp(state->valuestring, "LOW") == 0) {
            gpio_out[*count].state = false;
        } else {
            ESP_LOGE(TAG, "Invalid GPIO state: %s", state->valuestring);
            return ESP_ERR_INVALID_ARG;
        }

        (*count)++;
    }

    return ESP_OK;
}

/**
 * @brief Parse a single LoRa function entry from a JSON object node
 */
static esp_err_t parse_function(cJSON *func_json,
                                json_lora_function_config_t *func_out) {
    // function_name
    cJSON *function_name = cJSON_GetObjectItem(func_json, "function_name");
    if (!cJSON_IsString(function_name)) {
        ESP_LOGE(TAG, "Missing or invalid 'function_name' field");
        return ESP_ERR_INVALID_ARG;
    }

    json_lora_function_id_t func_id = get_function_id(function_name->valuestring);
    if (func_id == JSON_LORA_FUNC_MAX) {
        ESP_LOGE(TAG, "Unknown LoRa function: %s", function_name->valuestring);
        return ESP_ERR_INVALID_ARG;
    }
    func_out->function_id = func_id;
    func_out->available   = true;

    // command
    cJSON *command = cJSON_GetObjectItem(func_json, "command");
    if (cJSON_IsString(command)) {
        strncpy(func_out->command, command->valuestring, LORA_COMMAND_LEN - 1);
        func_out->command[LORA_COMMAND_LEN - 1] = '\0';
    } else {
        func_out->command[0] = '\0';
    }

    // is_prefix
    cJSON *is_prefix = cJSON_GetObjectItem(func_json, "is_prefix");
    func_out->is_prefix = cJSON_IsBool(is_prefix) ? cJSON_IsTrue(is_prefix) : false;

    // is_hex
    cJSON *is_hex = cJSON_GetObjectItem(func_json, "is_hex");
    func_out->is_hex = cJSON_IsBool(is_hex) && cJSON_IsTrue(is_hex);

    // gpio_start_control
    cJSON *gpio_start = cJSON_GetObjectItem(func_json, "gpio_start_control");
    esp_err_t ret = parse_gpio_array(gpio_start, func_out->gpio_start,
                                     &func_out->gpio_start_count);
    if (ret != ESP_OK) return ret;

    // delay_start
    cJSON *delay_start = cJSON_GetObjectItem(func_json, "delay_start");
    func_out->delay_start_ms =
        cJSON_IsNumber(delay_start) ? (uint16_t)delay_start->valueint : 0;

    // expect_response
    cJSON *expect_response = cJSON_GetObjectItem(func_json, "expect_response");
    if (cJSON_IsString(expect_response)) {
        strncpy(func_out->expect_response, expect_response->valuestring,
                LORA_RESPONSE_LEN - 1);
        func_out->expect_response[LORA_RESPONSE_LEN - 1] = '\0';
    } else {
        func_out->expect_response[0] = '\0';
    }

    // timeout
    cJSON *timeout = cJSON_GetObjectItem(func_json, "timeout");
    func_out->timeout_ms = cJSON_IsNumber(timeout) ? (uint16_t)timeout->valueint : 0;

    // gpio_end_control
    cJSON *gpio_end = cJSON_GetObjectItem(func_json, "gpio_end_control");
    ret = parse_gpio_array(gpio_end, func_out->gpio_end, &func_out->gpio_end_count);
    if (ret != ESP_OK) return ret;

    // delay_end
    cJSON *delay_end = cJSON_GetObjectItem(func_json, "delay_end");
    func_out->delay_end_ms =
        cJSON_IsNumber(delay_end) ? (uint16_t)delay_end->valueint : 0;

    ESP_LOGI(TAG, "Parsed function: %s (ID=%d)", function_name->valuestring,
             func_id);

    return ESP_OK;
}

/* ============================================================================
 * Public API Implementation
 * ========================================================================== */

esp_err_t json_lora_config_parse(const char *json_str,
                                 json_lora_module_config_t *config) {
    if (json_str == NULL || config == NULL) {
        ESP_LOGE(TAG, "Invalid arguments");
        return ESP_ERR_INVALID_ARG;
    }

    // Parse metadata using common parser
    esp_err_t ret = json_config_parse_metadata(json_str, &config->metadata);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to parse metadata");
        return ret;
    }

    // Verify module type is LORA
    if (strcmp(config->metadata.module_type, "LORA") != 0) {
        ESP_LOGE(TAG, "Module type is not LORA: %s",
                 config->metadata.module_type);
        return ESP_ERR_INVALID_ARG;
    }

    // Re-parse JSON root to access functions array
    cJSON *root = cJSON_Parse(json_str);
    if (root == NULL) {
        ESP_LOGE(TAG, "Failed to parse JSON");
        return ESP_ERR_INVALID_ARG;
    }

    cJSON *functions = cJSON_GetObjectItem(root, "functions");
    if (!cJSON_IsArray(functions)) {
        ESP_LOGE(TAG, "Missing or invalid 'functions' array");
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }

    int func_array_size = cJSON_GetArraySize(functions);
    if (func_array_size > LORA_MAX_FUNCTIONS) {
        ESP_LOGE(TAG, "Too many functions: %d (max %d)", func_array_size,
                 LORA_MAX_FUNCTIONS);
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }

    // Initialise all slots
    config->function_count = 0;
    memset(config->functions, 0, sizeof(config->functions));

    cJSON *func_json = NULL;
    cJSON_ArrayForEach(func_json, functions) {
        json_lora_function_config_t temp_func;
        memset(&temp_func, 0, sizeof(temp_func));

        ret = parse_function(func_json, &temp_func);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to parse function");
            cJSON_Delete(root);
            return ret;
        }

        // Store indexed by function_id (same pattern as BLE parser)
        config->functions[temp_func.function_id] = temp_func;
        config->function_count++;
    }

    ESP_LOGI(TAG, "LoRa config parsed successfully: %d functions",
             config->function_count);

    cJSON_Delete(root);
    return ESP_OK;
}
