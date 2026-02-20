/**
 * @file config_global.c
 * @brief Global configuration variables implementation
 */

#include "config_global.h"
#include "esp_log.h"
#include <stdlib.h>
#include <string.h>

static const char *TAG = "config_global";

/* ===== Global Configuration Variables ===== */
char g_stack_1_id[4] = MODULE_ID_NONE;
char g_stack_2_id[4] = MODULE_ID_NONE;
uint32_t g_rs485_baudrate = 115200; // Default baudrate

char *g_stack_1_json_config = NULL;
uint16_t g_stack_1_json_len = 0;

char *g_stack_2_json_config = NULL;
uint16_t g_stack_2_json_len = 0;

/* ===== Getter Functions ===== */

const char* config_get_stack_1_id(void) {
    return g_stack_1_id;
}

const char* config_get_stack_2_id(void) {
    return g_stack_2_id;
}

uint32_t config_get_rs485_baudrate(void) {
    return g_rs485_baudrate;
}

const char* config_get_stack_1_json(uint16_t *len) {
    if (len) {
        *len = g_stack_1_json_len;
    }
    return g_stack_1_json_config;
}

const char* config_get_stack_2_json(uint16_t *len) {
    if (len) {
        *len = g_stack_2_json_len;
    }
    return g_stack_2_json_config;
}

/* ===== Setter Functions ===== */

void config_set_stack_1_id(const char *module_id) {
    if (module_id && strlen(module_id) <= 3) {
        strncpy(g_stack_1_id, module_id, sizeof(g_stack_1_id) - 1);
        g_stack_1_id[3] = '\0';
        ESP_LOGI(TAG, "Stack 1 ID set to: %s", g_stack_1_id);
    }
}

void config_set_stack_2_id(const char *module_id) {
    if (module_id && strlen(module_id) <= 3) {
        strncpy(g_stack_2_id, module_id, sizeof(g_stack_2_id) - 1);
        g_stack_2_id[3] = '\0';
        ESP_LOGI(TAG, "Stack 2 ID set to: %s", g_stack_2_id);
    }
}

void config_set_rs485_baudrate(uint32_t baudrate) {
    g_rs485_baudrate = baudrate;
    ESP_LOGI(TAG, "RS485 baudrate set to: %lu", g_rs485_baudrate);
}

void config_set_stack_1_json(const char *json_str, uint16_t len) {
    if (!json_str || len == 0) {
        ESP_LOGE(TAG, "Invalid Stack 1 JSON config (len=%u)", len);
        return;
    }

    free(g_stack_1_json_config);
    g_stack_1_json_config = malloc(len + 1);
    if (!g_stack_1_json_config) {
        ESP_LOGE(TAG, "Failed to alloc Stack 1 JSON (%u bytes)", len);
        g_stack_1_json_len = 0;
        return;
    }
    memcpy(g_stack_1_json_config, json_str, len);
    g_stack_1_json_config[len] = '\0';
    g_stack_1_json_len = len;
    ESP_LOGI(TAG, "Stack 1 JSON config saved (%u bytes)", len);
}

void config_set_stack_2_json(const char *json_str, uint16_t len) {
    if (!json_str || len == 0) {
        ESP_LOGE(TAG, "Invalid Stack 2 JSON config (len=%u)", len);
        return;
    }

    free(g_stack_2_json_config);
    g_stack_2_json_config = malloc(len + 1);
    if (!g_stack_2_json_config) {
        ESP_LOGE(TAG, "Failed to alloc Stack 2 JSON (%u bytes)", len);
        g_stack_2_json_len = 0;
        return;
    }
    memcpy(g_stack_2_json_config, json_str, len);
    g_stack_2_json_config[len] = '\0';
    g_stack_2_json_len = len;
    ESP_LOGI(TAG, "Stack 2 JSON config saved (%u bytes)", len);
}

void config_global_init(void) {
    strncpy(g_stack_1_id, MODULE_ID_NONE, sizeof(g_stack_1_id));
    strncpy(g_stack_2_id, MODULE_ID_NONE, sizeof(g_stack_2_id));
    g_rs485_baudrate = 115200;

    free(g_stack_1_json_config);
    g_stack_1_json_config = NULL;
    g_stack_1_json_len = 0;

    free(g_stack_2_json_config);
    g_stack_2_json_config = NULL;
    g_stack_2_json_len = 0;

    ESP_LOGI(TAG, "Config globals initialized");
}
