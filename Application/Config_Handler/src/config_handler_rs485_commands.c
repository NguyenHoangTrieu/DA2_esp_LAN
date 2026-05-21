/**
 * @file config_handler_rs485_commands.c
 * @brief RS485 configuration command handlers
 *
 * Handles:
 *   CFRS:JSON:<stack_id>:<json_data>  — Load GPIO mode config from JSON
 *
 * The baud-rate command (CFRS:BR:<baud>) is handled inline in config_handler.c.
 */

#include "config_handler_rs485_commands.h"
#include "json_rs485_config_parser.h"
#include "rs485_comm.h"
#include "rs485_handler.h"
#include "mcu_wan_handler.h"
#include "module_monitor_task.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "rs485_cmd";

static esp_err_t config_apply_rs485_json_config_internal(
    uint8_t stack_id, const char *json_data, uint16_t json_len) {
    if (!json_data || json_len < 2 || json_len > 4096) {
        ESP_LOGE(TAG, "RS485 JSON: invalid raw JSON parameters (stack=%u len=%u)",
                 stack_id, json_len);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG, "RS485 JSON: applying stack=%u, len=%u", stack_id, json_len);

    json_rs485_module_config_t rs485_cfg;
    esp_err_t ret = json_rs485_config_parse(json_data, json_len, &rs485_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RS485 JSON: parse failed: %s", esp_err_to_name(ret));
        return ret;
    }

    rs485_cfg.stack_id = stack_id;

    rs485_gpio_mode_config_t gpio_cfg;
    memset(&gpio_cfg, 0, sizeof(gpio_cfg));
    gpio_cfg.stack_id = stack_id;

    const json_rs485_function_config_t *send_fn =
        &rs485_cfg.functions[JSON_RS485_FUNC_SEND_MODE];
    if (send_fn->available) {
        uint8_t cnt = send_fn->gpio_start_count;
        if (cnt > RS485_COMM_MAX_GPIO_ACTIONS) cnt = RS485_COMM_MAX_GPIO_ACTIONS;
        for (uint8_t i = 0; i < cnt; i++) {
            const char *pin_str = send_fn->gpio_start[i].pin;
            if (strlen(pin_str) < 2) continue;
            gpio_cfg.send_actions[i].pin_1indexed = (uint8_t)(pin_str[1] - '0');
            gpio_cfg.send_actions[i].state = send_fn->gpio_start[i].state;
        }
        gpio_cfg.send_count = cnt;
        gpio_cfg.send_delay_ms = send_fn->delay_start_ms;
    }

    const json_rs485_function_config_t *recv_fn =
        &rs485_cfg.functions[JSON_RS485_FUNC_RECEIVE_MODE];
    if (recv_fn->available) {
        uint8_t cnt = recv_fn->gpio_start_count;
        if (cnt > RS485_COMM_MAX_GPIO_ACTIONS) cnt = RS485_COMM_MAX_GPIO_ACTIONS;
        for (uint8_t i = 0; i < cnt; i++) {
            const char *pin_str = recv_fn->gpio_start[i].pin;
            if (strlen(pin_str) < 2) continue;
            gpio_cfg.recv_actions[i].pin_1indexed = (uint8_t)(pin_str[1] - '0');
            gpio_cfg.recv_actions[i].state = recv_fn->gpio_start[i].state;
        }
        gpio_cfg.recv_count = cnt;
        gpio_cfg.recv_delay_ms = recv_fn->delay_start_ms;
    }

    ret = rs485_comm_load_gpio_config(&gpio_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RS485 JSON: failed to load GPIO config: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "RS485 JSON config applied: SEND[%d actions, %dms], RECV[%d actions, %dms]",
             gpio_cfg.send_count, gpio_cfg.send_delay_ms,
             gpio_cfg.recv_count, gpio_cfg.recv_delay_ms);
    return ESP_OK;
}

esp_err_t config_apply_rs485_json_config(uint8_t stack_id, const char *json_data,
                                         uint16_t json_len) {
    if (stack_id > 1) {
        ESP_LOGE(TAG, "RS485 JSON: invalid stack_id %u", stack_id);
        return ESP_ERR_INVALID_ARG;
    }

    return config_apply_rs485_json_config_internal(stack_id, json_data, json_len);
}

/* ============================================================================
 * Public API
 * ========================================================================== */

/**
 * @brief Parse and apply RS485 JSON GPIO configuration
 *
 * Format: "CFRS:JSON:<stack_id>:<json_data>"
 */
esp_err_t config_parse_rs485_json(const uint8_t *data, uint16_t len) {
    if (!data || len < 14) { /* "CFRS:JSON:0:{}" */
        ESP_LOGE(TAG, "RS485 JSON: invalid parameters (len=%u)", len);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check prefix */
    if (strncmp((const char *)data, "CFRS:JSON:", 10) != 0) {
        ESP_LOGE(TAG, "RS485 JSON: invalid prefix");
        return ESP_FAIL;
    }

    /* Parse: CFRS:JSON:<stack_id>:<json_data> */
    const char *ptr = (const char *)(data + 10);
    const char *colon = strchr(ptr, ':');
    if (!colon) {
        ESP_LOGE(TAG, "RS485 JSON: missing separator after stack_id");
        return ESP_FAIL;
    }

    uint8_t stack_id = (uint8_t)atoi(ptr);
    if (stack_id > 1) {
        ESP_LOGE(TAG, "RS485 JSON: invalid stack_id %u", stack_id);
        return ESP_FAIL;
    }

    const char *json_data = colon + 1;
    uint16_t json_len = (uint16_t)(len - (json_data - (const char *)data));

    esp_err_t ret = config_apply_rs485_json_config_internal(stack_id, json_data, json_len);
    if (ret != ESP_OK) {
        return ret;
    }

    ret = module_monitor_notify_rs485_configured(stack_id, json_data, json_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RS485 JSON: failed to notify module_monitor: %s",
                 esp_err_to_name(ret));
        const char ack[] = "CFRS:JSON:FAIL:QUEUE";
        mcu_wan_enqueue_uplink_local(HANDLER_RS485, (uint8_t *)ack, sizeof(ack) - 1);
        return ret;
    }

    ESP_LOGI(TAG, "RS485 JSON: config forwarded to module_monitor_task");
    return ESP_OK;
}

/**
 * @brief Parse and send RS485 downlink data
 *
 * Format: "CFRS:<stack_id>:DATA:<hex_data>"
 * Example: "CFRS:0:DATA:010306000A" → sends hex bytes 01 03 06 00 0A
 */
esp_err_t config_parse_rs485_downlink(const uint8_t *data, uint16_t len) {
    if (!data || len < 14) { /* Minimum: "CFRS:0:DATA:" */
        ESP_LOGE(TAG, "RS485 Downlink: invalid parameters (len=%u)", len);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check prefix */
    if (strncmp((const char *)data, "CFRS:", 5) != 0) {
        ESP_LOGE(TAG, "RS485 Downlink: invalid prefix");
        return ESP_FAIL;
    }

    /* Parse: CFRS:<stack_id>:DATA:<hex_data> */
    const char *ptr = (const char *)(data + 5);
    const char *colon = strchr(ptr, ':');
    if (!colon) {
        ESP_LOGE(TAG, "RS485 Downlink: missing stack_id separator");
        return ESP_FAIL;
    }

    uint8_t stack_id = (uint8_t)atoi(ptr);
    if (stack_id > 1) {
        ESP_LOGE(TAG, "RS485 Downlink: invalid stack_id %u", stack_id);
        return ESP_FAIL;
    }

    /* Check DATA prefix */
    const char *data_part = colon + 1;
    if (strncmp(data_part, "DATA:", 5) != 0) {
        ESP_LOGE(TAG, "RS485 Downlink: missing DATA: prefix");
        return ESP_FAIL;
    }

    const char *hex_str = data_part + 5;
    uint16_t hex_len = (uint16_t)(len - (hex_str - (const char *)data));

    if (hex_len == 0 || hex_len > 1024 || hex_len % 2 != 0) {
        ESP_LOGE(TAG, "RS485 Downlink: invalid hex length %u (must be even)", hex_len);
        return ESP_FAIL;
    }

    /* Convert hex string to binary */
    uint8_t *bin_data = (uint8_t *)malloc(hex_len / 2);
    if (!bin_data) {
        ESP_LOGE(TAG, "RS485 Downlink: malloc failed");
        return ESP_ERR_NO_MEM;
    }

    uint16_t bin_len = 0;
    for (uint16_t i = 0; i < hex_len; i += 2) {
        char hex_byte[3] = {hex_str[i], hex_str[i + 1], '\0'};
        uint8_t byte_val = (uint8_t)strtol(hex_byte, NULL, 16);
        bin_data[bin_len++] = byte_val;
    }

    rs485_gpio_mode_config_t gpio_cfg;
    esp_err_t cfg_ret = rs485_comm_get_gpio_config(&gpio_cfg);
    if (cfg_ret == ESP_OK) {
        ESP_LOGI(TAG, "RS485 Downlink: requested stack=%u, configured stack=%u",
                 stack_id, gpio_cfg.stack_id);
        if (gpio_cfg.stack_id != stack_id) {
            ESP_LOGW(TAG, "RS485 Downlink: stack mismatch, active RS485 config is stack=%u",
                     gpio_cfg.stack_id);
        }
    } else {
        ESP_LOGW(TAG, "RS485 Downlink: no RS485 JSON config loaded yet; send CFRS:JSON:%u:<json> first",
                 stack_id);
    }

    ESP_LOGI(TAG, "RS485 Downlink: stack=%u, hex_len=%u, bin_len=%u",
             stack_id, hex_len, bin_len);
    ESP_LOG_BUFFER_HEX(TAG, bin_data, bin_len);

    /* Send to RS485 handler */
    bool success = rs485_handler_enqueue_downlink(bin_data, bin_len);
    free(bin_data);

    if (success) {
        ESP_LOGI(TAG, "RS485 Downlink: sent %u bytes to handler", bin_len);
        const char ack[] = "CFRS:DATA:OK";
        mcu_wan_enqueue_uplink_local(HANDLER_RS485, (uint8_t *)ack, sizeof(ack) - 1);
        return ESP_OK;
    } else {
        ESP_LOGE(TAG, "RS485 Downlink: enqueue failed for stack=%u, len=%u (see RS485_HANDLER logs)",
                 stack_id, bin_len);
        return ESP_FAIL;
    }
}
