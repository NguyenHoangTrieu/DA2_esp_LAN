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
#include "mcu_wan_handler.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "rs485_cmd";

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

    if (json_len < 2 || json_len > 4096) {
        ESP_LOGE(TAG, "RS485 JSON: invalid length %u", json_len);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "RS485 JSON: parsing stack=%u, len=%u", stack_id, json_len);

    /* Parse JSON into RS485 config struct */
    json_rs485_module_config_t rs485_cfg;
    esp_err_t ret = json_rs485_config_parse(json_data, json_len, &rs485_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RS485 JSON: parse failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* Override stack_id from command prefix (more reliable than JSON body) */
    rs485_cfg.stack_id = stack_id;

    /* Convert parsed config → rs485_gpio_mode_config_t for BSP layer */
    rs485_gpio_mode_config_t gpio_cfg;
    memset(&gpio_cfg, 0, sizeof(gpio_cfg));
    gpio_cfg.stack_id = stack_id;

    /* SEND mode */
    const json_rs485_function_config_t *send_fn =
        &rs485_cfg.functions[JSON_RS485_FUNC_SEND_MODE];
    if (send_fn->available) {
        uint8_t cnt = send_fn->gpio_start_count;
        if (cnt > RS485_COMM_MAX_GPIO_ACTIONS) cnt = RS485_COMM_MAX_GPIO_ACTIONS;
        for (uint8_t i = 0; i < cnt; i++) {
            /* JSON pin "XY": X=stack port, Y=pin_1indexed */
            const char *pin_str = send_fn->gpio_start[i].pin;
            if (strlen(pin_str) < 2) continue;
            gpio_cfg.send_actions[i].pin_1indexed = (uint8_t)(pin_str[1] - '0');
            gpio_cfg.send_actions[i].state        = send_fn->gpio_start[i].state;
        }
        gpio_cfg.send_count    = cnt;
        gpio_cfg.send_delay_ms = send_fn->delay_start_ms;
    }

    /* RECEIVE mode */
    const json_rs485_function_config_t *recv_fn =
        &rs485_cfg.functions[JSON_RS485_FUNC_RECEIVE_MODE];
    if (recv_fn->available) {
        uint8_t cnt = recv_fn->gpio_start_count;
        if (cnt > RS485_COMM_MAX_GPIO_ACTIONS) cnt = RS485_COMM_MAX_GPIO_ACTIONS;
        for (uint8_t i = 0; i < cnt; i++) {
            const char *pin_str = recv_fn->gpio_start[i].pin;
            if (strlen(pin_str) < 2) continue;
            gpio_cfg.recv_actions[i].pin_1indexed = (uint8_t)(pin_str[1] - '0');
            gpio_cfg.recv_actions[i].state        = recv_fn->gpio_start[i].state;
        }
        gpio_cfg.recv_count    = cnt;
        gpio_cfg.recv_delay_ms = recv_fn->delay_start_ms;
    }

    /* Load into RS485 comm driver */
    ret = rs485_comm_load_gpio_config(&gpio_cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RS485 JSON: failed to load GPIO config: %s",
                 esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "RS485 JSON config applied: "
             "SEND[%d actions, %dms], RECV[%d actions, %dms]",
             gpio_cfg.send_count, gpio_cfg.send_delay_ms,
             gpio_cfg.recv_count, gpio_cfg.recv_delay_ms);

    /* Send ACK to WAN MCU / PC App */
    const char ack[] = "CFRS:JSON:OK";
    mcu_wan_enqueue_uplink(HANDLER_RS485, (uint8_t *)ack, sizeof(ack) - 1);

    return ESP_OK;
}
