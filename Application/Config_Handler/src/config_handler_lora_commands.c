/**
 * @file config_handler_lora_commands.c
 * @brief LoRa command parsers – RAW COMMAND PASS-THROUGH
 *
 * Mirrors config_handler_ble_commands.c with LoRa prefix "CFLR:" and
 * LoRa-specific task/handler types.
 *
 * KEY DESIGN: Commands are NOT interpreted here – they are passed through
 * verbatim to the LoRa module.  The JSON config loaded in lora_handler
 * determines GPIO sequencing, timing, and response matching.
 */

#include "config_handler.h"
#include "config_handler_lora_commands.h"
#include "lora_handler.h"
#include "lora_handler_task.h"
#include "module_monitor_task.h"
#include "json_lora_config_parser.h"
#include "esp_log.h"
#include "mcu_wan_handler.h"
#include "frame_types.h"
#include <string.h>
#include <stdlib.h>

#define LORA_MAX_STACKS 2

static const char *TAG = "lora_commands";

/* ===========================================================================
 * Unified LoRa Command Parser
 * ========================================================================= */

esp_err_t config_parse_lora_command(const uint8_t *data, uint16_t len) {
    if (!data || len < 8) { /* minimum: "CFLR:0:X" */
        ESP_LOGE(TAG, "LORA CMD: invalid parameters (len=%u)", len);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check prefix "CFLR:" */
    if (strncmp((const char *)data, "CFLR:", 5) != 0) {
        ESP_LOGE(TAG, "LORA CMD: invalid prefix");
        return ESP_FAIL;
    }

    /* Parse: CFLR:<stack_id>:<command> */
    const char *ptr   = (const char *)(data + 5);
    const char *colon = strchr(ptr, ':');
    if (!colon) {
        ESP_LOGE(TAG, "LORA CMD: missing separator");
        return ESP_FAIL;
    }

    uint8_t stack_id = (uint8_t)atoi(ptr);
    if (stack_id >= LORA_MAX_STACKS) {
        ESP_LOGE(TAG, "LORA CMD: invalid stack_id %u", stack_id);
        return ESP_FAIL;
    }

    const char *command = colon + 1;
    uint16_t    cmd_len = len - (uint16_t)(command - (const char *)data);

    if (cmd_len == 0 || cmd_len > 255) {
        ESP_LOGE(TAG, "LORA CMD: invalid command length %u", cmd_len);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "LORA CMD: stack=%u command='%.*s'", stack_id, cmd_len, command);

    /* Match command against loaded JSON config */
    lora_function_config_t func_config = {0};
    esp_err_t ret = lora_handler_get_function_by_command(stack_id, command, &func_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LORA CMD: no matching function for '%.*s'", cmd_len, command);
        char err_resp[64];
        int  err_len = snprintf(err_resp, sizeof(err_resp),
                                "CFLR:%d:FAIL:NO_MATCH", stack_id);
        if (err_len > 0) {
            mcu_wan_enqueue_uplink(HANDLER_LORA, (uint8_t *)err_resp,
                                   (uint16_t)err_len);
        }
        return ESP_FAIL;
    }

    /* Build command request */
    lora_command_request_t cmd_req = {0};
    cmd_req.stack_id    = stack_id;
    cmd_req.is_streaming = false;

    uint16_t copy_len = (cmd_len > sizeof(cmd_req.command) - 1)
                         ? (sizeof(cmd_req.command) - 1) : cmd_len;
    memcpy(cmd_req.command, command, copy_len);
    cmd_req.command[copy_len] = '\0';
    cmd_req.command_len = copy_len;

    memcpy(&cmd_req.func_config, &func_config, sizeof(lora_function_config_t));

    /* Enqueue to downlink task */
    ret = lora_handler_task_execute_command(&cmd_req);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LORA CMD: failed to enqueue: %s", esp_err_to_name(ret));
        char err_resp[64];
        int  err_len = snprintf(err_resp, sizeof(err_resp),
                                "CFLR:%d:FAIL:QUEUE_FULL", stack_id);
        if (err_len > 0) {
            mcu_wan_enqueue_uplink(HANDLER_LORA, (uint8_t *)err_resp,
                                   (uint16_t)err_len);
        }
        return ret;
    }

    ESP_LOGI(TAG, "LORA CMD: command enqueued successfully");
    return ESP_OK;
}

/* ===========================================================================
 * JSON Config Parser
 * ========================================================================= */

esp_err_t config_parse_lora_json(const uint8_t *data, uint16_t len) {
    if (!data || len < 15) { /* minimum: "CFLR:JSON:0:{}" */
        ESP_LOGE(TAG, "LORA JSON: invalid parameters (len=%u)", len);
        return ESP_ERR_INVALID_ARG;
    }

    /* Check prefix "CFLR:JSON:" */
    if (strncmp((const char *)data, "CFLR:JSON:", 10) != 0) {
        ESP_LOGE(TAG, "LORA JSON: invalid prefix");
        return ESP_FAIL;
    }

    /* Parse: CFLR:JSON:<stack_id>:<json_data> */
    const char *ptr   = (const char *)(data + 10);
    const char *colon = strchr(ptr, ':');
    if (!colon) {
        ESP_LOGE(TAG, "LORA JSON: missing separator");
        return ESP_FAIL;
    }

    uint8_t     stack_id  = (uint8_t)atoi(ptr);
    const char *json_data = colon + 1;
    uint16_t    json_len  = len - (uint16_t)(json_data - (const char *)data);

    ESP_LOGI(TAG, "LORA JSON: stack=%u, length=%u bytes", stack_id, json_len);

    if (stack_id >= LORA_MAX_STACKS) {
        ESP_LOGE(TAG, "LORA JSON: invalid stack_id %u", stack_id);
        return ESP_FAIL;
    }
    if (json_len < 2 || json_len > 8912) {
        ESP_LOGE(TAG, "LORA JSON: invalid length %u", json_len);
        return ESP_FAIL;
    }

    esp_err_t ret = module_monitor_send_config(stack_id, json_data, json_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LORA JSON: failed to forward to module_monitor: %s",
                 esp_err_to_name(ret));
        /* Notify App – module_monitor not running or queue full */
        uint8_t err_resp[] = "LR:JSON:FAIL:QUEUE";
        mcu_wan_enqueue_uplink(HANDLER_LORA, err_resp, sizeof(err_resp) - 1);
        return ret;
    }

    ESP_LOGI(TAG, "LORA JSON: config forwarded to module_monitor_task");
    /* ACK ("CFLR:JSON:OK" / "CFLR:JSON:FAIL") sent by module_monitor_task */
    return ESP_OK;
}
