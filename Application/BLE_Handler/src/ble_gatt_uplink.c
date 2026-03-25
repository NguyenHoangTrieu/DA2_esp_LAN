/**
 * @file ble_gatt_uplink.c
 * @brief BLE GATT Central uplink — batches CFBG: responses and sends to WAN MCU.
 */

#include "ble_gatt_uplink.h"
#include "frame_types.h"
#include "mcu_wan_handler.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "ble_gatt_up";

typedef struct {
    char     msg[BLE_GATT_UPLINK_MSG_MAX];
    uint16_t len;
} uplink_item_t;

static QueueHandle_t s_uplink_queue = NULL;
static TaskHandle_t  s_uplink_task  = NULL;
static volatile bool s_task_running = false;

/* --------------------------------------------------------------------------
 * Task
 * -------------------------------------------------------------------------- */

static void uplink_task(void *arg) {
    uplink_item_t *item = NULL;
    ESP_LOGI(TAG, "Uplink task started");

    while (s_task_running) {
        if (!item) {
            item = (uplink_item_t *)malloc(sizeof(uplink_item_t));
            if (!item) {
                ESP_LOGE(TAG, "OOM allocating uplink item");
                vTaskDelay(pdMS_TO_TICKS(200));
                continue;
            }
        }

        if (xQueueReceive(s_uplink_queue, item, pdMS_TO_TICKS(100)) == pdTRUE) {
            bool sent = mcu_wan_enqueue_uplink(HANDLER_BLE_GATT,
                                               (uint8_t *)item->msg,
                                               item->len);
            if (!sent) {
                ESP_LOGW(TAG, "WAN uplink queue full, dropped: %.*s",
                         (int)item->len, item->msg);
            } else {
                ESP_LOGD(TAG, "Uplink sent (%u B): %.*s",
                         item->len, (int)item->len, item->msg);
            }
        }
    }

    free(item);
    ESP_LOGI(TAG, "Uplink task exiting");
    vTaskDelete(NULL);
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

esp_err_t ble_gatt_uplink_task_start(void) {
    if (s_task_running) return ESP_OK;

    if (!s_uplink_queue) {
        s_uplink_queue = xQueueCreate(BLE_GATT_UPLINK_QUEUE_DEPTH,
                                      sizeof(uplink_item_t));
        if (!s_uplink_queue) {
            ESP_LOGE(TAG, "Failed to create uplink queue");
            return ESP_ERR_NO_MEM;
        }
    }

    s_task_running = true;
    BaseType_t ret = xTaskCreate(uplink_task, "ble_gatt_up",
                                 4 * 1024, NULL, 5, &s_uplink_task);
    if (ret != pdPASS) {
        s_task_running = false;
        return ESP_FAIL;
    }
    return ESP_OK;
}

void ble_gatt_uplink_task_stop(void) {
    s_task_running = false;
    vTaskDelay(pdMS_TO_TICKS(200));
    s_uplink_task = NULL;
}

esp_err_t ble_gatt_uplink_send_ok(uint8_t stack_id, const char *payload) {
    if (!s_uplink_queue || !payload) return ESP_ERR_INVALID_STATE;

    uplink_item_t item;
    (void)stack_id; /* native: no slot in response */
    int n = snprintf(item.msg, sizeof(item.msg), "CFBG:OK:%s", payload);
    if (n <= 0 || n >= (int)sizeof(item.msg)) {
        n = (int)sizeof(item.msg) - 1;
        item.msg[n] = '\0';
    }
    item.len = (uint16_t)n;

    if (xQueueSend(s_uplink_queue, &item, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "Uplink queue full (OK)");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t ble_gatt_uplink_send_fail(uint8_t stack_id, const char *reason) {
    if (!s_uplink_queue || !reason) return ESP_ERR_INVALID_STATE;

    uplink_item_t item;
    (void)stack_id; /* native: no slot in response */
    int n = snprintf(item.msg, sizeof(item.msg), "CFBG:FAIL:%s", reason);
    if (n <= 0 || n >= (int)sizeof(item.msg)) {
        n = (int)sizeof(item.msg) - 1;
        item.msg[n] = '\0';
    }
    item.len = (uint16_t)n;

    if (xQueueSend(s_uplink_queue, &item, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "Uplink queue full (FAIL)");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t ble_gatt_uplink_send_raw(const char *msg, uint16_t msg_len) {
    if (!s_uplink_queue || !msg || msg_len == 0) return ESP_ERR_INVALID_ARG;
    if (msg_len >= BLE_GATT_UPLINK_MSG_MAX) msg_len = BLE_GATT_UPLINK_MSG_MAX - 1;

    uplink_item_t item;
    memcpy(item.msg, msg, msg_len);
    item.msg[msg_len] = '\0';
    item.len = msg_len;

    if (xQueueSend(s_uplink_queue, &item, pdMS_TO_TICKS(50)) != pdTRUE) {
        ESP_LOGW(TAG, "Uplink queue full (raw)");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
