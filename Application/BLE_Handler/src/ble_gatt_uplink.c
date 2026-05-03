/**
 * @file ble_gatt_uplink.c
 * @brief BLE GATT Central uplink — batches CFBG: responses and sends to WAN MCU.
 */

#include "ble_gatt_uplink.h"
#include "frame_types.h"
#include "mcu_wan_handler.h"
#include "bench_counter.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

static const char *TAG = "ble_gatt_up";

typedef struct {
    char    *msg;   /* heap-allocated from PSRAM, freed after dispatch */
    uint16_t len;
} uplink_item_t;

static QueueHandle_t s_uplink_queue = NULL;
static TaskHandle_t  s_uplink_task  = NULL;
static StackType_t   *s_uplink_stack = NULL;
static StaticTask_t  *s_uplink_tcb   = NULL;
static volatile bool s_task_running = false;

/* --------------------------------------------------------------------------
 * Task
 * -------------------------------------------------------------------------- */

static void uplink_task(void *arg) {
    uplink_item_t *item = NULL;
    ESP_LOGI(TAG, "Uplink task started");

    while (s_task_running) {
        if (xQueueReceive(s_uplink_queue, &item, pdMS_TO_TICKS(100)) == pdTRUE) {
            bool sent = mcu_wan_enqueue_uplink(HANDLER_BLE_GATT,
                                               (uint8_t *)item->msg,
                                               item->len);
            if (!sent) {
#if !BENCH_QUIET_LOG
                ESP_LOGW(TAG, "WAN uplink queue full, dropped: %.*s",
                         (int)item->len, item->msg);
#endif
            } else {
                ESP_LOGD(TAG, "Uplink sent (%u B): %.*s",
                         item->len, (int)item->len, item->msg);
            }
            free(item->msg);
            free(item);
            item = NULL;
        }
    }

    ESP_LOGI(TAG, "Uplink task exiting");
    vTaskDelete(NULL);
}

/* --------------------------------------------------------------------------
 * Public API
 * -------------------------------------------------------------------------- */

esp_err_t ble_gatt_uplink_task_start(void) {
    if (s_task_running) return ESP_OK;

    if (!s_uplink_queue) {
        /* Pointer queue: 16 × 8 bytes = 128 bytes internal RAM */
        s_uplink_queue = xQueueCreate(BLE_GATT_UPLINK_QUEUE_DEPTH,
                                      sizeof(uplink_item_t *));
        if (!s_uplink_queue) {
            ESP_LOGE(TAG, "Failed to create uplink queue");
            return ESP_ERR_NO_MEM;
        }
    }

    s_task_running = true;
    s_uplink_stack = (StackType_t *)heap_caps_malloc(4 * 1024, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_uplink_tcb = (StaticTask_t *)heap_caps_malloc(sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (!s_uplink_stack || !s_uplink_tcb) {
        ESP_LOGE(TAG, "Failed to allocate memory for uplink task");
        if (s_uplink_stack) heap_caps_free(s_uplink_stack);
        if (s_uplink_tcb) heap_caps_free(s_uplink_tcb);
        s_task_running = false;
        return ESP_FAIL;
    }

    s_uplink_task = xTaskCreateStatic(uplink_task, "ble_gatt_up", 4 * 1024 / sizeof(StackType_t),
                                      NULL, 5, s_uplink_stack, s_uplink_tcb);

    if (s_uplink_task == NULL) {
        s_task_running = false;
        heap_caps_free(s_uplink_stack);
        heap_caps_free(s_uplink_tcb);
        ESP_LOGE(TAG, "Failed to create uplink task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "BLE GATT Uplink task created in PSRAM");
    return ESP_OK;
}

void ble_gatt_uplink_task_stop(void) {
    s_task_running = false;
    vTaskDelay(pdMS_TO_TICKS(200));
    s_uplink_task = NULL;
    if (s_uplink_stack) heap_caps_free(s_uplink_stack);
    if (s_uplink_tcb) heap_caps_free(s_uplink_tcb);
    s_uplink_stack = NULL;
    s_uplink_tcb = NULL;
}

static uplink_item_t *alloc_uplink_item(uint16_t msg_cap) {
    uplink_item_t *item = malloc(sizeof(uplink_item_t));
    if (!item) return NULL;
    item->msg = heap_caps_malloc(msg_cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!item->msg) { free(item); return NULL; }
    return item;
}

esp_err_t ble_gatt_uplink_send_ok(uint8_t stack_id, const char *payload) {
    if (!s_uplink_queue || !payload) return ESP_ERR_INVALID_STATE;
    (void)stack_id;

    uplink_item_t *item = alloc_uplink_item(BLE_GATT_UPLINK_MSG_MAX);
    if (!item) return ESP_ERR_NO_MEM;

    int n = snprintf(item->msg, BLE_GATT_UPLINK_MSG_MAX, "CFBG:OK:%s", payload);
    if (n <= 0 || n >= BLE_GATT_UPLINK_MSG_MAX) n = BLE_GATT_UPLINK_MSG_MAX - 1;
    item->msg[n] = '\0';
    item->len = (uint16_t)n;

    if (xQueueSend(s_uplink_queue, &item, pdMS_TO_TICKS(50)) != pdTRUE) {
#if !BENCH_QUIET_LOG
        ESP_LOGW(TAG, "Uplink queue full (OK)");
#endif
        free(item->msg); free(item);
        /* Count the dropped NOTIFY event for benchmark drop tracking */
        if (strstr(payload, "NOTIFY:") == payload) bench_count_ble_drop();
        return ESP_ERR_NO_MEM;
    }
    /* Count each forwarded NOTIFY as a benchmark BLE event.  payload_bytes is
     * the full CFBG:OK:NOTIFY:... string length — good proxy for uplink cost. */
    if (strstr(payload, "NOTIFY:") == payload) bench_count_ble((uint16_t)item->len);
    return ESP_OK;
}

esp_err_t ble_gatt_uplink_send_fail(uint8_t stack_id, const char *reason) {
    if (!s_uplink_queue || !reason) return ESP_ERR_INVALID_STATE;
    (void)stack_id;

    uplink_item_t *item = alloc_uplink_item(BLE_GATT_UPLINK_MSG_MAX);
    if (!item) return ESP_ERR_NO_MEM;

    int n = snprintf(item->msg, BLE_GATT_UPLINK_MSG_MAX, "CFBG:FAIL:%s", reason);
    if (n <= 0 || n >= BLE_GATT_UPLINK_MSG_MAX) n = BLE_GATT_UPLINK_MSG_MAX - 1;
    item->msg[n] = '\0';
    item->len = (uint16_t)n;

    if (xQueueSend(s_uplink_queue, &item, pdMS_TO_TICKS(50)) != pdTRUE) {
#if !BENCH_QUIET_LOG
        ESP_LOGW(TAG, "Uplink queue full (FAIL)");
#endif
        free(item->msg); free(item);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t ble_gatt_uplink_send_raw(const char *msg, uint16_t msg_len) {
    if (!s_uplink_queue || !msg || msg_len == 0) return ESP_ERR_INVALID_ARG;
    if (msg_len >= BLE_GATT_UPLINK_MSG_MAX) msg_len = BLE_GATT_UPLINK_MSG_MAX - 1;

    uplink_item_t *item = alloc_uplink_item(msg_len + 1);
    if (!item) return ESP_ERR_NO_MEM;

    memcpy(item->msg, msg, msg_len);
    item->msg[msg_len] = '\0';
    item->len = msg_len;

    if (xQueueSend(s_uplink_queue, &item, pdMS_TO_TICKS(50)) != pdTRUE) {
#if !BENCH_QUIET_LOG
        ESP_LOGW(TAG, "Uplink queue full (raw)");
#endif
        free(item->msg); free(item);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}
