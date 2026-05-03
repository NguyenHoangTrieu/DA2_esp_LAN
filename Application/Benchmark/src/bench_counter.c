/**
 * @file bench_counter.c
 * @brief Per-protocol benchmark counters — BLE GATT, Zigbee, LoRa.
 *
 * Counters are protected by a FreeRTOS portMUX critical section so they can
 * be incremented safely from any task.  Every BENCH_REPORT_INTERVAL_MS a
 * snapshot is taken, counters are reset, and a JSON line is sent upstream via
 * the existing BLE GATT uplink path:
 *
 *   BENCH:{"ble_pkt":N,"ble_b":N,"ble_drop":N,
 *           "zb_pkt":N,"zb_b":N,"zb_drop":N,
 *           "lr_pkt":N,"lr_b":N,"lr_drop":N,"ms":2000}
 *
 * Because this uses HANDLER_BLE_GATT, the WAN MCU prepends the 3-byte tag
 * "BLG" before hex-encoding and publishing as {"data":"<hex>"}.  The monitor
 * widget strips the 3-byte prefix and matches BENCH: to display fw-reported
 * throughput alongside dashboard-estimated throughput.
 */

#include "bench_counter.h"
#include "mcu_wan_handler.h"
#include "frame_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "bench_ctr";

#define BENCH_TASK_STACK_WORDS  (4096 / sizeof(StackType_t))
#define BENCH_JSON_BUF_SIZE     256

/* ---------- Counters (portMUX protected) ---------- */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t s_ble_pkt  = 0;
static volatile uint32_t s_ble_b    = 0;
static volatile uint32_t s_ble_drop = 0;

static volatile uint32_t s_zb_pkt   = 0;
static volatile uint32_t s_zb_b     = 0;
static volatile uint32_t s_zb_drop  = 0;

static volatile uint32_t s_lr_pkt   = 0;
static volatile uint32_t s_lr_b     = 0;
static volatile uint32_t s_lr_drop  = 0;

static volatile bool     s_running  = false;

/* ---------- Public counter API ---------- */

void bench_count_ble(uint16_t payload_bytes) {
    portENTER_CRITICAL(&s_mux);
    s_ble_pkt++;
    s_ble_b += payload_bytes;
    portEXIT_CRITICAL(&s_mux);
}

void bench_count_zb(uint16_t payload_bytes) {
    portENTER_CRITICAL(&s_mux);
    s_zb_pkt++;
    s_zb_b += payload_bytes;
    portEXIT_CRITICAL(&s_mux);
}

void bench_count_lr(uint16_t payload_bytes) {
    portENTER_CRITICAL(&s_mux);
    s_lr_pkt++;
    s_lr_b += payload_bytes;
    portEXIT_CRITICAL(&s_mux);
}

void bench_count_ble_drop(void) {
    portENTER_CRITICAL(&s_mux);
    s_ble_drop++;
    portEXIT_CRITICAL(&s_mux);
}

void bench_count_zb_drop(void) {
    portENTER_CRITICAL(&s_mux);
    s_zb_drop++;
    portEXIT_CRITICAL(&s_mux);
}

void bench_count_lr_drop(void) {
    portENTER_CRITICAL(&s_mux);
    s_lr_drop++;
    portEXIT_CRITICAL(&s_mux);
}

/* ---------- Reporter task ---------- */

static void bench_reporter_task(void *arg) {
    char *buf = (char *)heap_caps_malloc(BENCH_JSON_BUF_SIZE,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        ESP_LOGE(TAG, "Cannot allocate bench JSON buffer");
        s_running = false;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Bench reporter started (interval %d ms)", BENCH_REPORT_INTERVAL_MS);

    while (s_running) {
        vTaskDelay(pdMS_TO_TICKS(BENCH_REPORT_INTERVAL_MS));
        if (!s_running) break;

        /* Atomic snapshot then reset */
        uint32_t ble_pkt, ble_b, ble_drop;
        uint32_t zb_pkt,  zb_b,  zb_drop;
        uint32_t lr_pkt,  lr_b,  lr_drop;

        portENTER_CRITICAL(&s_mux);
        ble_pkt  = s_ble_pkt;  ble_b  = s_ble_b;  ble_drop = s_ble_drop;
        zb_pkt   = s_zb_pkt;   zb_b   = s_zb_b;   zb_drop  = s_zb_drop;
        lr_pkt   = s_lr_pkt;   lr_b   = s_lr_b;   lr_drop  = s_lr_drop;
        s_ble_pkt  = s_ble_b  = s_ble_drop = 0;
        s_zb_pkt   = s_zb_b   = s_zb_drop  = 0;
        s_lr_pkt   = s_lr_b   = s_lr_drop  = 0;
        portEXIT_CRITICAL(&s_mux);

        /* Build compact JSON preceded by BENCH: marker */
        int n = snprintf(buf, BENCH_JSON_BUF_SIZE,
            "BENCH:{\"ble_pkt\":%lu,\"ble_b\":%lu,\"ble_drop\":%lu,"
                   "\"zb_pkt\":%lu,\"zb_b\":%lu,\"zb_drop\":%lu,"
                   "\"lr_pkt\":%lu,\"lr_b\":%lu,\"lr_drop\":%lu,"
                   "\"ms\":%d}",
            (unsigned long)ble_pkt, (unsigned long)ble_b,  (unsigned long)ble_drop,
            (unsigned long)zb_pkt,  (unsigned long)zb_b,   (unsigned long)zb_drop,
            (unsigned long)lr_pkt,  (unsigned long)lr_b,   (unsigned long)lr_drop,
            BENCH_REPORT_INTERVAL_MS);

        if (n <= 0 || n >= BENCH_JSON_BUF_SIZE) {
            ESP_LOGW(TAG, "Bench JSON truncated or error (%d)", n);
            continue;
        }

        /* Send via BLE GATT uplink path — WAN MCU will prepend "BLG" tag and
         * publish to ThingsBoard.  The monitor widget strips the 3-byte prefix
         * and matches the BENCH: marker. */
        if (!mcu_wan_enqueue_uplink(HANDLER_BLE_GATT, (uint8_t *)buf, (uint16_t)n)) {
            ESP_LOGW(TAG, "Bench report uplink enqueue failed (WAN queue full)");
        } else {
            ESP_LOGD(TAG, "Bench report: %s", buf);
        }
    }

    free(buf);
    ESP_LOGI(TAG, "Bench reporter stopped");
    vTaskDelete(NULL);
}

/* ---------- Public task lifecycle ---------- */

esp_err_t bench_task_start(void) {
    if (s_running) return ESP_OK;

    s_running = true;

    StackType_t  *stack = (StackType_t *)heap_caps_malloc(
        BENCH_TASK_STACK_WORDS * sizeof(StackType_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    StaticTask_t *tcb   = (StaticTask_t *)heap_caps_malloc(
        sizeof(StaticTask_t),
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (!stack || !tcb) {
        ESP_LOGE(TAG, "Failed to allocate bench task memory");
        if (stack) heap_caps_free(stack);
        if (tcb)   heap_caps_free(tcb);
        s_running = false;
        return ESP_ERR_NO_MEM;
    }

    TaskHandle_t h = xTaskCreateStatic(
        bench_reporter_task, "bench_ctr",
        BENCH_TASK_STACK_WORDS, NULL,
        3,        /* lower than uplink tasks (5) */
        stack, tcb);

    if (!h) {
        ESP_LOGE(TAG, "xTaskCreateStatic failed");
        heap_caps_free(stack);
        heap_caps_free(tcb);
        s_running = false;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Bench counter task started");
    return ESP_OK;
}

void bench_task_stop(void) {
    s_running = false;
}
