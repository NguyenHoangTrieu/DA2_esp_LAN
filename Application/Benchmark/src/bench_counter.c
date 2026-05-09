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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <stdio.h>

static const char *TAG = "bench_ctr";

#define BENCH_TASK_STACK_WORDS  (4096 / sizeof(StackType_t))

#if BENCH_ENABLE

/* ---------- Counters (portMUX protected) ---------- */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t s_ble_rx_pkt = 0;
static volatile uint32_t s_ble_rx_b   = 0;

static volatile uint32_t s_ble_pkt  = 0;
static volatile uint32_t s_ble_b    = 0;
static volatile uint32_t s_ble_drop = 0;

static volatile uint32_t s_zb_pkt   = 0;
static volatile uint32_t s_zb_b     = 0;
static volatile uint32_t s_zb_drop  = 0;
static volatile uint32_t s_zb_rx_pkt = 0;
static volatile uint32_t s_zb_rx_b   = 0;

static volatile uint32_t s_lr_pkt   = 0;
static volatile uint32_t s_lr_b     = 0;
static volatile uint32_t s_lr_drop  = 0;
static volatile uint32_t s_lr_rx_pkt = 0;
static volatile uint32_t s_lr_rx_b   = 0;

static volatile bool     s_running  = false;

/* ---------- Public counter API ---------- */

void bench_count_ble(uint16_t payload_bytes) {
    portENTER_CRITICAL(&s_mux);
    s_ble_pkt++;
    s_ble_b += payload_bytes;
    portEXIT_CRITICAL(&s_mux);
}

void bench_count_ble_rx(uint16_t payload_bytes) {
    portENTER_CRITICAL(&s_mux);
    s_ble_rx_pkt++;
    s_ble_rx_b += payload_bytes;
    portEXIT_CRITICAL(&s_mux);
}

void bench_count_zb_rx(uint16_t payload_bytes) {
    portENTER_CRITICAL(&s_mux);
    s_zb_rx_pkt++;
    s_zb_rx_b += payload_bytes;
    portEXIT_CRITICAL(&s_mux);
}

void bench_count_zb_fwd(uint16_t payload_bytes) {
    portENTER_CRITICAL(&s_mux);
    s_zb_pkt++;
    s_zb_b += payload_bytes;
    portEXIT_CRITICAL(&s_mux);
}

void bench_count_lr_rx(uint16_t payload_bytes) {
    portENTER_CRITICAL(&s_mux);
    s_lr_rx_pkt++;
    s_lr_rx_b += payload_bytes;
    portEXIT_CRITICAL(&s_mux);
}

void bench_count_lr_fwd(uint16_t payload_bytes) {
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
    ESP_LOGI(TAG, "Bench reporter started (interval %d ms)", BENCH_REPORT_INTERVAL_MS);

    while (s_running) {
        vTaskDelay(pdMS_TO_TICKS(BENCH_REPORT_INTERVAL_MS));
        if (!s_running) break;

        /* Atomic snapshot then reset */
        uint32_t ble_rx_pkt, ble_rx_b;
        uint32_t ble_pkt, ble_b, ble_drop;
        uint32_t zb_rx_pkt, zb_rx_b, zb_pkt,  zb_b,  zb_drop;
        uint32_t lr_rx_pkt, lr_rx_b, lr_pkt,  lr_b,  lr_drop;

        portENTER_CRITICAL(&s_mux);
        ble_rx_pkt = s_ble_rx_pkt; ble_rx_b = s_ble_rx_b;
        ble_pkt  = s_ble_pkt;  ble_b  = s_ble_b;  ble_drop = s_ble_drop;
        zb_rx_pkt = s_zb_rx_pkt; zb_rx_b = s_zb_rx_b;
        zb_pkt   = s_zb_pkt;   zb_b   = s_zb_b;   zb_drop  = s_zb_drop;
        lr_rx_pkt = s_lr_rx_pkt; lr_rx_b = s_lr_rx_b;
        lr_pkt   = s_lr_pkt;   lr_b   = s_lr_b;   lr_drop  = s_lr_drop;
        s_ble_rx_pkt = s_ble_rx_b = 0;
        s_ble_pkt  = s_ble_b  = s_ble_drop = 0;
        s_zb_rx_pkt = s_zb_rx_b = 0;
        s_zb_pkt   = s_zb_b   = s_zb_drop  = 0;
        s_lr_rx_pkt = s_lr_rx_b = 0;
        s_lr_pkt   = s_lr_b   = s_lr_drop  = 0;
        portEXIT_CRITICAL(&s_mux);

        {
            const float interval_s = (float)BENCH_REPORT_INTERVAL_MS / 1000.0f;
            const float ble_rx_kbps = interval_s > 0.0f ? ((float)ble_rx_b * 8.0f) / (interval_s * 1000.0f) : 0.0f;
            const float ble_rx_pps  = interval_s > 0.0f ? ((float)ble_rx_pkt) / interval_s : 0.0f;
            const float ble_kbps = interval_s > 0.0f ? ((float)ble_b * 8.0f) / (interval_s * 1000.0f) : 0.0f;
            const float zb_rx_kbps = interval_s > 0.0f ? ((float)zb_rx_b * 8.0f) / (interval_s * 1000.0f) : 0.0f;
            const float zb_kbps  = interval_s > 0.0f ? ((float)zb_b  * 8.0f) / (interval_s * 1000.0f) : 0.0f;
            const float lr_rx_kbps = interval_s > 0.0f ? ((float)lr_rx_b * 8.0f) / (interval_s * 1000.0f) : 0.0f;
            const float lr_kbps  = interval_s > 0.0f ? ((float)lr_b  * 8.0f) / (interval_s * 1000.0f) : 0.0f;
            const float ble_pps  = interval_s > 0.0f ? ((float)ble_pkt) / interval_s : 0.0f;
            const float zb_rx_pps = interval_s > 0.0f ? ((float)zb_rx_pkt) / interval_s : 0.0f;
            const float zb_pps   = interval_s > 0.0f ? ((float)zb_pkt) / interval_s : 0.0f;
            const float lr_rx_cps = interval_s > 0.0f ? ((float)lr_rx_pkt) / interval_s : 0.0f;
            const float lr_cps   = interval_s > 0.0f ? ((float)lr_pkt) / interval_s : 0.0f;
            const float ble_fwd_ratio = (ble_rx_b > 0) ? (((float)ble_b * 100.0f) / (float)ble_rx_b) : 0.0f;
            const float ble_drop_ratio = (ble_rx_pkt > 0) ? (((float)ble_drop * 100.0f) / (float)ble_rx_pkt) : 0.0f;
            const float zb_fwd_ratio = (zb_rx_b > 0) ? (((float)zb_b * 100.0f) / (float)zb_rx_b) : 0.0f;
            const float zb_drop_ratio = (zb_rx_pkt > 0) ? (((float)zb_drop * 100.0f) / (float)zb_rx_pkt) : 0.0f;
            const float lr_fwd_ratio = (lr_rx_b > 0) ? (((float)lr_b * 100.0f) / (float)lr_rx_b) : 0.0f;
            const float lr_drop_ratio = (lr_rx_pkt > 0) ? (((float)lr_drop * 100.0f) / (float)lr_rx_pkt) : 0.0f;
            const uint32_t agg_b = ble_b + zb_b + lr_b;
            const uint32_t agg_pkt = ble_pkt + zb_pkt + lr_pkt;
            const uint32_t agg_drop = ble_drop + zb_drop + lr_drop;
            const float agg_kbps = interval_s > 0.0f ? ((float)agg_b * 8.0f) / (interval_s * 1000.0f) : 0.0f;

            ESP_LOGI(TAG,
                     "[BENCH %dms] BLE_RX pkt=%lu b=%lu pps=%.1f kbps=%.1f | "
                     "BLE_FWD pkt=%lu b=%lu drop=%lu pps=%.1f kbps=%.1f fwd=%.1f%% drop=%.1f%% | "
                     "ZB_RX pkt=%lu b=%lu pps=%.1f kbps=%.1f | "
                     "ZB_FWD pkt=%lu b=%lu drop=%lu pps=%.1f kbps=%.1f fwd=%.1f%% drop=%.1f%% | "
                     "LR_RX chunk=%lu b=%lu cps=%.1f kbps_raw=%.1f | "
                     "LR_FWD chunk=%lu b=%lu drop=%lu cps=%.1f kbps_raw=%.1f fwd=%.1f%% drop=%.1f%% | "
                     "AGG pkt=%lu b=%lu drop=%lu kbps=%.1f",
                     BENCH_REPORT_INTERVAL_MS,
                     (unsigned long)ble_rx_pkt, (unsigned long)ble_rx_b, ble_rx_pps, ble_rx_kbps,
                     (unsigned long)ble_pkt, (unsigned long)ble_b, (unsigned long)ble_drop,
                     ble_pps, ble_kbps, ble_fwd_ratio, ble_drop_ratio,
                     (unsigned long)zb_rx_pkt, (unsigned long)zb_rx_b, zb_rx_pps, zb_rx_kbps,
                     (unsigned long)zb_pkt,  (unsigned long)zb_b,  (unsigned long)zb_drop,
                     zb_pps,  zb_kbps, zb_fwd_ratio, zb_drop_ratio,
                     (unsigned long)lr_rx_pkt, (unsigned long)lr_rx_b, lr_rx_cps, lr_rx_kbps,
                     (unsigned long)lr_pkt,  (unsigned long)lr_b,  (unsigned long)lr_drop,
                     lr_cps,  lr_kbps, lr_fwd_ratio, lr_drop_ratio,
                     (unsigned long)agg_pkt, (unsigned long)agg_b, (unsigned long)agg_drop, agg_kbps);
        }
    }

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

#else

void bench_count_ble(uint16_t payload_bytes) { (void)payload_bytes; }
void bench_count_ble_rx(uint16_t payload_bytes) { (void)payload_bytes; }
void bench_count_zb_rx(uint16_t payload_bytes) { (void)payload_bytes; }
void bench_count_zb_fwd(uint16_t payload_bytes) { (void)payload_bytes; }
void bench_count_lr_rx(uint16_t payload_bytes) { (void)payload_bytes; }
void bench_count_lr_fwd(uint16_t payload_bytes) { (void)payload_bytes; }
void bench_count_ble_drop(void) {}
void bench_count_zb_drop(void) {}
void bench_count_lr_drop(void) {}

esp_err_t bench_task_start(void) {
    ESP_LOGI(TAG, "Benchmark disabled (BENCH_ENABLE=0)");
    return ESP_OK;
}

void bench_task_stop(void) {}

#endif
