/**
 * @file bench_throughput.c
 * @brief Inter-MCU SPI throughput benchmark — LAN side (SPI Master).
 *
 * LAN→WAN direction  : bench_tp_sender floods mcu_wan_enqueue_uplink() with
 *                       max-size BNC frames as fast as the queue accepts them.
 * WAN→LAN direction  : the WAN side calls mcu_lan_enqueue_downlink(BNC,...),
 *                       which arrives here via the normal GPIO-ISR → DQ → DT
 *                       path; mcu_wan_handler_downlink calls
 *                       bench_throughput_count_rx() for each frame.
 *
 * Every BENCH_TP_REPORT_INTERVAL_MS a snapshot is taken and printed:
 *
 *   [BENCH_TP 2000ms]
 *     TX (LAN→WAN): pkt=N  bytes=N  kbps=XXX.X  drop=N
 *     RX (WAN→LAN): pkt=N  bytes=N  kbps=XXX.X
 *
 * Compile-time gate: BENCH_THROUGHPUT_ENABLE (bench_throughput.h).
 */

#include "bench_throughput.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "frame_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mcu_wan_handler.h"
#include <string.h>

static const char *TAG = "BENCH_TP";

#if BENCH_THROUGHPUT_ENABLE

/* ---------- Configuration ---------- */
#define BENCH_TP_TASK_STACK_WORDS (4096 / sizeof(StackType_t))
#define BENCH_TP_TASK_PRIORITY    2   /* below all real handler tasks */
#define BENCH_TP_PAYLOAD_LEN      INTER_MCU_PAYLOAD_MAX_LEN /* 2048 bytes */

/* ---------- Shared state (portMUX protected) ---------- */
static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static volatile uint32_t s_tx_pkt  = 0;
static volatile uint32_t s_tx_b    = 0;
static volatile uint32_t s_tx_drop = 0;
static volatile uint32_t s_rx_pkt  = 0;
static volatile uint32_t s_rx_b    = 0;

static volatile bool s_running = false;

/* Reusable fill buffer — allocated once in bench_throughput_start() */
static uint8_t *s_tx_buf = NULL;

/* ---------- Public counter API ---------- */

void bench_throughput_count_rx(uint32_t bytes) {
    portENTER_CRITICAL(&s_mux);
    s_rx_pkt++;
    s_rx_b += bytes;
    portEXIT_CRITICAL(&s_mux);
}

void bench_throughput_count_tx_drop(void) {
    portENTER_CRITICAL(&s_mux);
    s_tx_drop++;
    portEXIT_CRITICAL(&s_mux);
}

/* ---------- Sender task ---------- */

static void bench_tp_sender_task(void *arg) {
    ESP_LOGI(TAG, "Sender task started (payload=%u bytes, prio=%d)",
             BENCH_TP_PAYLOAD_LEN, BENCH_TP_TASK_PRIORITY);

    while (s_running) {
        bool ok = mcu_wan_enqueue_uplink(HANDLER_BENCH,
                                         s_tx_buf,
                                         (uint16_t)BENCH_TP_PAYLOAD_LEN);
        if (ok) {
            portENTER_CRITICAL(&s_mux);
            s_tx_pkt++;
            s_tx_b += BENCH_TP_PAYLOAD_LEN;
            portEXIT_CRITICAL(&s_mux);
        } else {
            portENTER_CRITICAL(&s_mux);
            s_tx_drop++;
            portEXIT_CRITICAL(&s_mux);
            /* Yield briefly when queue is full to avoid busy-spinning */
            taskYIELD();
        }
    }

    ESP_LOGI(TAG, "Sender task stopped");
    vTaskDelete(NULL);
}

/* ---------- Reporter task ---------- */

static void bench_tp_reporter_task(void *arg) {
    ESP_LOGI(TAG, "Reporter task started (interval=%d ms)",
             BENCH_TP_REPORT_INTERVAL_MS);

    while (s_running) {
        vTaskDelay(pdMS_TO_TICKS(BENCH_TP_REPORT_INTERVAL_MS));
        if (!s_running) break;

        /* Atomic snapshot + reset */
        uint32_t tx_pkt, tx_b, tx_drop, rx_pkt, rx_b;

        portENTER_CRITICAL(&s_mux);
        tx_pkt  = s_tx_pkt;  s_tx_pkt  = 0;
        tx_b    = s_tx_b;    s_tx_b    = 0;
        tx_drop = s_tx_drop; s_tx_drop = 0;
        rx_pkt  = s_rx_pkt;  s_rx_pkt  = 0;
        rx_b    = s_rx_b;    s_rx_b    = 0;
        portEXIT_CRITICAL(&s_mux);

        const float interval_s = (float)BENCH_TP_REPORT_INTERVAL_MS / 1000.0f;
        const float tx_kbps = (interval_s > 0.0f)
            ? ((float)tx_b * 8.0f) / (interval_s * 1000.0f) : 0.0f;
        const float rx_kbps = (interval_s > 0.0f)
            ? ((float)rx_b * 8.0f) / (interval_s * 1000.0f) : 0.0f;
        const float tx_pps  = (interval_s > 0.0f)
            ? (float)tx_pkt / interval_s : 0.0f;
        const float rx_pps  = (interval_s > 0.0f)
            ? (float)rx_pkt / interval_s : 0.0f;

        ESP_LOGI(TAG,
                 "[BENCH_TP %dms] "
                 "TX(LAN->WAN): pkt=%lu b=%lu pps=%.1f kbps=%.1f drop=%lu | "
                 "RX(WAN->LAN): pkt=%lu b=%lu pps=%.1f kbps=%.1f",
                 BENCH_TP_REPORT_INTERVAL_MS,
                 (unsigned long)tx_pkt, (unsigned long)tx_b,
                 tx_pps, tx_kbps, (unsigned long)tx_drop,
                 (unsigned long)rx_pkt, (unsigned long)rx_b,
                 rx_pps, rx_kbps);
    }

    ESP_LOGI(TAG, "Reporter task stopped");
    vTaskDelete(NULL);
}

/* ---------- Public lifecycle API ---------- */

esp_err_t bench_throughput_start(void) {
    if (s_running) return ESP_OK;

    /* Allocate fill buffer once */
    s_tx_buf = (uint8_t *)heap_caps_malloc(BENCH_TP_PAYLOAD_LEN,
                                           MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_tx_buf) {
        ESP_LOGE(TAG, "Failed to allocate TX buffer (%u bytes)",
                 BENCH_TP_PAYLOAD_LEN);
        return ESP_ERR_NO_MEM;
    }
    /* Fill with 0xAA pattern so the receiver can optionally verify */
    memset(s_tx_buf, 0xAA, BENCH_TP_PAYLOAD_LEN);

    s_running = true;

    /* --- Sender task --- */
    StackType_t  *send_stack = (StackType_t *)heap_caps_malloc(
        BENCH_TP_TASK_STACK_WORDS * sizeof(StackType_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    StaticTask_t *send_tcb   = (StaticTask_t *)heap_caps_malloc(
        sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (!send_stack || !send_tcb) {
        ESP_LOGE(TAG, "Failed to allocate sender task memory");
        heap_caps_free(s_tx_buf); s_tx_buf = NULL;
        if (send_stack) heap_caps_free(send_stack);
        if (send_tcb)   heap_caps_free(send_tcb);
        s_running = false;
        return ESP_ERR_NO_MEM;
    }

    TaskHandle_t h_sender = xTaskCreateStatic(
        bench_tp_sender_task, "bench_tp_tx",
        BENCH_TP_TASK_STACK_WORDS, NULL,
        BENCH_TP_TASK_PRIORITY,
        send_stack, send_tcb);

    if (!h_sender) {
        ESP_LOGE(TAG, "Failed to create sender task");
        heap_caps_free(s_tx_buf); s_tx_buf = NULL;
        heap_caps_free(send_stack);
        heap_caps_free(send_tcb);
        s_running = false;
        return ESP_FAIL;
    }

    /* --- Reporter task --- */
    StackType_t  *rep_stack = (StackType_t *)heap_caps_malloc(
        BENCH_TP_TASK_STACK_WORDS * sizeof(StackType_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    StaticTask_t *rep_tcb   = (StaticTask_t *)heap_caps_malloc(
        sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (!rep_stack || !rep_tcb) {
        ESP_LOGE(TAG, "Failed to allocate reporter task memory");
        /* sender is already running; stop it cleanly */
        s_running = false;
        if (rep_stack) heap_caps_free(rep_stack);
        if (rep_tcb)   heap_caps_free(rep_tcb);
        return ESP_ERR_NO_MEM;
    }

    TaskHandle_t h_reporter = xTaskCreateStatic(
        bench_tp_reporter_task, "bench_tp_rep",
        BENCH_TP_TASK_STACK_WORDS, NULL,
        BENCH_TP_TASK_PRIORITY,
        rep_stack, rep_tcb);

    if (!h_reporter) {
        ESP_LOGE(TAG, "Failed to create reporter task");
        s_running = false;
        heap_caps_free(rep_stack);
        heap_caps_free(rep_tcb);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Inter-MCU throughput benchmark started");
    ESP_LOGI(TAG, "  Payload size : %u bytes/frame", BENCH_TP_PAYLOAD_LEN);
    ESP_LOGI(TAG, "  Report every : %d ms", BENCH_TP_REPORT_INTERVAL_MS);
    return ESP_OK;
}

void bench_throughput_stop(void) {
    if (!s_running) return;
    s_running = false;
    /* Tasks detect s_running==false on their next iteration and self-delete */
    ESP_LOGI(TAG, "Stop requested — tasks will self-delete");
}

#else /* BENCH_THROUGHPUT_ENABLE == 0 */

void bench_throughput_count_rx(uint32_t bytes)  { (void)bytes; }
void bench_throughput_count_tx_drop(void)        {}

esp_err_t bench_throughput_start(void) {
    ESP_LOGI(TAG, "Inter-MCU throughput benchmark disabled (BENCH_THROUGHPUT_ENABLE=0)");
    return ESP_OK;
}

void bench_throughput_stop(void) {}

#endif /* BENCH_THROUGHPUT_ENABLE */
