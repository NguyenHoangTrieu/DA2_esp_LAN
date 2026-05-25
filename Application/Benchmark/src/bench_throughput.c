/**
 * @file bench_throughput.c
 * @brief Inter-MCU SPI throughput benchmark — LAN side (SPI Master).
 *        Sender + reporter; counters protected by portMUX.
 */

#include "bench_throughput.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "frame_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mcu_wan_handler.h"
#include "wan_comm.h"
#include <string.h>

/* Mode 1: counter in this task. Mode 2: counter in dispatcher. */
#define BENCH_TP_DIRECT_SEND BENCH_TP_MODE_DRIVER

extern wan_comm_handle_t g_wan_handle;
extern volatile bool g_handshake_done;

static const char *TAG = "BENCH_TP";

#if BENCH_THROUGHPUT_ENABLE

/* ---------- Configuration ---------- */
#define BENCH_TP_TASK_STACK_WORDS (4096 / sizeof(StackType_t))
#define BENCH_TP_TASK_PRIORITY    4   /* below uplink(5) to avoid starving it */
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

#if BENCH_TP_RAMP_ENABLE
/* Ramp rate steps (pps). Adjust to span expected range — start low so the
 * lossless plateau is clear, then exceed expected ceiling. */
static const uint32_t s_ramp_steps_pps[] = {
    200, 400, 600, 800, 1000, 1200, 1500, 2000, 2500, 3000
};
static const size_t s_ramp_n = sizeof(s_ramp_steps_pps) /
                               sizeof(s_ramp_steps_pps[0]);
static volatile uint32_t s_ramp_step       = 0;
static volatile uint32_t s_ramp_step_start = 0; /* ms since boot           */
static volatile uint32_t s_autostop_hits   = 0; /* consecutive over-thresh */
static uint64_t          s_next_send_us    = 0; /* token-bucket deadline   */
static uint32_t          s_idle_feed_cnt   = 0; /* WDT-feed cycle counter  */
#endif

uint32_t bench_throughput_current_pps(void) {
#if BENCH_TP_RAMP_ENABLE
    if (!s_running) return 0;
    uint32_t idx = s_ramp_step;
    if (idx >= s_ramp_n) idx = s_ramp_n - 1;
    return s_ramp_steps_pps[idx];
#else
    return 0;
#endif
}

#if BENCH_TP_DIRECT_SEND
/* DT inner: [BNC 3B][len 2B BE][rtc 19B][payload N]. */
#define BENCH_TP_INNER_LEN (3u + 2u + 19u + BENCH_TP_PAYLOAD_LEN)
static uint8_t *s_inner_buf = NULL;
#endif

/* ---------- Public counter API ---------- */

void bench_throughput_count_rx(uint32_t bytes) {
    portENTER_CRITICAL(&s_mux);
    s_rx_pkt++;
    s_rx_b += bytes;
    portEXIT_CRITICAL(&s_mux);
}

/* Mode 2: called by uplink dispatcher on send OK. */
void bench_throughput_count_tx(uint32_t bytes) {
    portENTER_CRITICAL(&s_mux);
    s_tx_pkt++;
    s_tx_b += bytes;
    portEXIT_CRITICAL(&s_mux);
}

void bench_throughput_count_tx_drop(void) {
    portENTER_CRITICAL(&s_mux);
    s_tx_drop++;
    portEXIT_CRITICAL(&s_mux);
}

/* Parse WAN→LAN BNC frames captured by wan_comm full-duplex flush.
 * Inner: [DT][BNC][len 2B BE][rtc 19B][payload]. Count payload bytes. */
static void bench_tp_rx_cb(const spi_frame_view_t *view, void *user) {
    (void)user;
    if (view == NULL || view->payload == NULL || view->len < 7) return;
    const uint8_t *p = view->payload;
    if (p[0] != 'D' || p[1] != 'T') return;
    if (p[2] != 'B' || p[3] != 'N' || p[4] != 'C') return;
    uint16_t data_length = ((uint16_t)p[5] << 8) | p[6];
    uint32_t payload_bytes = (data_length > 19u) ? (uint32_t)(data_length - 19u)
                                                  : (uint32_t)data_length;
    bench_throughput_count_rx(payload_bytes);
}

/* ---------- Sender task ---------- */

static void bench_tp_sender_task(void *arg) {
    ESP_LOGI(TAG, "Sender task started (payload=%u bytes, prio=%d, direct=%d)",
             BENCH_TP_PAYLOAD_LEN, BENCH_TP_TASK_PRIORITY, BENCH_TP_DIRECT_SEND);

    /* Wait for handshake before flooding (slave's CF capture window). */
    while (s_running && !g_handshake_done) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (!s_running) {
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "Handshake done — sender entering %s loop",
             BENCH_TP_RAMP_ENABLE ? "ramp" : "flood");

#if BENCH_TP_RAMP_ENABLE
    s_ramp_step_start = (uint32_t)(esp_log_timestamp());
#endif

    while (s_running) {
#if BENCH_TP_RAMP_ENABLE
        /* Token-bucket schedule via esp_timer_get_time() (microsecond
         * precision). vTaskDelay alone is wrong here: pdMS_TO_TICKS(5ms)
         * collapses to 0 ticks at the default 100Hz tick rate, so any rate
         * above ~100 pps silently degrades into a flood loop. We block-delay
         * for the coarse part of the wait and tight-yield for the rest. */
        const uint32_t pps = bench_throughput_current_pps();
        if (pps > 0) {
            uint64_t now_us = (uint64_t)esp_timer_get_time();
            if (s_next_send_us == 0) s_next_send_us = now_us;

            if (now_us < s_next_send_us) {
                uint64_t wait_us = s_next_send_us - now_us;
                if (wait_us >= 5000ULL) {
                    /* ≥5 ms: vTaskDelay so other tasks aren't starved. */
                    vTaskDelay(pdMS_TO_TICKS(wait_us / 1000ULL));
                } else if (wait_us >= 5ULL) {
                    /* Sub-tick: busy-wait with µs precision. taskYIELD here
                     * is too heavy (can hand off ~10 ms) and undershoots
                     * the requested rate at >1 kHz. esp_rom_delay_us burns
                     * CPU for the wait but is exact. */
                    esp_rom_delay_us((uint32_t)wait_us);
                }
                /* else: < 5 µs, just fall through and send now. */
            }
            s_next_send_us += (1000000ULL / pps);
            /* Catch-up cap: never schedule into the past. */
            now_us = (uint64_t)esp_timer_get_time();
            if (s_next_send_us < now_us) s_next_send_us = now_us;

            /* WDT feed: esp_rom_delay_us is a tight busy-wait so the IDLE
             * task on the same core can't run. Without IDLE running, the
             * task watchdog fires after 5 s. Every ~100 sends (≈ 100 ms at
             * 1 kHz, 500 ms at 200 Hz) we force a real sleep of 1 tick to
             * let IDLE reset the WDT. Cost: a few percent throughput. */
            if (++s_idle_feed_cnt >= 100u) {
                s_idle_feed_cnt = 0;
                vTaskDelay(1);
                /* Reset throttle baseline after the forced sleep so we
                 * don't burst-catch-up. */
                s_next_send_us = (uint64_t)esp_timer_get_time();
            }
        }
#endif
#if BENCH_TP_MODE_DRIVER
        /* Mode 1: direct call, no queue. Counter increments on OK. */
        if (g_wan_handle == NULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        wan_comm_status_t st = wan_comm_send_data(g_wan_handle, s_inner_buf,
                                                   (uint16_t)BENCH_TP_INNER_LEN);
        bool ok = (st == WAN_COMM_OK);
        if (ok) {
            portENTER_CRITICAL(&s_mux);
            s_tx_pkt++;
            s_tx_b += BENCH_TP_PAYLOAD_LEN;
            portEXIT_CRITICAL(&s_mux);
        } else {
            portENTER_CRITICAL(&s_mux);
            s_tx_drop++;
            portEXIT_CRITICAL(&s_mux);
            taskYIELD();
        }
#elif BENCH_TP_MODE_PROD_REAL
        /* Mode 2: post via the real uplink queue. TX counter increments
         * inside the dispatcher's HANDLER_BENCH branch on send OK. Use the
         * non-blocking try_enqueue so a tight-loop sender does not stall on
         * a full queue. */
        bool ok = mcu_wan_try_enqueue_uplink(HANDLER_BENCH,
                                             s_tx_buf,
                                             (uint16_t)BENCH_TP_PAYLOAD_LEN);
        if (!ok) {
            portENTER_CRITICAL(&s_mux);
            s_tx_drop++;
            portEXIT_CRITICAL(&s_mux);
        }
        taskYIELD();
#else
#  error "BENCH_THROUGHPUT_ENABLE must be 0, 1, or 2"
#endif
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

#if BENCH_TP_RAMP_ENABLE
        const uint32_t rate_pps = bench_throughput_current_pps();
        ESP_LOGI(TAG,
                 "[BENCH_TP %dms] rate=%lu pps step=%lu/%u "
                 "TX(LAN->WAN): pkt=%lu b=%lu pps=%.1f kbps=%.1f drop=%lu | "
                 "RX(WAN->LAN): pkt=%lu b=%lu pps=%.1f kbps=%.1f",
                 BENCH_TP_REPORT_INTERVAL_MS,
                 (unsigned long)rate_pps,
                 (unsigned long)s_ramp_step + 1u, (unsigned)s_ramp_n,
                 (unsigned long)tx_pkt, (unsigned long)tx_b,
                 tx_pps, tx_kbps, (unsigned long)tx_drop,
                 (unsigned long)rx_pkt, (unsigned long)rx_b,
                 rx_pps, rx_kbps);

#if !BENCH_TP_RAMP_LOOP
        /* Auto-stop (one-shot mode only): producer outran dispatcher for
         * N consecutive windows ⇒ stop ramp. Disabled in LOOP mode. */
        const uint32_t attempts = tx_pkt + tx_drop;
        if (attempts > 0) {
            const float drop_pct = 100.0f * (float)tx_drop / (float)attempts;
            if (drop_pct > BENCH_TP_RAMP_AUTOSTOP_DROP_PCT) {
                s_autostop_hits++;
                if (s_autostop_hits >= BENCH_TP_RAMP_AUTOSTOP_WINDOWS) {
                    ESP_LOGW(TAG, "Auto-stop: drop_pct=%.2f for %lu windows "
                                  "at rate=%lu pps — stopping ramp",
                             drop_pct, (unsigned long)s_autostop_hits,
                             (unsigned long)rate_pps);
                    s_running = false;
                    break;
                }
            } else {
                s_autostop_hits = 0;
            }
        }
#endif

        /* Advance step if dwell elapsed. In LOOP mode wrap back to step 0
         * so the ramp cycles forever; otherwise hold at top step. */
        const uint32_t now_ms = (uint32_t)esp_log_timestamp();
        if (now_ms - s_ramp_step_start >= BENCH_TP_RAMP_STEP_MS) {
#if BENCH_TP_RAMP_LOOP
            s_ramp_step = (s_ramp_step + 1u) % s_ramp_n;
            s_ramp_step_start = now_ms;
            s_next_send_us = 0; /* reset throttle baseline at step boundary */
            ESP_LOGI(TAG, "Ramp step → %lu pps (loop)",
                     (unsigned long)bench_throughput_current_pps());
#else
            if (s_ramp_step + 1u < s_ramp_n) {
                s_ramp_step++;
                s_ramp_step_start = now_ms;
                s_next_send_us = 0;
                ESP_LOGI(TAG, "Ramp step → %lu pps",
                         (unsigned long)bench_throughput_current_pps());
            } else {
                ESP_LOGI(TAG, "Ramp reached top step (%lu pps), holding",
                         (unsigned long)bench_throughput_current_pps());
                s_ramp_step_start = now_ms; /* avoid log spam */
            }
#endif
        }
#else
        ESP_LOGI(TAG,
                 "[BENCH_TP %dms] "
                 "TX(LAN->WAN): pkt=%lu b=%lu pps=%.1f kbps=%.1f drop=%lu | "
                 "RX(WAN->LAN): pkt=%lu b=%lu pps=%.1f kbps=%.1f",
                 BENCH_TP_REPORT_INTERVAL_MS,
                 (unsigned long)tx_pkt, (unsigned long)tx_b,
                 tx_pps, tx_kbps, (unsigned long)tx_drop,
                 (unsigned long)rx_pkt, (unsigned long)rx_b,
                 rx_pps, rx_kbps);
#endif

        /* Framing diagnostics — cumulative since boot. */
        if (g_wan_handle) {
            uint32_t fok = 0, hcrc = 0, pcrc = 0, resync = 0, gap = 0;
            wan_comm_get_framing_stats(g_wan_handle, &fok, &hcrc, &pcrc,
                                       &resync, &gap);
            ESP_LOGI(TAG,
                     "[BENCH_TP frame] rx_ok=%lu hdr_crc_fail=%lu "
                     "pay_crc_fail=%lu resync_bytes=%lu seq_gap=%lu",
                     (unsigned long)fok, (unsigned long)hcrc,
                     (unsigned long)pcrc, (unsigned long)resync,
                     (unsigned long)gap);
        }
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

#if BENCH_TP_DIRECT_SEND
    /* Pre-build inner DT: [BNC][len BE][rtc 19B][payload]. */
    s_inner_buf = (uint8_t *)heap_caps_malloc(BENCH_TP_INNER_LEN,
                                              MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_inner_buf) {
        ESP_LOGE(TAG, "Failed to allocate inner buf (%u bytes)",
                 (unsigned)BENCH_TP_INNER_LEN);
        heap_caps_free(s_tx_buf); s_tx_buf = NULL;
        return ESP_ERR_NO_MEM;
    }
    s_inner_buf[0] = 'B';
    s_inner_buf[1] = 'N';
    s_inner_buf[2] = 'C';
    uint16_t data_len = 19u + BENCH_TP_PAYLOAD_LEN;
    s_inner_buf[3] = (uint8_t)((data_len >> 8) & 0xFFu);
    s_inner_buf[4] = (uint8_t)(data_len & 0xFFu);
    memcpy(&s_inner_buf[5],  "00/00/0000-00:00:00", 19);
    memset(&s_inner_buf[24], 0xAA, BENCH_TP_PAYLOAD_LEN);
#endif

    s_running = true;

    /* Hook into full-duplex flush for WAN→LAN BNC frame counting. */
    if (g_wan_handle != NULL) {
        wan_comm_register_rx_frame_callback(g_wan_handle, bench_tp_rx_cb, NULL);
    }

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
    if (g_wan_handle != NULL) {
        wan_comm_register_rx_frame_callback(g_wan_handle, NULL, NULL);
    }
    /* Tasks detect s_running==false on their next iteration and self-delete */
    ESP_LOGI(TAG, "Stop requested — tasks will self-delete");
}

#else /* BENCH_THROUGHPUT_ENABLE == 0 */

void bench_throughput_count_rx(uint32_t bytes)  { (void)bytes; }
void bench_throughput_count_tx(uint32_t bytes)  { (void)bytes; }
void bench_throughput_count_tx_drop(void)        {}
uint32_t bench_throughput_current_pps(void)      { return 0; }

esp_err_t bench_throughput_start(void) {
    ESP_LOGI(TAG, "Inter-MCU throughput benchmark disabled (BENCH_THROUGHPUT_ENABLE=0)");
    return ESP_OK;
}

void bench_throughput_stop(void) {}

#endif /* BENCH_THROUGHPUT_ENABLE */
