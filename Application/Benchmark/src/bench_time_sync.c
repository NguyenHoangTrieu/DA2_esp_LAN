/**
 * @file bench_time_sync.c  (LAN MCU — master)
 * @brief PTP-style cross-MCU clock sync. Runs a background task that issues
 *        a TSYNC round every BENCH_TIME_SYNC_INTERVAL_MS, updates the global
 *        offset, and exposes lan↔wan time conversion helpers.
 *
 * Wire-level SPI integration is performed by the caller — this module only
 * builds the request bytes and parses the response. See bench_tsync_dispatch()
 * in mcu_wan_handler_uplink.c (TS-2) for the actual SPI roundtrip.
 */

#include "bench_time_sync.h"
#include "frame_types.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "TSYNC";

#if BENCH_TIME_SYNC_ENABLE

/* Forward declaration of the SPI roundtrip helper. Implemented in TS-2 inside
 * mcu_wan_handler_uplink.c so we can reuse the existing wan_comm framer. The
 * helper:
 *   1. Captures T1.
 *   2. Sends [CF][TSYNC_REQ][T1] over SPI.
 *   3. Polls the slave for a tsync_response_t (using the existing DQ path).
 *   4. Captures T4 the moment the response is fully received.
 *   5. Returns ESP_OK and fills out_t1/t2/t3/t4. ESP_FAIL on timeout/parse error.
 */
esp_err_t bench_tsync_spi_roundtrip(int64_t *out_t1_us, int64_t *out_t2_us,
                                    int64_t *out_t3_us, int64_t *out_t4_us);

/* ── State ─────────────────────────────────────────────────────────────── */

#define TSYNC_RTT_HISTORY 8

static SemaphoreHandle_t s_state_mutex = NULL;
static int64_t s_offset_us = 0;
static int64_t s_last_rtt_us = 0;
static int64_t s_median_rtt_us = 0;
static int64_t s_last_sync_us = 0;
static uint32_t s_sync_count = 0;
static uint32_t s_sync_fail = 0;
static bool s_synced = false;

/* Recent RTT samples for median-based outlier rejection. */
static int64_t s_rtt_history[TSYNC_RTT_HISTORY] = {0};
static uint8_t s_rtt_history_idx = 0;
static uint8_t s_rtt_history_n = 0;

static TaskHandle_t s_tsync_task = NULL;
static bool s_task_running = false;

/* ── Helpers ───────────────────────────────────────────────────────────── */

static int64_t _median_rtt_locked(void) {
  if (s_rtt_history_n == 0) return 0;
  int64_t tmp[TSYNC_RTT_HISTORY];
  uint8_t n = s_rtt_history_n;
  for (uint8_t i = 0; i < n; i++) tmp[i] = s_rtt_history[i];
  /* Simple insertion sort — small N. */
  for (uint8_t i = 1; i < n; i++) {
    int64_t v = tmp[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && tmp[j] > v) { tmp[j + 1] = tmp[j]; j--; }
    tmp[j + 1] = v;
  }
  return tmp[n / 2];
}

static void _push_rtt_locked(int64_t rtt_us) {
  s_rtt_history[s_rtt_history_idx] = rtt_us;
  s_rtt_history_idx = (uint8_t)((s_rtt_history_idx + 1) % TSYNC_RTT_HISTORY);
  if (s_rtt_history_n < TSYNC_RTT_HISTORY) s_rtt_history_n++;
  s_median_rtt_us = _median_rtt_locked();
}

static void _apply_round(int64_t t1, int64_t t2, int64_t t3, int64_t t4) {
  int64_t rtt = (t4 - t1) - (t3 - t2);
  int64_t offset = ((t2 - t1) + (t3 - t4)) / 2;

  if (rtt <= 0) {
    /* Negative/zero RTT ⇒ peer captured timestamps out of order. Reject. */
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_sync_fail++;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGW(TAG, "Reject round: rtt=%lld (must be > 0)", (long long)rtt);
    return;
  }

  xSemaphoreTake(s_state_mutex, portMAX_DELAY);

  /* Outlier filter: skip if RTT is wildly larger than recent median. */
  if (s_rtt_history_n >= 3 && s_median_rtt_us > 0 &&
      rtt > s_median_rtt_us * BENCH_TIME_SYNC_RTT_OUTLIER_K) {
    s_sync_fail++;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGW(TAG, "Reject round: rtt=%lld vs median=%lld (×%d outlier)",
             (long long)rtt, (long long)s_median_rtt_us,
             BENCH_TIME_SYNC_RTT_OUTLIER_K);
    return;
  }

  s_offset_us = offset;
  s_last_rtt_us = rtt;
  s_last_sync_us = esp_timer_get_time();
  s_sync_count++;
  s_synced = true;
  _push_rtt_locked(rtt);
  xSemaphoreGive(s_state_mutex);

  ESP_LOGI(TAG, "round#%lu offset=%lld us rtt=%lld us median=%lld us",
           (unsigned long)s_sync_count, (long long)offset, (long long)rtt,
           (long long)s_median_rtt_us);
}

static void tsync_task(void *arg) {
  (void)arg;
  ESP_LOGI(TAG, "task started, interval=%d ms", BENCH_TIME_SYNC_INTERVAL_MS);
  /* Brief settle so the first round happens after handshake. */
  vTaskDelay(pdMS_TO_TICKS(2000));
  while (s_task_running) {
    int64_t t1, t2, t3, t4;
    esp_err_t st = bench_tsync_spi_roundtrip(&t1, &t2, &t3, &t4);
    if (st == ESP_OK) {
      _apply_round(t1, t2, t3, t4);
    } else {
      xSemaphoreTake(s_state_mutex, portMAX_DELAY);
      s_sync_fail++;
      xSemaphoreGive(s_state_mutex);
      ESP_LOGW(TAG, "SPI roundtrip failed: %s", esp_err_to_name(st));
    }
    vTaskDelay(pdMS_TO_TICKS(BENCH_TIME_SYNC_INTERVAL_MS));
  }
  s_tsync_task = NULL;
  vTaskDelete(NULL);
}

#endif /* BENCH_TIME_SYNC_ENABLE */

/* ── Public API ────────────────────────────────────────────────────────── */

esp_err_t bench_time_sync_init(void) {
#if BENCH_TIME_SYNC_ENABLE
  if (s_state_mutex == NULL) {
    s_state_mutex = xSemaphoreCreateMutex();
    if (s_state_mutex == NULL) return ESP_ERR_NO_MEM;
  }
  if (s_tsync_task == NULL) {
    s_task_running = true;
    BaseType_t ok = xTaskCreatePinnedToCore(tsync_task, "bench_tsync", 4096,
                                            NULL, 6 /* high prio */, &s_tsync_task,
                                            tskNO_AFFINITY);
    if (ok != pdPASS) {
      s_task_running = false;
      return ESP_ERR_NO_MEM;
    }
  }
#endif
  return ESP_OK;
}

int64_t bench_time_sync_to_peer_us(int64_t local_us) {
#if BENCH_TIME_SYNC_ENABLE
  int64_t off = 0;
  bool synced = false;
  if (s_state_mutex) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    off = s_offset_us;
    synced = s_synced;
    xSemaphoreGive(s_state_mutex);
  }
  return synced ? (local_us + off) : local_us;
#else
  return local_us;
#endif
}

int64_t bench_time_sync_from_peer_us(int64_t peer_us) {
#if BENCH_TIME_SYNC_ENABLE
  int64_t off = 0;
  bool synced = false;
  if (s_state_mutex) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    off = s_offset_us;
    synced = s_synced;
    xSemaphoreGive(s_state_mutex);
  }
  return synced ? (peer_us - off) : peer_us;
#else
  return peer_us;
#endif
}

void bench_time_sync_get_state(bench_time_sync_state_t *out) {
  if (!out) return;
  memset(out, 0, sizeof(*out));
#if BENCH_TIME_SYNC_ENABLE
  if (s_state_mutex) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    out->offset_us = s_offset_us;
    out->last_rtt_us = s_last_rtt_us;
    out->median_rtt_us = s_median_rtt_us;
    out->sync_count = s_sync_count;
    out->sync_fail = s_sync_fail;
    out->synced = s_synced;
    int64_t now = esp_timer_get_time();
    out->sync_age_ms = s_synced
        ? (uint32_t)((now - s_last_sync_us) / 1000)
        : 0xFFFFFFFFu;
    xSemaphoreGive(s_state_mutex);
  }
#endif
}

esp_err_t bench_time_sync_request_round(void) {
#if BENCH_TIME_SYNC_ENABLE
  int64_t t1, t2, t3, t4;
  esp_err_t st = bench_tsync_spi_roundtrip(&t1, &t2, &t3, &t4);
  if (st == ESP_OK) _apply_round(t1, t2, t3, t4);
  return st;
#else
  return ESP_ERR_NOT_SUPPORTED;
#endif
}

/* LAN side: not a slave responder. Provide stub. */
bool bench_time_sync_slave_build_response(const uint8_t *req_payload,
                                          uint16_t req_len,
                                          int64_t now_t2_us,
                                          uint8_t *out_rsp,
                                          uint16_t out_cap,
                                          uint16_t *out_len) {
  (void)req_payload; (void)req_len; (void)now_t2_us;
  (void)out_rsp; (void)out_cap; (void)out_len;
  return false;
}
