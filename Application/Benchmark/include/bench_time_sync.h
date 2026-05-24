/**
 * @file bench_time_sync.h
 * @brief Cross-MCU time synchronisation at µs precision.
 *
 * Goal: any moment in time captured on the LAN MCU can be translated to the
 * equivalent `esp_timer_get_time()` reading on the WAN MCU (and vice-versa)
 * with sub-millisecond accuracy.
 *
 * Method: PTP/NTP-style four-timestamp round-trip over SPI.
 *
 *     LAN MCU (master)                       WAN MCU (slave)
 *     ----------------                       ----------------
 *     T1 = esp_timer_get_time()
 *      └─ SPI: tsync_request_t ────────────► T2 = esp_timer_get_time()
 *                                            (RX immediately on slave ISR)
 *                                                ...
 *                                            T3 = esp_timer_get_time()
 *      ◄────────── SPI: tsync_response_t ────┘  (right before reply load)
 *     T4 = esp_timer_get_time()
 *
 *     offset_us = ((T2 - T1) + (T3 - T4)) / 2     // wan_us − lan_us
 *     rtt_us    = (T4 - T1) - (T3 - T2)            // pure wire round-trip
 *
 * After at least one successful round, `lan_to_wan_us()` returns an estimate
 * of the WAN clock for any LAN-clock timestamp. Drift between syncs is bounded
 * by XTAL spec (~20 ppm) × interval (5 s default) ≈ 100 µs.
 *
 * The module is gated by `BENCH_TIME_SYNC_ENABLE` (default OFF). When off, the
 * task does not start and SPI carries no extra traffic.
 */

#ifndef BENCH_TIME_SYNC_H
#define BENCH_TIME_SYNC_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Master switch. 0 = OFF (task not started, all APIs return safe defaults). */
#ifndef BENCH_TIME_SYNC_ENABLE
#define BENCH_TIME_SYNC_ENABLE 1
#endif

/* Round-trip cadence (ms). 5000 ms × ±20 ppm XTAL ⇒ ~100 µs drift bound. */
#ifndef BENCH_TIME_SYNC_INTERVAL_MS
#define BENCH_TIME_SYNC_INTERVAL_MS 5000
#endif

/* Outlier filter: skip update when current RTT exceeds median × this factor. */
#ifndef BENCH_TIME_SYNC_RTT_OUTLIER_K
#define BENCH_TIME_SYNC_RTT_OUTLIER_K 3
#endif

typedef struct {
  int64_t  offset_us;     /* wan_us − lan_us (signed) */
  int64_t  last_rtt_us;   /* pure wire round-trip from last successful sync */
  int64_t  median_rtt_us; /* median over recent samples; baseline for outlier */
  uint32_t sync_age_ms;   /* milliseconds since last successful sync */
  uint32_t sync_count;    /* total successful rounds since boot */
  uint32_t sync_fail;     /* total failed/dropped rounds since boot */
  bool     synced;        /* true once at least one round succeeded */
} bench_time_sync_state_t;

/**
 * @brief Initialise the module. Idempotent. Starts the periodic task only on
 *        the master MCU (LAN). On the slave (WAN) this only sets up state for
 *        responding to incoming TSYNC_REQ frames.
 */
esp_err_t bench_time_sync_init(void);

/**
 * @brief Convert a local µs timestamp to the peer MCU's clock estimate.
 *        LAN side  → wan_us. WAN side → lan_us.
 *        Returns `local_us` unchanged when sync state has never converged.
 */
int64_t bench_time_sync_to_peer_us(int64_t local_us);

/**
 * @brief Convert a peer-clock µs timestamp to the local clock.
 */
int64_t bench_time_sync_from_peer_us(int64_t peer_us);

/** Snapshot of internal state for diagnostics / [TSYNC] log line. */
void bench_time_sync_get_state(bench_time_sync_state_t *out);

/**
 * @brief Manually trigger one sync round (LAN side only). Returns ESP_OK on
 *        successful round and updates state; ESP_FAIL on transport error.
 *        Normally called by the internal task — exposed for testing.
 */
esp_err_t bench_time_sync_request_round(void);

/**
 * @brief (WAN side only) Slave hook: parse an incoming TSYNC_REQ frame and
 *        populate `out_rsp` with t2/t3 timestamps. To be called from the
 *        slave SPI parser when it sees a CF frame with cmd = 0x09.
 *        `t2_us` should be captured by the caller right after frame RX and
 *        passed in via `now_t2_us` — this function then snapshots t3.
 *        Returns true if the response was filled and the slave should send
 *        it on the next SPI exchange.
 */
bool bench_time_sync_slave_build_response(const uint8_t *req_payload,
                                          uint16_t req_len,
                                          int64_t now_t2_us,
                                          uint8_t *out_rsp,
                                          uint16_t out_cap,
                                          uint16_t *out_len);

#ifdef __cplusplus
}
#endif

#endif /* BENCH_TIME_SYNC_H */
