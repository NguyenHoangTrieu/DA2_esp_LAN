/**
 * @file bench_throughput.h
 * @brief Inter-MCU SPI throughput benchmark — LAN side (SPI Master).
 *
 * Sender + reporter tasks. WAN MCU must run bench_throughput_wan to count
 * incoming BNC frames and (optionally) load the WAN→LAN template/refresh.
 */

#ifndef BENCH_THROUGHPUT_H
#define BENCH_THROUGHPUT_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Master on/off + mode switch.
 *   0 = OFF (all public APIs become no-ops).
 *   1 = DRIVER. Sender bypasses the uplink queue and calls
 *       wan_comm_send_data() directly. Slave loads a static template once.
 *       Measures the SPI transport/framing ceiling.
 *   2 = PRODUCTION-REAL. Sender posts via mcu_wan_try_enqueue_uplink, items
 *       walk the full handler pipeline (queue → dispatcher → framing → SPI).
 *       Measures sustained throughput under the real production path.
 */
#define BENCH_THROUGHPUT_ENABLE 0

#define BENCH_TP_MODE_OFF        (BENCH_THROUGHPUT_ENABLE == 0)
#define BENCH_TP_MODE_DRIVER     (BENCH_THROUGHPUT_ENABLE == 1)
#define BENCH_TP_MODE_PROD_REAL  (BENCH_THROUGHPUT_ENABLE == 2)

#define BENCH_TP_REPORT_INTERVAL_MS 2000

/**
 * @brief Ramp-rate sub-mode for §4 (LAN→WAN lossless ceiling).
 *   0 = OFF — sender runs flood loop (current §1 behaviour).
 *   1 = ON  — sender throttles to RATE_STEPS[step], advancing every
 *             BENCH_TP_RAMP_STEP_MS. Reporter logs the active rate so the
 *             operator can correlate LAN tx with WAN rx at each step.
 *
 * Use with BENCH_THROUGHPUT_ENABLE=2 (production path) for §4.
 */
#define BENCH_TP_RAMP_ENABLE  0
#define BENCH_TP_RAMP_STEP_MS 10000

/**
 * @brief Continuous-loop mode. When 1, after the top step the ramp wraps
 *        back to step 0 and keeps cycling indefinitely — auto-stop is
 *        suppressed so the operator can watch drops appear/disappear as
 *        the rate climbs. Set to 0 if you want one-shot ramp + auto-stop.
 */
#define BENCH_TP_RAMP_LOOP    1

/** Auto-stop heuristic (only used when BENCH_TP_RAMP_LOOP == 0). */
#define BENCH_TP_RAMP_AUTOSTOP_DROP_PCT 1.0f
#define BENCH_TP_RAMP_AUTOSTOP_WINDOWS  3

/** Start sender + reporter. Call after mcu_wan_handler_start(). */
esp_err_t bench_throughput_start(void);

/** Stop both tasks. */
void bench_throughput_stop(void);

/** Increment RX counter (WAN→LAN). Called from mcu_wan_handler_downlink. */
void bench_throughput_count_rx(uint32_t bytes);

/** Increment TX counter — Mode 2 only, from uplink dispatcher on send OK. */
void bench_throughput_count_tx(uint32_t bytes);

/** Increment TX drop counter (uplink queue full). */
void bench_throughput_count_tx_drop(void);

/** Current ramp rate (pps). Returns 0 when ramp disabled or stopped. */
uint32_t bench_throughput_current_pps(void);

#ifdef __cplusplus
}
#endif

#endif /* BENCH_THROUGHPUT_H */
