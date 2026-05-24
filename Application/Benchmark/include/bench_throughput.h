/**
 * @file bench_throughput.h
 * @brief Inter-MCU SPI throughput benchmark — LAN side (SPI Master).
 *
 * When BENCH_THROUGHPUT_ENABLE = 1 this module creates two tasks:
 *
 *   bench_tp_sender  — priority 2, floods mcu_wan_enqueue_uplink() with
 *                      INTER_MCU_PAYLOAD_MAX_LEN-byte BNC frames as fast as
 *                      the uplink queue allows, measuring LAN→WAN throughput.
 *
 *   bench_tp_reporter — priority 2, prints a throughput summary every
 *                       BENCH_TP_REPORT_INTERVAL_MS via ESP_LOGI.
 *
 * The WAN MCU (DA2_esp) must have its own bench_throughput_wan module
 * compiled and started so that:
 *  - It counts BNC frames arriving from LAN (LAN→WAN direction).
 *  - It spam-sends BNC frames toward LAN (WAN→LAN direction) via
 *    mcu_lan_enqueue_downlink().
 *
 * Received WAN→LAN BNC frames are counted by bench_throughput_count_rx(),
 * which is called by mcu_wan_handler_downlink when it sees a DT/BNC frame.
 *
 * Master switch: set BENCH_THROUGHPUT_ENABLE to 1 in this header (or via
 * compiler -D flag) to compile the real implementation; 0 compiles all
 * public functions as no-ops so zero code/data is added to the production
 * binary.
 */

#ifndef BENCH_THROUGHPUT_H
#define BENCH_THROUGHPUT_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Master on/off switch for the inter-MCU throughput benchmark.
 *
 *   0 = OFF. All public functions are no-ops, zero production overhead.
 *
 *   1 = DRIVER mode (formerly BENCH_TP_DIRECT_SEND=1).
 *       Sender bypasses mcu_wan_enqueue_uplink/uplink_processor_task and
 *       calls wan_comm_send_data() directly. Slave loads a static template
 *       into tx_buffer once (no refresh). Measures the *ceiling* of the
 *       SPI transport/framing layer in isolation; **not** representative of
 *       what real handler traffic achieves.
 *
 *   2 = PRODUCTION-REAL mode.
 *       Sender posts via mcu_wan_enqueue_uplink(HANDLER_BENCH, ...) — the
 *       same queue real handlers use. The HANDLER_BENCH branch in
 *       uplink_handler_task is forced to take the ACK-gated send_data_to_wan
 *       path (instead of the fire-and-forget fast path). WAN side runs a
 *       refresh task that periodically calls mcu_lan_enqueue_downlink so the
 *       slave exercises queue + memcpy + downlink_handler_task +
 *       lan_comm_load_tx_data on every frame. Measures sustained
 *       application-layer throughput under the real production code path.
 *
 * Modes 1 and 2 cannot be combined (a single run measures one or the other).
 * Mode 2 yields a smaller number than Mode 1 — that gap is the cost of the
 * handler pipeline (queue, mutex, ACK round-trip), which is exactly what we
 * want to characterise.
 */
#define BENCH_THROUGHPUT_ENABLE 2

/* Derived flags — do NOT edit, computed from BENCH_THROUGHPUT_ENABLE. */
#define BENCH_TP_MODE_OFF        (BENCH_THROUGHPUT_ENABLE == 0)
#define BENCH_TP_MODE_DRIVER     (BENCH_THROUGHPUT_ENABLE == 1)
#define BENCH_TP_MODE_PROD_REAL  (BENCH_THROUGHPUT_ENABLE == 2)

/** Reporting interval in milliseconds. */
#define BENCH_TP_REPORT_INTERVAL_MS 2000

/**
 * @brief Start sender and reporter tasks.
 *        Call AFTER mcu_wan_handler_start().
 */
esp_err_t bench_throughput_start(void);

/**
 * @brief Stop both tasks (sets running flag to false; tasks self-delete).
 */
void bench_throughput_stop(void);

/**
 * @brief Increment the RX byte counter (WAN→LAN direction).
 *        Call from mcu_wan_handler_downlink when a DT/BNC frame arrives.
 *
 * @param bytes Number of payload bytes in the received frame.
 */
void bench_throughput_count_rx(uint32_t bytes);

/**
 * @brief Increment the TX byte counter — Mode 2 (PROD_REAL) only.
 *
 * Called by uplink_handler_task's HANDLER_BENCH branch AFTER the slave has
 * acknowledged the frame via send_data_to_wan(). This produces an honest
 * end-to-end TX number (post-ACK), unlike Mode 1 where the sender task
 * counts at wan_comm_send_data() return time (pre-wire).
 *
 * @param bytes Inner payload size (typically BENCH_TP_PAYLOAD_LEN).
 */
void bench_throughput_count_tx(uint32_t bytes);

/**
 * @brief Increment TX drop counter (uplink queue was full).
 *        Call from the sender task when mcu_wan_enqueue_uplink() returns false.
 */
void bench_throughput_count_tx_drop(void);

#ifdef __cplusplus
}
#endif

#endif /* BENCH_THROUGHPUT_H */
