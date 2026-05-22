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
 *        1 = compile real sender + reporter tasks.
 *        0 = all functions compiled as no-ops (zero production overhead).
 */
#define BENCH_THROUGHPUT_ENABLE 1

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
 * @brief Increment TX drop counter (uplink queue was full).
 *        Call from the sender task when mcu_wan_enqueue_uplink() returns false.
 */
void bench_throughput_count_tx_drop(void);

#ifdef __cplusplus
}
#endif

#endif /* BENCH_THROUGHPUT_H */
