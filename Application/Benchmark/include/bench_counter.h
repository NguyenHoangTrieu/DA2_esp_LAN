/**
 * @file bench_counter.h
 * @brief Lightweight per-protocol benchmark counters for BLE GATT, Zigbee, LoRa.
 *
 * Each handler increments counters when a packet is forwarded upstream or dropped.
 * A periodic task reports a JSON snapshot every BENCH_REPORT_INTERVAL_MS via the
 * existing BLE GATT uplink path so it reaches ThingsBoard as part of "data" telemetry.
 *
 * Wire format sent upstream:
 *   BENCH:{"ble_pkt":N,"ble_b":N,"ble_drop":N,"zb_pkt":N,"zb_b":N,"zb_drop":N,
 *           "lr_pkt":N,"lr_b":N,"lr_drop":N,"ms":2000}
 *
 * The monitor widget parses this and shows firmware-reported kbps alongside
 * dashboard-estimated kbps.
 */

#ifndef BENCH_COUNTER_H
#define BENCH_COUNTER_H

#include "esp_err.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Reporting interval in milliseconds. */
#define BENCH_REPORT_INTERVAL_MS 2000

/**
 * @brief Master switch for benchmark counters/reporter.
 *        1 = enable benchmark counting + periodic BENCH log.
 *        0 = compile benchmark API as no-op (similar to disabling logs).
 */
#define BENCH_ENABLE 1

/**
 * @brief Set to 1 to suppress high-frequency data-path log spam during
 *        benchmark runs (NOTIFY dumps, "Transmit attempt", "ACK received",
 *        "Uplink queued/processing", listener RX dumps, queue-full warnings).
 *        Set to 0 to restore full verbose output for debugging.
 */
#define BENCH_QUIET_LOG  0

/**
 * @brief Start the periodic benchmark reporter task.
 *        Call AFTER mcu_wan_handler_start().
 */
esp_err_t bench_task_start(void);

/**
 * @brief Stop the benchmark reporter task.
 */
void bench_task_stop(void);

/* ---- Increment counters (ISR-safe via portENTER_CRITICAL) ---- */

/** Call when a BLE GATT NOTIFY message is successfully forwarded upstream. */
void bench_count_ble(uint16_t payload_bytes);

/** Call when a BLE GATT NOTIFY/INDICATE is received from peer (raw DLE side). */
void bench_count_ble_rx(uint16_t payload_bytes);

/** Call when a Zigbee RPT:/ATTRREPORT event is successfully forwarded upstream. */
void bench_count_zb_fwd(uint16_t payload_bytes);

/** Call when a Zigbee benchmark event is received from the module before uplink formatting. */
void bench_count_zb_rx(uint16_t payload_bytes);

/** Call when LoRa listener data chunk is successfully forwarded upstream. */
void bench_count_lr_fwd(uint16_t payload_bytes);

/** Call when LoRa listener data chunk is received from module ingress. */
void bench_count_lr_rx(uint16_t payload_bytes);

/** Call when a BLE GATT uplink packet is dropped (queue full). */
void bench_count_ble_drop(void);

/** Call when a Zigbee uplink packet is dropped. */
void bench_count_zb_drop(void);

/** Call when a LoRa uplink packet is dropped. */
void bench_count_lr_drop(void);

#ifdef __cplusplus
}
#endif

#endif /* BENCH_COUNTER_H */
