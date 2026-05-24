/**
 * @file bench_e2e.h
 * @brief Internal end-to-end latency benchmark — LAN side gate.
 *
 * When enabled, each LAN module handler (BLE/LoRa/Zigbee/RS485) emits one
 * `[E2E_LAN]` log per uplink frame measuring the time from module RX
 * callback to inter-MCU SPI uplink dispatch (all in LAN-MCU clock domain).
 *
 * Pair this with `BENCH_E2E_WAN_ENABLE` on the WAN MCU (Section 2 Part B)
 * to get full internal E2E breakdown. SPI bridge cost itself is measured
 * separately by `bench_throughput` (Section 1).
 *
 * When disabled, all measurement code (timestamp diff + log call) is
 * compiled out — zero runtime cost.
 */

#ifndef BENCH_E2E_H
#define BENCH_E2E_H

#include "esp_log.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Master switch for LAN-side internal E2E latency logging.
 *   0 = OFF. Measurement code is fully compiled out.
 *   1 = ON.  Each successful uplink dispatch emits one `[E2E_LAN]` line.
 */
#define BENCH_E2E_LAN_ENABLE 1

#if BENCH_E2E_LAN_ENABLE
  /** Emit one `[E2E_LAN]` log line. Same args as `ESP_LOGI` body. */
  #define BENCH_E2E_LAN_LOG(fmt, ...) \
      ESP_LOGI("E2E_LAN", fmt, ##__VA_ARGS__)
#else
  #define BENCH_E2E_LAN_LOG(fmt, ...) ((void)0)
#endif

#ifdef __cplusplus
}
#endif

#endif /* BENCH_E2E_H */
