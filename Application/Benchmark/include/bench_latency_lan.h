#ifndef BENCH_LATENCY_LAN_H_
#define BENCH_LATENCY_LAN_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* §5 — End-to-end latency benchmark, LAN side (simple variant).
 *
 * Flow:
 *   1. clock_sync_lan locks LAN µs onto WAN µs via the 1 Hz RTC packet.
 *   2. RS485 handler, when bench mode is ON, hands its RX buffer to
 *      bench_latency_lan_send(). The helper prepends [T1 8B][seq 4B] and
 *      enqueues the frame as HANDLER_LAT through the normal uplink path.
 *   3. WAN parses HANDLER_LAT, ships the inner payload to the TCP sink,
 *      stamps T2 right after send(), and logs T2 − T1.
 *
 * No worker task, no clone, no observer — the timestamping happens on the
 * same RS485 RX context that already calls mcu_wan_enqueue_uplink().
 */

#ifndef BENCH_LATENCY_LAN_ENABLE
#define BENCH_LATENCY_LAN_ENABLE 0
#endif

/* Hard cap on payload we will stamp + ship. */
#ifndef BENCH_LATENCY_LAN_PAYLOAD_MAX
#define BENCH_LATENCY_LAN_PAYLOAD_MAX 256
#endif

/* Stamp and enqueue a single RS485 RX buffer toward WAN as HANDLER_LAT.
 * Returns true if enqueued, false if dropped (queue full or bench disabled).
 * Safe to call from the RS485 handler task. */
bool bench_latency_lan_send(const uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* BENCH_LATENCY_LAN_H_ */
