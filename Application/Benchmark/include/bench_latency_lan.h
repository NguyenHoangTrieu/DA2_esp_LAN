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
#define BENCH_LATENCY_LAN_ENABLE 0   /* §5 OFF (production). Set 1 to measure. */
#endif

/* ---------------------------------------------------------------------------
 * §5 ingress source selector
 *
 * The downstream path (T1 stamp → uplink queue → dispatcher → SPI framing →
 * WAN egress → T2) is IDENTICAL for both sources. Only the lane that feeds
 * bytes into the LAN MCU differs:
 *
 *   SRC_RS485 : PC → USB-RS485 adapter → RS485 lane. The rs485_handler task
 *               stamps T1 inline (legacy §5 path). Limited to ~tens of kbps,
 *               so it can only measure the LOW-LOAD (idle) latency floor.
 *
 *   SRC_USB   : ESP32-S3 rig → USB CDC lane. A dedicated ingress task frames
 *               the rig's fixed-size packets (4-byte LE seq header), stamps T1
 *               and ships them as HANDLER_LAT. USB CDC reaches multi-Mbps, so
 *               the same measurement can be repeated under HIGH LOAD to expose
 *               queueing latency. The downstream path is byte-for-byte the
 *               production path — see bench_latency_lan.c for the proof.
 * ------------------------------------------------------------------------- */
#define BENCH_LATENCY_LAN_SRC_RS485 0
#define BENCH_LATENCY_LAN_SRC_USB   1

#ifndef BENCH_LATENCY_LAN_SRC
#define BENCH_LATENCY_LAN_SRC BENCH_LATENCY_LAN_SRC_RS485   /* default source */
#endif

/* Hard cap on payload we will stamp + ship. */
#ifndef BENCH_LATENCY_LAN_PAYLOAD_MAX
#define BENCH_LATENCY_LAN_PAYLOAD_MAX 256
#endif

/* --- USB ingress source (§5, SRC_USB) configuration ----------------------- */

/* Fixed packet size emitted by the rig in latency mode. MUST equal the rig's
 * PKT_SIZE (bench_lane_rig.ino, RIG_LATENCY_E2E build).
 *
 * Wire layout of one rig packet (USB CDC is a delimiter-less byte stream, so a
 * start-of-frame magic is REQUIRED to lock/relock alignment — fixed-size
 * framing alone drifts forever after the first mid-packet byte):
 *
 *   [0..1]   SOF magic  = 0x55 0xAA
 *   [2..5]   seq         (uint32 LE, rig increments by 1 per packet)
 *   [6..N-1] payload pattern, byte k = (k & 0xFF)
 *
 * The ingress task scans for SOF, then reads PKT bytes, re-validating SOF on
 * every frame so a single dropped USB byte costs at most one packet, not sync.
 */
#ifndef BENCH_LATENCY_USB_PKT
#define BENCH_LATENCY_USB_PKT 64
#endif

#define BENCH_LATENCY_USB_SOF0    0x55
#define BENCH_LATENCY_USB_SOF1    0xAA
#define BENCH_LATENCY_USB_SEQ_OFF 2

/* Which module stack/lane the rig is wired to (USB host port). */
#ifndef BENCH_LATENCY_USB_STACK_ID
#define BENCH_LATENCY_USB_STACK_ID 0
#endif

/* Raw read chunk + timeout for the USB ingress loop. */
#ifndef BENCH_LATENCY_USB_READ_CHUNK
#define BENCH_LATENCY_USB_READ_CHUNK 512
#endif
#ifndef BENCH_LATENCY_USB_READ_TIMEOUT_MS
#define BENCH_LATENCY_USB_READ_TIMEOUT_MS 50
#endif

/* Diagnostic report interval for the USB ingress task (framed pkts, ingress
 * loss from rig-seq gaps, enqueue drops, misalignment). */
#ifndef BENCH_LATENCY_USB_REPORT_INTERVAL_MS
#define BENCH_LATENCY_USB_REPORT_INTERVAL_MS 2000
#endif

/* Stamp and enqueue a single RX buffer toward WAN as HANDLER_LAT.
 * Returns true if enqueued, false if dropped (queue full or bench disabled).
 * Safe to call from the RS485 handler task or the USB ingress task. */
bool bench_latency_lan_send(const uint8_t *data, uint16_t len);

/* Start the USB-rig ingress task (§5, SRC_USB only). No-op stub otherwise so
 * the call site in app_main never needs guarding. Call after
 * module_config_controller_init(). */
void bench_latency_usb_start(void);

#ifdef __cplusplus
}
#endif

#endif /* BENCH_LATENCY_LAN_H_ */
