#ifndef CLOCK_SYNC_LAN_H_
#define CLOCK_SYNC_LAN_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Cross-MCU clock sync — LAN side (simple variant).
 *
 * Reuses the existing 1 Hz RTC packet (WAN → LAN). WAN stamps wan_us when
 * it processes the [R][T] request (within a few ms of receiving it). LAN
 * stamps lan_ref_us RIGHT BEFORE issuing [R][T]. On every accepted tick:
 *
 *     offset_us = wan_us − lan_ref_us
 *
 * is written directly into a counter — no EMA, no median, no warmup filter.
 *
 * Anchoring to lan_ref_us (= send-time) instead of lan_recv_us makes the
 * residual bias = WAN's processing delay (~1-3 ms) instead of the master's
 * vTaskDelay between sending [R][T] and reading the response (~150 ms),
 * which would otherwise inflate every measured latency by the same amount.
 */

/* Called by the RTC-response parser on each successful RTC frame.
 * lan_ref_us should be esp_timer_get_time() captured immediately BEFORE
 * the [R][T] command is sent over SPI. */
void     clock_sync_lan_update(uint64_t wan_us, uint64_t lan_ref_us);

/* Convert a LAN-local esp_timer reading into the WAN (master) clock domain.
 * Returns local_us unchanged if no RTC tick has been seen yet. */
uint64_t clock_sync_lan_to_master_us(uint64_t local_us);

/* Convenience: WAN-domain "now". */
uint64_t clock_sync_lan_now_master_us(void);

#ifdef __cplusplus
}
#endif

#endif /* CLOCK_SYNC_LAN_H_ */
