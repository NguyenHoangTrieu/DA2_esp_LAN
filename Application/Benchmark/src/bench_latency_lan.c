#include "bench_latency_lan.h"

#if BENCH_LATENCY_LAN_ENABLE

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "clock_sync_lan.h"
#include "frame_types.h"
#include "mcu_wan_handler.h"

/* Wire format inside HANDLER_LAT payload:
 *   [0..7]   T1 little-endian uint64 (WAN-domain µs at LAN ingress)
 *   [8..11]  seq little-endian uint32
 *   [12..]   original RS485 RX bytes (capped at PAYLOAD_MAX)
 */
#define LAT_HEADER_BYTES 12

static portMUX_TYPE s_seq_lock = portMUX_INITIALIZER_UNLOCKED;
static uint32_t     s_seq      = 0;

bool bench_latency_lan_send(const uint8_t *data, uint16_t len)
{
    if (!data || len == 0) return false;

    uint16_t plen = (len > BENCH_LATENCY_LAN_PAYLOAD_MAX)
                        ? BENCH_LATENCY_LAN_PAYLOAD_MAX
                        : len;

    uint8_t  buf[LAT_HEADER_BYTES + BENCH_LATENCY_LAN_PAYLOAD_MAX];

    uint64_t t1 = clock_sync_lan_now_master_us();
    for (int i = 0; i < 8; i++) {
        buf[i] = (uint8_t)((t1 >> (8 * i)) & 0xFF);
    }

    portENTER_CRITICAL(&s_seq_lock);
    uint32_t seq = ++s_seq;
    portEXIT_CRITICAL(&s_seq_lock);
    buf[8]  = (uint8_t)( seq        & 0xFF);
    buf[9]  = (uint8_t)((seq >>  8) & 0xFF);
    buf[10] = (uint8_t)((seq >> 16) & 0xFF);
    buf[11] = (uint8_t)((seq >> 24) & 0xFF);

    memcpy(&buf[LAT_HEADER_BYTES], data, plen);

    return mcu_wan_try_enqueue_uplink(HANDLER_LAT, buf,
                                      (uint16_t)(LAT_HEADER_BYTES + plen));
}

#else  /* BENCH_LATENCY_LAN_ENABLE */

bool bench_latency_lan_send(const uint8_t *d, uint16_t l) { (void)d; (void)l; return false; }

#endif /* BENCH_LATENCY_LAN_ENABLE */
