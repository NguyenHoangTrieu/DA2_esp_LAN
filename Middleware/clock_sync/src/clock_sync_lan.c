#include "clock_sync_lan.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

/* Cross-MCU clock sync — LAN side, simple variant.
 *
 * Every call to clock_sync_lan_update() is GUARANTEED FRESH by the upstream
 * nonce echo check in request_rtc_and_status(): a response whose nonce
 * doesn't match the most recent request never reaches this function. So
 * here we only need to write the new offset; no EMA, no warmup, no bound
 * check, no 2-sample bootstrap. */

static portMUX_TYPE s_lock      = portMUX_INITIALIZER_UNLOCKED;
static int64_t      s_offset_us = 0;
static bool         s_valid     = false;

void clock_sync_lan_update(uint64_t wan_us, uint64_t lan_ref_us)
{
    if (wan_us == 0) return;
    int64_t new_offset = (int64_t)wan_us - (int64_t)lan_ref_us;
    portENTER_CRITICAL(&s_lock);
    s_offset_us = new_offset;
    s_valid     = true;
    portEXIT_CRITICAL(&s_lock);
}

uint64_t clock_sync_lan_to_master_us(uint64_t local_us)
{
    int64_t off;
    bool    valid;
    portENTER_CRITICAL(&s_lock);
    off   = s_offset_us;
    valid = s_valid;
    portEXIT_CRITICAL(&s_lock);
    return valid ? (uint64_t)((int64_t)local_us + off) : local_us;
}

uint64_t clock_sync_lan_now_master_us(void)
{
    return clock_sync_lan_to_master_us((uint64_t)esp_timer_get_time());
}
