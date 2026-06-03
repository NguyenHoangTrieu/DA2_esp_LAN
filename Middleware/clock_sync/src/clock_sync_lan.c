#include "clock_sync_lan.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

/* Cross-MCU clock sync — LAN side, drift-compensated variant.
 *
 * WHY NOT A PLAIN OFFSET:
 *   The old version wrote offset = wan_us − lan_ref_us on every fresh sample
 *   and nothing in between. The two MCUs run on independent crystals (~tens of
 *   ppm apart), so between samples the mapping drifts. Worse, under bench load
 *   the 1 Hz RTC response is often stale (nonce rejected) → the offset FREEZES
 *   for many seconds → the relative crystal skew leaks straight into the
 *   measured latency (the "lat slowly slides 8 ms → 3 ms" symptom).
 *
 * WHAT THIS DOES:
 *   Keep a small sliding window of the last CS_WIN fresh (lan_ref, wan) pairs
 *   and fit a line  wan ≈ slope·(lan − lan0) + intercept  by least squares.
 *   - slope  captures the crystal SKEW (rate ratio), so the mapping stays
 *     accurate even when samples are sparse → no slow drift.
 *   - the fit AVERAGES the ±1-3 ms per-sample jitter (WAN's stamp delay) →
 *     a stable reading instead of a per-tick jump.
 *   - the window (~CS_WIN seconds) lets it track slow thermal drift.
 *
 * Only the §5 latency bench consumes this (clock_sync_lan_now_master_us in
 * bench_latency_lan.c) — no production code path depends on it.
 */

#define CS_WIN 32   /* sliding-window length (≈ samples ≈ seconds of history) */

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static uint64_t s_lan[CS_WIN];   /* lan_ref_us samples (ring)                 */
static uint64_t s_wan[CS_WIN];   /* wan_us samples (ring)                     */
static int      s_count = 0;     /* valid samples in window                   */
static int      s_head  = 0;     /* next write index                          */
static uint64_t s_lan0  = 0;     /* x-origin so regression maths stays small  */

/* Cached fit: master ≈ s_slope·(local − s_lan0) + s_intercept */
static double   s_slope     = 1.0;
static double   s_intercept = 0.0;
static bool     s_valid     = false;

void clock_sync_lan_update(uint64_t wan_us, uint64_t lan_ref_us)
{
    if (wan_us == 0) return;

    portENTER_CRITICAL(&s_lock);

    if (s_count == 0) {
        s_lan0 = lan_ref_us;            /* anchor x-origin on first sample */
    }

    s_lan[s_head] = lan_ref_us;
    s_wan[s_head] = wan_us;
    s_head = (s_head + 1) % CS_WIN;
    if (s_count < CS_WIN) s_count++;

    if (s_count == 1) {
        /* Single sample → behave exactly like the old plain offset.
         * x = lan_ref − lan0 = 0 here, so y = wan ⇒ intercept = wan_us;
         * master(local) = (local − lan0) + wan_us = local + (wan − lan_ref). */
        s_slope     = 1.0;
        s_intercept = (double)wan_us;
        s_valid     = true;
    } else {
        /* Least squares over the window, MEAN-CENTERED for numerical
         * stability (x = lan − lan0 grows large over a long run; the naive
         * n·Σx² − (Σx)² form then subtracts two huge near-equal numbers and
         * loses all precision). Centering on the means avoids that:
         *   x = lan − lan0,  y = wan,  fit y = a·(x − x̄) + ȳ. */
        const double n = (double)s_count;
        double mx = 0.0, my = 0.0;
        for (int i = 0; i < s_count; i++) {
            mx += (double)((int64_t)s_lan[i] - (int64_t)s_lan0);
            my += (double)s_wan[i];
        }
        mx /= n;
        my /= n;

        double sxx = 0.0, sxy = 0.0;
        for (int i = 0; i < s_count; i++) {
            double dx = (double)((int64_t)s_lan[i] - (int64_t)s_lan0) - mx;
            double dy = (double)s_wan[i] - my;
            sxx += dx * dx;
            sxy += dx * dy;
        }
        if (sxx > 1.0) {
            double a = sxy / sxx;          /* slope (rate ratio ≈ 1) */
            double b = my - a * mx;        /* intercept at x = 0      */
            /* Reject nonsense slopes (crystals are well within ±2000 ppm);
             * if rejected, keep the previous fit rather than corrupt it. */
            if (a > 0.998 && a < 1.002) {
                s_slope     = a;
                s_intercept = b;
                s_valid     = true;
            }
        }
    }

    portEXIT_CRITICAL(&s_lock);
}

uint64_t clock_sync_lan_to_master_us(uint64_t local_us)
{
    bool     valid;
    double   slope, intercept;
    uint64_t lan0;

    portENTER_CRITICAL(&s_lock);
    valid     = s_valid;
    slope     = s_slope;
    intercept = s_intercept;
    lan0      = s_lan0;
    portEXIT_CRITICAL(&s_lock);

    if (!valid) return local_us;

    double m = slope * (double)((int64_t)local_us - (int64_t)lan0) + intercept;
    if (m < 0.0) m = 0.0;
    return (uint64_t)m;
}

uint64_t clock_sync_lan_now_master_us(void)
{
    return clock_sync_lan_to_master_us((uint64_t)esp_timer_get_time());
}
