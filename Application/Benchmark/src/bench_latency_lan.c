#include "bench_latency_lan.h"

#if BENCH_LATENCY_LAN_ENABLE

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "clock_sync_lan.h"
#include "frame_types.h"
#include "mcu_wan_handler.h"
#include "module_config_controller.h"
#include "json_config_parser.h"

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

/* ===========================================================================
 * §5 USB-rig ingress source (SRC_USB)
 *
 * Production-fidelity proof:
 *   The ONLY thing this task does that the legacy RS485 path didn't is read
 *   bytes from the USB CDC lane instead of the RS485 lane. From the moment a
 *   complete packet is assembled it calls the SAME bench_latency_lan_send()
 *   the RS485 handler calls — i.e. T1 stamp + mcu_wan_try_enqueue_uplink(
 *   HANDLER_LAT) → the shared uplink queue → the production dispatcher → the
 *   production SPI framing → WAN. Nothing downstream of the enqueue is aware
 *   of, or changed by, the ingress lane. The measured T2 − T1 therefore covers
 *   the identical inter-MCU bridge + WAN egress that a real handler's telemetry
 *   traverses; only the (sub-100 µs) ingress lane differs.
 *
 * Framing: the rig (bench_lane_rig.ino, RIG_LATENCY_E2E) emits fixed
 * BENCH_LATENCY_USB_PKT-byte packets back-to-back, each starting with a SOF
 * magic (0x55 0xAA) + 4-byte LE seq. USB CDC is a delimiter-less byte stream,
 * so we run a self-synchronising parser: scan for SOF, then read PKT bytes and
 * re-validate SOF on every frame. This locks alignment regardless of where in
 * the stream the host attaches, and recovers within one packet if a USB byte
 * is ever dropped. T1 is stamped the instant a full packet is assembled — the
 * exact analogue of "RS485 frame complete" in the legacy path. The seq header
 * is used to measure ingress loss (rig→LAN) via forward seq gaps.
 * ========================================================================= */
#if (BENCH_LATENCY_LAN_SRC == BENCH_LATENCY_LAN_SRC_USB)

static const char *USB_TAG = "bench_lat_usb";
static volatile bool s_usb_running = false;

#define USB_TASK_STACK_WORDS (4096 / sizeof(StackType_t))

static esp_err_t usb_lane_init(uint8_t stack_id) {
    usb_params_t p = {
        .bit_rate  = 921600, /* ignored by CDC, kept for parity with §2 */
        .stop_bits = 0,
        .parity    = 0,
        .data_bits = 8,
    };
    return module_config_controller_init_usb(stack_id, &p);
}

static void bench_latency_usb_task(void *arg) {
    (void)arg;
    const uint8_t stack_id = BENCH_LATENCY_USB_STACK_ID;

    esp_err_t init_ret = usb_lane_init(stack_id);
    if (init_ret != ESP_OK && init_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(USB_TAG, "USB lane init failed: %s", esp_err_to_name(init_ret));
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(USB_TAG,
             "USB ingress started (stack=%u pkt=%d) — SOF-resync framing (0x55AA)",
             (unsigned)stack_id, (int)BENCH_LATENCY_USB_PKT);

    static uint8_t scratch[BENCH_LATENCY_USB_READ_CHUNK];
    static uint8_t pkt[BENCH_LATENCY_USB_PKT];
    size_t   fill   = 0;           /* bytes accumulated into the current pkt   */
    bool     locked = false;       /* SOF found, currently filling a frame     */
    uint8_t  prev   = 0;           /* last byte, for the 2-byte SOF search      */
    uint32_t expect_seq = 0;
    bool     have_expect = false;

    /* Per-window diagnostics (rig→LAN ingress health). */
    uint32_t w_frames = 0, w_ingress_loss = 0, w_enq_drop = 0, w_resync = 0;
    TickType_t last_report = xTaskGetTickCount();

    while (s_usb_running) {
        size_t got = 0;
        (void)module_bus_read(stack_id, COMM_PORT_USB, scratch, sizeof(scratch),
                              BENCH_LATENCY_USB_READ_TIMEOUT_MS, &got);
        for (size_t i = 0; i < got; i++) {
            uint8_t b = scratch[i];

            if (!locked) {
                /* Hunt for the SOF magic (0x55 0xAA) with a 2-byte window. */
                if (prev == BENCH_LATENCY_USB_SOF0 &&
                    b    == BENCH_LATENCY_USB_SOF1) {
                    pkt[0] = BENCH_LATENCY_USB_SOF0;
                    pkt[1] = BENCH_LATENCY_USB_SOF1;
                    fill   = 2;
                    locked = true;
                    prev   = 0;
                } else {
                    prev = b;
                }
                continue;
            }

            pkt[fill++] = b;
            if (fill < BENCH_LATENCY_USB_PKT) continue;
            fill = 0;

            /* A frame's worth of bytes collected. The next packet must begin
             * with SOF; if not, a byte was lost — drop this frame and re-hunt. */
            if (pkt[0] != BENCH_LATENCY_USB_SOF0 ||
                pkt[1] != BENCH_LATENCY_USB_SOF1) {
                w_resync++;
                locked = false;
                prev   = 0;
                continue;
            }

            /* Full, SOF-validated packet — this is the §5 ingress instant. */
            uint32_t seq =
                (uint32_t)pkt[BENCH_LATENCY_USB_SEQ_OFF]            |
                ((uint32_t)pkt[BENCH_LATENCY_USB_SEQ_OFF + 1] << 8) |
                ((uint32_t)pkt[BENCH_LATENCY_USB_SEQ_OFF + 2] << 16)|
                ((uint32_t)pkt[BENCH_LATENCY_USB_SEQ_OFF + 3] << 24);

            /* Ingress loss (rig→LAN): forward gap in seq = packets lost on the
             * USB link. int32 diff tolerates wrap; ignore non-forward jumps. */
            if (have_expect) {
                int32_t d = (int32_t)(seq - expect_seq);
                if (d > 0) w_ingress_loss += (uint32_t)d;
            }
            expect_seq  = seq + 1;
            have_expect = true;

            /* SAME production hand-off as the RS485 path. */
            if (bench_latency_lan_send(pkt, BENCH_LATENCY_USB_PKT)) {
                w_frames++;
            } else {
                w_enq_drop++; /* uplink queue full — system at/over its ceiling */
            }
        }
        if (got == 0) vTaskDelay(1);

        TickType_t now = xTaskGetTickCount();
        if ((now - last_report) >=
            pdMS_TO_TICKS(BENCH_LATENCY_USB_REPORT_INTERVAL_MS)) {
            ESP_LOGI(USB_TAG,
                     "USB_LAT frames=%lu ingress_loss=%lu enq_drop=%lu resync=%lu locked=%d",
                     (unsigned long)w_frames, (unsigned long)w_ingress_loss,
                     (unsigned long)w_enq_drop, (unsigned long)w_resync,
                     (int)locked);
            w_frames = w_ingress_loss = w_enq_drop = w_resync = 0;
            last_report = now;
        }
    }
    ESP_LOGI(USB_TAG, "USB ingress stopped");
    vTaskDelete(NULL);
}

void bench_latency_usb_start(void) {
    if (s_usb_running) return;
    s_usb_running = true;

    StackType_t  *stack = (StackType_t *)heap_caps_malloc(
        USB_TASK_STACK_WORDS * sizeof(StackType_t),
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    StaticTask_t *tcb = (StaticTask_t *)heap_caps_malloc(
        sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!stack || !tcb) {
        if (stack) heap_caps_free(stack);
        if (tcb)   heap_caps_free(tcb);
        s_usb_running = false;
        ESP_LOGE(USB_TAG, "Failed to allocate USB ingress task memory");
        return;
    }
    /* Priority mirrors the §2 raw consumer: above the reporter, below the
     * production handlers, so it never starves the bridge it is measuring. */
    TaskHandle_t h = xTaskCreateStatic(
        bench_latency_usb_task, "bench_lat_usb",
        USB_TASK_STACK_WORDS, NULL, 4, stack, tcb);
    if (!h) {
        heap_caps_free(stack);
        heap_caps_free(tcb);
        s_usb_running = false;
    }
}

#else  /* SRC != USB → stub so app_main needs no guard */

void bench_latency_usb_start(void) {}

#endif /* BENCH_LATENCY_LAN_SRC == BENCH_LATENCY_LAN_SRC_USB */

#else  /* BENCH_LATENCY_LAN_ENABLE */

bool bench_latency_lan_send(const uint8_t *d, uint16_t l) { (void)d; (void)l; return false; }
void bench_latency_usb_start(void) {}

#endif /* BENCH_LATENCY_LAN_ENABLE */
