/**
 * @file bench_lane_ingress.c
 * @brief Implementation of per-lane LAN ingress benchmark.
 *
 * Counters are protected by a portMUX critical section so incrementers can
 * be called from any handler task safely (and even from ISR if needed).
 *
 * Every BENCH_LANE_REPORT_INTERVAL_MS the reporter task takes a snapshot,
 * resets the live counters, and emits a single ESP_LOG line in JSON form.
 */

#include "bench_lane_ingress.h"

#if BENCH_LANE_INGRESS_ENABLE

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "module_config_controller.h"
#include "json_config_parser.h"

static const char *TAG = "bench_lane";

#define BENCH_LANE_TASK_STACK_WORDS (4096 / sizeof(StackType_t))

typedef struct {
    volatile uint32_t pkt;            /* successful reads (count)      */
    volatile uint32_t bytes;          /* total bytes consumed          */
    volatile uint32_t miss;           /* empty/timeout reads           */
    volatile uint32_t drop;           /* explicit overflow drops       */
    volatile uint32_t drv_buf_full;   /* driver buffer-full events     */
    volatile uint32_t hw_fifo_max;    /* high-water mark, bytes in ring*/
} lane_counter_t;

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;
static lane_counter_t s_ctr[BENCH_LANE_STACK_COUNT][BENCH_LANE_COUNT];
static volatile bool s_running = false;

static const char *lane_name(bench_lane_id_t l) {
    switch (l) {
        case BENCH_LANE_UART: return "uart";
        case BENCH_LANE_SPI:  return "spi";
        case BENCH_LANE_I2C:  return "i2c";
        case BENCH_LANE_USB:  return "usb";
        default:              return "?";
    }
}

void bench_lane_count_rx(uint8_t stack_id, bench_lane_id_t lane,
                        uint32_t bytes) {
    if (stack_id >= BENCH_LANE_STACK_COUNT || lane >= BENCH_LANE_COUNT)
        return;
    portENTER_CRITICAL(&s_mux);
    s_ctr[stack_id][lane].pkt++;
    s_ctr[stack_id][lane].bytes += bytes;
    portEXIT_CRITICAL(&s_mux);
}

void bench_lane_count_miss(uint8_t stack_id, bench_lane_id_t lane) {
    if (stack_id >= BENCH_LANE_STACK_COUNT || lane >= BENCH_LANE_COUNT)
        return;
    portENTER_CRITICAL(&s_mux);
    s_ctr[stack_id][lane].miss++;
    portEXIT_CRITICAL(&s_mux);
}

void bench_lane_count_drop(uint8_t stack_id, bench_lane_id_t lane) {
    if (stack_id >= BENCH_LANE_STACK_COUNT || lane >= BENCH_LANE_COUNT)
        return;
    portENTER_CRITICAL(&s_mux);
    s_ctr[stack_id][lane].drop++;
    portEXIT_CRITICAL(&s_mux);
}

void bench_lane_count_drv_buf_full(uint8_t stack_id, bench_lane_id_t lane) {
    if (stack_id >= BENCH_LANE_STACK_COUNT || lane >= BENCH_LANE_COUNT)
        return;
    portENTER_CRITICAL(&s_mux);
    s_ctr[stack_id][lane].drv_buf_full++;
    portEXIT_CRITICAL(&s_mux);
}

void bench_lane_observe_hw_fifo(uint8_t stack_id, bench_lane_id_t lane,
                                uint32_t depth_bytes) {
    if (stack_id >= BENCH_LANE_STACK_COUNT || lane >= BENCH_LANE_COUNT)
        return;
    portENTER_CRITICAL(&s_mux);
    if (depth_bytes > s_ctr[stack_id][lane].hw_fifo_max)
        s_ctr[stack_id][lane].hw_fifo_max = depth_bytes;
    portEXIT_CRITICAL(&s_mux);
}

static void bench_lane_reporter_task(void *arg) {
    (void)arg;
    ESP_LOGI(TAG, "Lane ingress reporter started (interval %d ms)",
             BENCH_LANE_REPORT_INTERVAL_MS);

    while (s_running) {
        vTaskDelay(pdMS_TO_TICKS(BENCH_LANE_REPORT_INTERVAL_MS));
        if (!s_running) break;

        lane_counter_t snap[BENCH_LANE_STACK_COUNT][BENCH_LANE_COUNT];

        portENTER_CRITICAL(&s_mux);
        for (int s = 0; s < BENCH_LANE_STACK_COUNT; s++) {
            for (int l = 0; l < BENCH_LANE_COUNT; l++) {
                snap[s][l] = s_ctr[s][l];
                s_ctr[s][l].pkt          = 0;
                s_ctr[s][l].bytes        = 0;
                s_ctr[s][l].miss         = 0;
                s_ctr[s][l].drop         = 0;
                s_ctr[s][l].drv_buf_full = 0;
                s_ctr[s][l].hw_fifo_max  = 0;
            }
        }
        portEXIT_CRITICAL(&s_mux);

        const float interval_s =
            (float)BENCH_LANE_REPORT_INTERVAL_MS / 1000.0f;

        for (int s = 0; s < BENCH_LANE_STACK_COUNT; s++) {
            for (int l = 0; l < BENCH_LANE_COUNT; l++) {
                const lane_counter_t *c = &snap[s][l];
                if (c->pkt == 0 && c->miss == 0 && c->drop == 0 &&
                    c->drv_buf_full == 0 && c->hw_fifo_max == 0)
                    continue; /* skip silent lanes */
                const float pps  = interval_s > 0.0f
                                       ? ((float)c->pkt) / interval_s
                                       : 0.0f;
                const float kbps = interval_s > 0.0f
                                       ? ((float)c->bytes * 8.0f) /
                                             (interval_s * 1000.0f)
                                       : 0.0f;
                ESP_LOGI(TAG,
                         "LANE_BENCH stack=%d %s pkt=%lu b=%lu miss=%lu "
                         "drop=%lu drv_buf_full=%lu hw_fifo_max=%lu "
                         "pps=%.1f kbps=%.1f",
                         s, lane_name((bench_lane_id_t)l),
                         (unsigned long)c->pkt, (unsigned long)c->bytes,
                         (unsigned long)c->miss, (unsigned long)c->drop,
                         (unsigned long)c->drv_buf_full,
                         (unsigned long)c->hw_fifo_max,
                         pps, kbps);
            }
        }
    }

    ESP_LOGI(TAG, "Lane ingress reporter stopped");
    vTaskDelete(NULL);
}

/* Raw consumer: drains the configured lane in a tight loop, bypassing any
 * production handler (which is too slow to saturate). The counter is already
 * incremented inside module_bus_read, so this task does no extra accounting.
 *
 * Lane init: this task auto-inits the chosen lane with sensible defaults so
 * the bench works on a fresh-flashed MCU without needing any prior config. */
static esp_err_t raw_init_lane(uint8_t stack_id, comm_port_type_t port) {
    switch (port) {
        case COMM_PORT_UART: {
            uart_params_t p = {
                .baudrate = 921600,
                .parity   = 0,
                .stopbit  = 1,
            };
            return module_config_controller_init_uart(stack_id, &p);
        }
        case COMM_PORT_SPI: {
            spi_params_t p = {
                .clock_speed = 10000000,
                .mode        = 0,
            };
            return module_config_controller_init_spi(stack_id, &p);
        }
        case COMM_PORT_I2C: {
            i2c_params_t p = {
                .address     = 0x42,
                .clock_speed = 400000,
            };
            return module_config_controller_init_i2c(stack_id, &p);
        }
        case COMM_PORT_USB: {
            usb_params_t p = {
                .bit_rate  = 921600,
                .stop_bits = 0,
                .parity    = 0,
                .data_bits = 8,
            };
            return module_config_controller_init_usb(stack_id, &p);
        }
        default:
            return ESP_ERR_NOT_SUPPORTED;
    }
}

static void bench_lane_raw_consumer_task(void *arg) {
    (void)arg;
    static uint8_t rx_buf[BENCH_LANE_RAW_READ_CHUNK];
    const uint8_t stack_id = BENCH_LANE_RAW_STACK_ID;
    const comm_port_type_t port = (comm_port_type_t)BENCH_LANE_RAW_PORT;

    esp_err_t init_ret = raw_init_lane(stack_id, port);
    if (init_ret != ESP_OK && init_ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Raw lane init failed: %s",
                 esp_err_to_name(init_ret));
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Raw consumer started (stack=%u port=%d chunk=%d)",
             (unsigned)stack_id, (int)port, BENCH_LANE_RAW_READ_CHUNK);

    while (s_running) {
        /* High-water sample BEFORE draining: the ring holds whatever piled up
         * since the last read, so this catches the peak fill (e.g. when the
         * consumer was preempted by higher-prio tasks). UART only — other lanes
         * return 0. Frequent + cheap, and only touches the installed port, so it
         * neither spams errors nor needs the once-per-window reporter sample. */
        if (port == COMM_PORT_UART) {
            bench_lane_observe_hw_fifo(
                stack_id, BENCH_LANE_UART,
                (uint32_t)module_bus_rx_pending(stack_id, port));
        }
        size_t got = 0;
        (void)module_bus_read(stack_id, port, rx_buf,
                              sizeof(rx_buf),
                              BENCH_LANE_RAW_READ_TIMEOUT_MS, &got);
        if (got == 0) {
            vTaskDelay(1);
        } else if (port == COMM_PORT_SPI) {
            /* SPI full-duplex: got is always BENCH_LANE_RAW_READ_CHUNK
             * (spi_device_transmit clocks max_len bytes unconditionally).
             * We MUST NOT vTaskDelay(1) here — that would cap the master
             * to 1000/10ms = 100 pps regardless of the bench step rate.
             * taskYIELD() gives up the CPU to equal/higher-priority tasks
             * for one scheduler pass but lets this task re-run immediately
             * if no other task needs the CPU.
             * spi_device_transmit() already blocks during DMA, so this
             * task yields naturally during every transaction. */
            taskYIELD();
        }
    }
    ESP_LOGI(TAG, "Raw consumer stopped");
    vTaskDelete(NULL);
}

esp_err_t bench_lane_ingress_start(void) {
    if (s_running) return ESP_OK;
    s_running = true;

    for (int s = 0; s < BENCH_LANE_STACK_COUNT; s++)
        for (int l = 0; l < BENCH_LANE_COUNT; l++)
            s_ctr[s][l] = (lane_counter_t){0};

    /* Reporter task */
    {
        StackType_t  *stack = (StackType_t *)heap_caps_malloc(
            BENCH_LANE_TASK_STACK_WORDS * sizeof(StackType_t),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        StaticTask_t *tcb = (StaticTask_t *)heap_caps_malloc(
            sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!stack || !tcb) {
            if (stack) heap_caps_free(stack);
            if (tcb)   heap_caps_free(tcb);
            s_running = false;
            ESP_LOGE(TAG, "Failed to allocate reporter memory");
            return ESP_ERR_NO_MEM;
        }
        TaskHandle_t h = xTaskCreateStatic(
            bench_lane_reporter_task, "bench_lane",
            BENCH_LANE_TASK_STACK_WORDS, NULL, 3, stack, tcb);
        if (!h) {
            heap_caps_free(stack);
            heap_caps_free(tcb);
            s_running = false;
            return ESP_FAIL;
        }
    }

    /* Raw consumer task — always spawned when ENABLE = 1 */
    {
        StackType_t  *stack = (StackType_t *)heap_caps_malloc(
            BENCH_LANE_TASK_STACK_WORDS * sizeof(StackType_t),
            MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        StaticTask_t *tcb = (StaticTask_t *)heap_caps_malloc(
            sizeof(StaticTask_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (!stack || !tcb) {
            if (stack) heap_caps_free(stack);
            if (tcb)   heap_caps_free(tcb);
            ESP_LOGE(TAG, "Raw consumer alloc failed (reporter still up)");
            return ESP_ERR_NO_MEM;
        }
        TaskHandle_t h = xTaskCreateStatic(
            bench_lane_raw_consumer_task, "bench_lane_raw",
            BENCH_LANE_TASK_STACK_WORDS, NULL,
            4, /* higher than reporter, lower than handlers */
            stack, tcb);
        if (!h) {
            heap_caps_free(stack);
            heap_caps_free(tcb);
            return ESP_FAIL;
        }
    }

    ESP_LOGI(TAG, "Lane ingress bench started");
    return ESP_OK;
}

void bench_lane_ingress_stop(void) {
    s_running = false;
}

#endif /* BENCH_LANE_INGRESS_ENABLE */
