/**
 * @file bench_lane_ingress.h
 * @brief LAN-side per-lane ingress benchmark (UART/SPI/I2C/USB).
 *
 * Goal: find the per-lane ceiling — how much raw data each LAN MCU lane
 * can swallow per second before driver-side overflow.
 *
 * Method: a dedicated raw consumer task drains the chosen lane in a tight
 * loop (bypassing production handlers, which are too slow to saturate).
 * External rig (ESP32-S3) generates the burst traffic. The counter is
 * incremented inside module_bus_read() — the single choke point shared by
 * all four lanes.
 */

#ifndef BENCH_LANE_INGRESS_H
#define BENCH_LANE_INGRESS_H

#include "esp_err.h"
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Master switch.
 *   0 = OFF — all public APIs no-op; zero overhead in module_bus_read.
 *   1 = ON  — raw consumer task drains the chosen lane + counter live.
 */
#define BENCH_LANE_INGRESS_ENABLE 1

/**
 * @brief Isolation switch for the lane bench.
 *   0 = full layout: WAN bridge + the §1/§4 inter-MCU SPI FLOOD benches both run
 *       alongside the lane consumer (worst-case contention).
 *   1 = no-flood: app_main keeps the WAN handler up (stable, normal bridge) but
 *       skips the §1/§4 SPI throughput/counter benches (the artificial flood).
 *       This isolates how much of the UART throughput drop is caused by the SPI
 *       flood. If UART jumps toward ~720 kbps, the contention hypothesis is
 *       confirmed. Revert to 0 afterwards.
 */
#ifndef BENCH_LANE_ISOLATE
#define BENCH_LANE_ISOLATE 0
#endif

#define BENCH_LANE_REPORT_INTERVAL_MS 2000

/**
 * @brief Stack ID + lane the raw consumer drains (Mode A only).
 *        Edit before flashing for the lane under test.
 */
#define BENCH_LANE_RAW_STACK_ID 1
#define BENCH_LANE_RAW_PORT     1  /* 0=UART, 1=SPI, 2=I2C, 3=USB */

/** Read chunk size for raw consumer (bytes). */
#define BENCH_LANE_RAW_READ_CHUNK 512

/** Read timeout for raw consumer (ms). */
#define BENCH_LANE_RAW_READ_TIMEOUT_MS 50

/** Lane identifiers — must stay in sync with the indexing below. */
typedef enum {
    BENCH_LANE_UART = 0,
    BENCH_LANE_SPI  = 1,
    BENCH_LANE_I2C  = 2,
    BENCH_LANE_USB  = 3,
    BENCH_LANE_COUNT
} bench_lane_id_t;

/** Number of stacks instrumented. Mirrors module_config_controller (0/1). */
#define BENCH_LANE_STACK_COUNT 2

#if BENCH_LANE_INGRESS_ENABLE

/** Start reporter task. Call after module_config_controller_init(). */
esp_err_t bench_lane_ingress_start(void);

/** Stop reporter task. */
void bench_lane_ingress_stop(void);

/** Count a successful read from module_bus_read(). */
void bench_lane_count_rx(uint8_t stack_id, bench_lane_id_t lane,
                        uint32_t bytes);

/** Count a failed/empty read (timeout, queue empty, etc.). */
void bench_lane_count_miss(uint8_t stack_id, bench_lane_id_t lane);

/** Count an explicit drop (queue full / overflow signaled by lower driver). */
void bench_lane_count_drop(uint8_t stack_id, bench_lane_id_t lane);

/** Lower driver signalled buffer-full / FIFO overflow event. Call from inside
 *  the driver overflow ISR or status check (UART_BUFFER_FULL / FIFO_OVF,
 *  I2C STR.OVERFLOW, USB CDC RX overflow, SPI slave overrun). */
void bench_lane_count_drv_buf_full(uint8_t stack_id, bench_lane_id_t lane);

/** Update the high-water mark of the lower hardware FIFO (bytes pending in
 *  driver software ring buffer, sampled periodically by reporter). */
void bench_lane_observe_hw_fifo(uint8_t stack_id, bench_lane_id_t lane,
                                uint32_t depth_bytes);

#else  /* BENCH_LANE_INGRESS_ENABLE == 0 → stubs */

static inline esp_err_t bench_lane_ingress_start(void) { return ESP_OK; }
static inline void bench_lane_ingress_stop(void) {}
static inline void bench_lane_count_rx(uint8_t stack_id, bench_lane_id_t lane,
                                      uint32_t bytes) {
    (void)stack_id; (void)lane; (void)bytes;
}
static inline void bench_lane_count_miss(uint8_t stack_id,
                                        bench_lane_id_t lane) {
    (void)stack_id; (void)lane;
}
static inline void bench_lane_count_drop(uint8_t stack_id,
                                        bench_lane_id_t lane) {
    (void)stack_id; (void)lane;
}
static inline void bench_lane_count_drv_buf_full(uint8_t stack_id,
                                                bench_lane_id_t lane) {
    (void)stack_id; (void)lane;
}
static inline void bench_lane_observe_hw_fifo(uint8_t stack_id,
                                             bench_lane_id_t lane,
                                             uint32_t depth_bytes) {
    (void)stack_id; (void)lane; (void)depth_bytes;
}

#endif /* BENCH_LANE_INGRESS_ENABLE */

#ifdef __cplusplus
}
#endif

#endif /* BENCH_LANE_INGRESS_H */
