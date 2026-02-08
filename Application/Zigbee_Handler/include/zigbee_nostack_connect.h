/**
 * @file zigbee_nostack_connect.h
 * @brief Zigbee No-Stack Connect Task (Gateway Side)
 */

#ifndef ZIGBEE_NOSTACK_CONNECT_H
#define ZIGBEE_NOSTACK_CONNECT_H

#include "esp_err.h"
#include "zigbee_nostack_handler.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Zigbee connect statistics
 */
typedef struct {
  uint32_t tx_ok;             /**< Successfully sent frames */
  uint32_t rx_ok;             /**< Successfully received frames */
  uint32_t rx_error;          /**< RX parse errors */
  uint32_t uplink_forwarded;  /**< Frames forwarded to WAN uplink */
  uint32_t uplink_queue_full; /**< WAN uplink queue full events */
  uint32_t downlink_enqueued; /**< Downlink frames accepted */
  uint32_t downlink_dropped;  /**< Downlink frames dropped */
} zigbee_nostack_connect_stats_t;

/**
 * @brief Global Zigbee handler context
 *
 * The CC2530 driver should call zigbee_nostack_handler_handle_rx()
 * with this context when data is received.
 */
extern zigbee_nostack_handler_ctx_t g_zigbee_nostack_ctx;

/**
 * @brief Start Zigbee no-stack connect task
 *
 * Requirements:
 * - CC2530 UART driver must be initialized externally
 * - Global handle g_zigbee_cc_handle must be valid
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t zigbee_nostack_connect_start(void);

/**
 * @brief Stop Zigbee no-stack connect task
 *
 * @return ESP_OK on success
 */
esp_err_t zigbee_nostack_connect_stop(void);

/**
 * @brief Enqueue downlink message to Zigbee node
 *
 * Expected payload format:
 * [sensor_addr(2)][length(2)][data(length)]
 *
 * @param data Downlink buffer
 * @param len Total buffer length
 * @return true on success, false on error/queue full
 */
bool zigbee_nostack_connect_enqueue_downlink(uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* ZIGBEE_NOSTACK_CONNECT_H */