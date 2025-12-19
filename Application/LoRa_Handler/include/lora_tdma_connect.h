/**
 * @file lora_tdma_connect.h
 * @brief LoRa TDMA connect task (gateway side)
 *
 * - Uplink: Forward received sensor data frames to WAN MCU
 *           via mcu_wan_enqueue_uplink(HANDLER_LORA, ...)
 * - Downlink: Receive downlink payload from WAN MCU and
 *             schedule LoRa TDMA transmissions to sensors.
 *
 * Downlink payload format (from dispatch_downlink_to_handler):
 *   [sensor_addr(2)][length(2)][data(length)]
 *   - sensor_addr: big endian (0x1234 -> 0x12 0x34)
 *   - length:      big endian
 *   - data:        raw application payload for the sensor
 */

#ifndef LORA_TDMA_CONNECT_H
#define LORA_TDMA_CONNECT_H

#include "esp_err.h"
#include "lora_tdma_handler.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief High-level statistics for the LoRa TDMA connect task.
 *
 * Values are accumulated from:
 *  - Internal LoRa TDMA stats (tx_ok, rx_ok, rx_error, missed_beacon)
 *  - Connect-layer counters (uplink/downlink activity)
 */
typedef struct {
  uint32_t tx_ok;         /**< Frames successfully sent over LoRa       */
  uint32_t rx_ok;         /**< Frames successfully received over LoRa   */
  uint32_t rx_error;      /**< RX parse/crypto errors                   */
  uint32_t missed_beacon; /**< Number of times sensor lost sync         */

  uint32_t uplink_forwarded;  /**< Sensor frames forwarded to WAN uplink    */
  uint32_t uplink_queue_full; /**< Times WAN uplink queue was full          */

  uint32_t downlink_enqueued; /**< Downlink frames accepted into queue      */
  uint32_t downlink_dropped;  /**< Downlink frames dropped (queue/busy)     */
} lora_tdma_connect_stats_t;

/**
 * @brief Global LoRa TDMA context.
 *
 * The radio driver (E32) should call lora_handler_handle_rx()
 * with this context when raw bytes are received from the radio.
 */
extern lora_handler_ctx_t g_lora_tdma_ctx;

/**
 * @brief Start LoRa TDMA connect task.
 *
 * Requirements:
 *  - LoRa E32 radio driver must be initialized externally.
 *  - Global handle g_lora_e32_handle (from lora_e32_comm) must be valid.
 *
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t lora_tdma_connect_start(void);

/**
 * @brief Stop LoRa TDMA connect task and free resources.
 *
 * @return ESP_OK on success.
 */
esp_err_t lora_tdma_connect_stop(void);

/**
 * @brief Enqueue a downlink message to be sent to a sensor via LoRa TDMA.
 *
 * Expected payload format (from WAN handler):
 *   [sensor_addr(2)][length(2)][data(length)]
 *
 * - sensor_addr: sensor logical address (big endian)
 * - length:      number of data bytes (big endian)
 * - data:        payload for the sensor
 *
 * The data buffer is copied internally; caller does not need to keep it.
 *
 * @param data Pointer to downlink buffer
 * @param len  Total length of buffer
 * @return true on success, false on error or queue full
 */
bool lora_tdma_connect_enqueue_downlink(uint8_t *data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* LORA_TDMA_CONNECT_H */
