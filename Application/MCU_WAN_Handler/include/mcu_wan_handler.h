/**
 * @file mcu_wan_handler.h
 * @brief MCU WAN Handler - LAN Side (SPI Master)
 */

#ifndef MCU_WAN_HANDLER_H
#define MCU_WAN_HANDLER_H

#include "esp_err.h"
#include "frame_types.h"
#include <stdbool.h>
#include <stdint.h>

// ===== Firmware Version =====
#define DA2_VERSION_MAJOR 2
#define DA2_VERSION_MINOR 1
#define DA2_VERSION_PATCH 1
#define DA2_VERSION_BUILD 0 // Increment after FOTA

#define DA2_STR_HELPER(x) #x
#define DA2_STR(x) DA2_STR_HELPER(x)

#define DA2_CURRENT_VERSION_STR                                                \
  DA2_STR(DA2_VERSION_MAJOR) "." DA2_STR(DA2_VERSION_MINOR) "."            \
      DA2_STR(DA2_VERSION_PATCH)

#define LAN_FW_VERSION_MAJOR DA2_VERSION_MAJOR
#define LAN_FW_VERSION_MINOR DA2_VERSION_MINOR
#define LAN_FW_VERSION_PATCH DA2_VERSION_PATCH
#define LAN_FW_VERSION_BUILD DA2_VERSION_BUILD

#define LAN_FW_VERSION                                                         \
  FW_VERSION_MAKE(LAN_FW_VERSION_MAJOR, LAN_FW_VERSION_MINOR,                  \
                  LAN_FW_VERSION_PATCH, LAN_FW_VERSION_BUILD)

// ===== Public API =====

/**
 * @brief Start MCU WAN handler
 */
esp_err_t mcu_wan_handler_start(void);

/**
 * @brief Stop MCU WAN handler
 */
esp_err_t mcu_wan_handler_stop(void);

/**
 * @brief Enqueue uplink data from WAN handlers to be sent to WAN MCU
 *
 * Default route is UPLINK_ROUTE_CLOUD: payload is treated as telemetry,
 * persisted to SD when internet is offline, and replayed when online.
 * Use for node-originated data (LoRa/BLE/Zigbee/RS485 sensors, events).
 */
bool mcu_wan_enqueue_uplink(handler_id_t source_id, uint8_t *data,
                            uint16_t len);

/**
 * @brief Enqueue a local response that must reach the WAN MCU immediately,
 *        regardless of internet state, and must NOT be persisted to SD.
 *
 * Use for every ACK / error / result that answers a CF command originated by
 * the config app (UART/USB/Web). WAN MCU correlates these responses with the
 * last CF source via a short-lived cache; stale responses replayed from SD
 * would route to MQTT instead of the originating channel.
 */
bool mcu_wan_enqueue_uplink_local(handler_id_t source_id, uint8_t *data,
                                  uint16_t len);

/**
 * @brief Non-blocking enqueue (CLOUD route). Returns false immediately on
 *        full queue. Use for high-rate producers that prefer dropping over
 *        backpressure blocking.
 */
bool mcu_wan_try_enqueue_uplink(handler_id_t source_id, uint8_t *data,
                                uint16_t len);

/**
 * @brief Variant tagging the uplink item with the absolute LAN
 *        `esp_timer_get_time()` value captured the moment the wireless module
 *        first received the data. Carried verbatim inside the SPI DT frame
 *        so the WAN MCU can convert via `bench_time_sync_from_peer_us()` and
 *        compute the unified `[E2E_TOTAL]` latency. Pass 0 if not measured.
 */
bool mcu_wan_enqueue_uplink_with_ts(handler_id_t source_id, uint8_t *data,
                                    uint16_t len, int64_t lan_rx_us);

/**
 * @brief Get current internet status (cached from WAN MCU)
 */
internet_status_t mcu_wan_handler_get_internet_status(void);

/**
 * @brief Get cached RTC time string (from WAN MCU, updated every 1s)
 */
esp_err_t mcu_wan_handler_get_rtc(char *buffer);

/**
 * @brief Register callback for config data reception
 */
void mcu_wan_handler_register_config_callback(void (*callback)(const uint8_t *,
                                                               uint16_t, bool));

/**
 * @brief Get cached WAN MCU firmware version
 */
uint32_t mcu_wan_handler_get_wan_fw_version(void);

#endif // MCU_WAN_HANDLER_H
