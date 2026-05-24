/**
 * @file frame_types.h
 * @brief Common frame types and data structures for LAN-WAN SPI communication
 */

#ifndef FRAME_TYPES_H
#define FRAME_TYPES_H

#include <stdbool.h>
#include <stdint.h>

// ===== SPI Frame Header Values =====
#define WAN_COMM_HEADER_CF 0x4346  // "CF" - Command Frame
#define WAN_COMM_HEADER_DT 0x4454  // "DT" - Data Frame
#define WAN_COMM_HEADER_DQ 0x4451  // "DQ" - Data Query
#define WAN_COMM_HEADER_CQ 0x4351  // "CQ" - Config Query
#define WAN_COMM_HEADER_RT 0x5254  // "RT" - RTC Response
#define WAN_COMM_HEADER_ACK 0x0241 // ACK byte prefix

// ===== Legacy ASCII Prefixes (for compatibility) =====
#define PREFIX_RTC "RT"    // RTC Config Response
#define PREFIX_CONFIG "CF" // Configuration Data
#define PREFIX_DATA "DT"   // Data Packet
#define PREFIX_QUERY "DQ"  // Data Query
#define PREFIX_CFCQ "CQ"   // Config Query

// ===== Inter-MCU Common Limits =====
#define INTER_MCU_PAYLOAD_MAX_LEN 2048
#define INTER_MCU_BATCH_INTERVAL_MS 10

// ===== Handler Type Identifiers (3-byte ASCII) =====
#define HANDLER_TYPE_CAN "CAN"
#define HANDLER_TYPE_LOR "LOR" // LoRa
#define HANDLER_TYPE_ZIG "ZIG" // ZigBee
#define HANDLER_TYPE_RS4 "RS4" // RS485
#define HANDLER_TYPE_BLE "BLE" // Bluetooth Low Energy (AT module via UART)
#define HANDLER_TYPE_BLN "BLN" // BLE Native — ESP32 BLE Mesh provisioner
#define HANDLER_TYPE_BLG "BLG" // BLE GATT Central — ESP32 native GATT client
#define HANDLER_TYPE_BENCH "BNC" // Throughput benchmark (sink only, never routed)

// ===== Frame Types (Single Byte) =====
typedef enum {
  FRAME_TYPE_HANDSHAKE = 0x01,
  FRAME_TYPE_ACK = 0x02,
  FRAME_TYPE_RTC = 0x03,         // "RT" prefix
  FRAME_TYPE_CONFIG = 0x04,      // "CF" prefix
  FRAME_TYPE_DATA = 0x05,        // "DT" prefix
  FRAME_TYPE_FOTA = 0x06,        // CFFW - Firmware Update
  FRAME_TYPE_QUERY = 0x07,       // "DQ" - Data Query
  FRAME_TYPE_CONFIG_QUERY = 0x08,// "CQ" - Config Query
  FRAME_TYPE_TSYNC_REQ = 0x09,   // CF + time-sync request (LAN → WAN)
  FRAME_TYPE_TSYNC_RSP = 0x0A    // ACK + time-sync response (WAN → LAN)
} frame_type_t;

// ===== ACK Types =====
typedef enum {
  ACK_TYPE_HANDSHAKE = 0x10,
  ACK_TYPE_RECEIVED_OK = 0x11,
  ACK_TYPE_INTERNET_OK = 0x12,
  ACK_TYPE_NO_INTERNET = 0x13,
  ACK_TYPE_OK = 0x14,
  ACK_TYPE_TIMEOUT = 0x15,
  ACK_TYPE_ERROR = 0x16,
  ACK_TYPE_TSYNC_RSP = 0x1A   // Time-sync response carries WAN t2/t3
} ack_type_t;

// ===== Handler ID Enum =====
typedef enum {
  HANDLER_CAN = 0x01,
  HANDLER_LORA = 0x02,
  HANDLER_ZIGBEE = 0x03,
  HANDLER_RS485 = 0x04,
  HANDLER_BLE = 0x05,
  HANDLER_BLE_NATIVE = 0x06,  /* ESP32-S3 direct BLE Mesh provisioner */
  HANDLER_BLE_GATT   = 0x07,  /* ESP32-S3 native BLE GATT Central      */
  HANDLER_BENCH = 0xFE, // Throughput benchmark traffic — never routed
  HANDLER_UNKNOWN = 0xFF
} handler_id_t;

// ===== Internet Status =====
typedef enum {
  INTERNET_STATUS_OFFLINE = 0,
  INTERNET_STATUS_ONLINE = 1
} internet_status_t;

// ===== Uplink Route =====
/* CLOUD: telemetry-bound; goes through internet check, persisted to SD when
 *        offline, replayed when online. Use for node data (LoRa/BLE/Zigbee/...).
 * LOCAL: response to a config command from app (UART/USB/Web). MUST be
 *        forwarded to WAN MCU immediately so WAN can route it back to the
 *        originating channel before its CF-source correlation expires.
 *        NEVER persisted to SD — stale local responses are useless because the
 *        app has already timed out and the WAN-side correlation is gone. */
typedef enum {
  UPLINK_ROUTE_CLOUD = 0,
  UPLINK_ROUTE_LOCAL = 1
} uplink_route_t;

// ===== Data Structures =====

/**
 * @brief Handshake Request with Firmware Version
 * Format: [CF][0x01][fw_version(4)]
 */
typedef struct __attribute__((packed)) {
  uint16_t header;     // 0x4346 (CF)
  uint8_t cmd;         // 0x01 (handshake command)
  uint32_t fw_version; // Firmware version (e.g., 0x01020304 = v1.2.3.4)
} handshake_request_t;

/**
 * @brief Handshake Response with Internet Status and WAN Version
 * Format: [ACK][0x10][internet_flag][wan_fw_version(4)]
 */
typedef struct __attribute__((packed)) {
  uint8_t ack_prefix;      // 0x02 (ACK)
  uint8_t ack_type;        // 0x10 (handshake)
  uint8_t internet_flag;   // 1 = online, 0 = offline
  uint32_t wan_fw_version; // WAN MCU firmware version
} handshake_response_t;

/**
 * @brief RTC Config Response
 * Format: [RT][dd/mm/yyyy-hh:mm:ss][network_status]
 */
typedef struct __attribute__((packed)) {
  uint8_t prefix[2];      // "RT"
  char rtc_string[20];    // "dd/mm/yyyy-hh:mm:ss\0"
  uint8_t network_status; // 1 = connected, 0 = disconnected
} rtc_config_response_t;

/**
 * @brief Config Data Packet
 * Format: [CF][config_length(2)][config_data]
 */
typedef struct __attribute__((packed)) {
  uint16_t header;        // 0x4346 (CF)
  uint16_t config_length; // Length of config data
  uint8_t config_data[];  // Flexible array for config data
} config_data_t;

/**
 * @brief Data Packet
 * Format: [DT][handler_type(3)][data_length(2)][lan_rx_us(8 LE)][data_payload]
 *
 * `lan_rx_us` is the *absolute* LAN-MCU `esp_timer_get_time()` reading the
 * moment the wireless module callback received the data. The WAN MCU
 * converts via `bench_time_sync_from_peer_us()` and computes total internal
 * E2E as `(wan_now_us − converted_lan_rx_us)`. Zero = not measured.
 */
typedef struct __attribute__((packed)) {
  uint16_t header;         // 0x4454 (DT)
  uint8_t handler_type[3]; // "CAN", "LOR", "ZIG", "RS4"
  uint16_t data_length;    // Length of data payload
  uint64_t lan_rx_us;      // LE — absolute LAN esp_timer_get_time() at RX
  uint8_t data_payload[];  // Flexible array for payload
} data_packet_t;

/**
 * @brief ACK Packet
 * Format: [ACK][ack_type][extra_data...]
 */
typedef struct __attribute__((packed)) {
  uint8_t ack_prefix;   // 0x02 (ACK)
  uint8_t ack_type;     // ACK_TYPE_*
  uint8_t extra_data[]; // Optional extra data
} ack_packet_t;

/**
 * @brief Config Query Request
 * Format: [CF][CQ]
 */
typedef struct __attribute__((packed)) {
  uint16_t header;   // 0x4346 (CF)
  uint8_t prefix[2]; // "CQ"
} config_query_request_t;

/**
 * @brief Time-Sync Request (LAN master → WAN slave)
 * Format: [CF][0x09][lan_t1_us(8 LE)]
 *
 * LAN captures `lan_t1_us = esp_timer_get_time()` immediately before this
 * frame is handed to the SPI framer. The WAN side captures `wan_t2_us`
 * on RX and `wan_t3_us` right before loading the response.
 */
typedef struct __attribute__((packed)) {
  uint16_t header;    // 0x4346 (CF)
  uint8_t  cmd;       // FRAME_TYPE_TSYNC_REQ = 0x09
  uint64_t lan_t1_us; // LAN esp_timer_get_time() at frame dispatch
} tsync_request_t;

/**
 * @brief Time-Sync Response (WAN slave → LAN master)
 * Format: [ACK][0x1A][lan_t1_us(8)][wan_t2_us(8)][wan_t3_us(8)]
 *
 * LAN captures T4 on receipt. Offset = ((T2 − T1) + (T3 − T4)) / 2.
 */
typedef struct __attribute__((packed)) {
  uint8_t  ack_prefix; // 0x02
  uint8_t  ack_type;   // ACK_TYPE_TSYNC_RSP = 0x1A
  uint64_t lan_t1_us;  // echoed back from request
  uint64_t wan_t2_us;  // WAN esp_timer_get_time() on RX
  uint64_t wan_t3_us;  // WAN esp_timer_get_time() right before reply load
} tsync_response_t;

/**
 * @brief Config Query Response
 * Format: [CQ][length(2)][key=value|key=value|...]
 */
typedef struct __attribute__((packed)) {
  uint16_t header;         // 0x4351 (CQ)
  uint16_t config_length;  // Length of config string
  uint8_t config_string[]; // key=value pairs separated by |
} config_query_response_t;

// ===== Size Definitions =====
#define HANDSHAKE_REQUEST_SIZE sizeof(handshake_request_t)
#define HANDSHAKE_RESPONSE_SIZE sizeof(handshake_response_t)
#define RTC_RESPONSE_SIZE sizeof(rtc_config_response_t)
#define DATA_PACKET_HEADER_SIZE                                                \
  (2 + 3 + 2 + 8)                  // header + handler_type + data_length + lan_rx_us
#define CONFIG_HEADER_SIZE (2 + 2) // header + config_length
#define ACK_PACKET_MIN_SIZE 2      // ack_prefix + ack_type

// ===== Firmware Version Macros =====
#define FW_VERSION_MAKE(major, minor, patch, build)                            \
  (((uint32_t)(major) << 24) | ((uint32_t)(minor) << 16) |                     \
   ((uint32_t)(patch) << 8) | ((uint32_t)(build)))

#define FW_VERSION_MAJOR(ver) (((ver) >> 24) & 0xFF)
#define FW_VERSION_MINOR(ver) (((ver) >> 16) & 0xFF)
#define FW_VERSION_PATCH(ver) (((ver) >> 8) & 0xFF)
#define FW_VERSION_BUILD(ver) ((ver) & 0xFF)

// ===== Helper Functions =====

/**
 * @brief Convert handler ID enum to 3-byte string
 * @param id Handler ID enum
 * @return Pointer to 3-byte string (not null-terminated)
 */
static inline const char *handler_id_to_string(handler_id_t id) {
  switch (id) {
  case HANDLER_CAN:
    return HANDLER_TYPE_CAN;
  case HANDLER_LORA:
    return HANDLER_TYPE_LOR;
  case HANDLER_ZIGBEE:
    return HANDLER_TYPE_ZIG;
  case HANDLER_RS485:
    return HANDLER_TYPE_RS4;
  case HANDLER_BLE:
    return HANDLER_TYPE_BLE;
  case HANDLER_BLE_NATIVE:
    return HANDLER_TYPE_BLN;
  case HANDLER_BLE_GATT:
    return HANDLER_TYPE_BLG;
  case HANDLER_BENCH:
    return HANDLER_TYPE_BENCH;
  default:
    return "UNK";
  }
}

/**
 * @brief Convert 3-byte handler string to enum
 * @param str Handler type string (3 bytes)
 * @return Handler ID enum
 */
static inline handler_id_t handler_string_to_id(const uint8_t *str) {
  if (!str)
    return HANDLER_UNKNOWN;
  if (str[0] == 'C' && str[1] == 'A' && str[2] == 'N')
    return HANDLER_CAN;
  if (str[0] == 'L' && str[1] == 'O' && str[2] == 'R')
    return HANDLER_LORA;
  if (str[0] == 'Z' && str[1] == 'I' && str[2] == 'G')
    return HANDLER_ZIGBEE;
  if (str[0] == 'R' && str[1] == 'S' && str[2] == '4')
    return HANDLER_RS485;
  if (str[0] == 'B' && str[1] == 'L' && str[2] == 'E')
    return HANDLER_BLE;
  if (str[0] == 'B' && str[1] == 'L' && str[2] == 'N')
    return HANDLER_BLE_NATIVE;
  if (str[0] == 'B' && str[1] == 'N' && str[2] == 'C')
    return HANDLER_BENCH;
  return HANDLER_UNKNOWN;
}

#endif // FRAME_TYPES_H
