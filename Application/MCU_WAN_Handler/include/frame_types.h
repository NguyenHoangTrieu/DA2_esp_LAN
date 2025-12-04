/**
 * @file frame_types.h
 * @brief Common frame types and data structures for LAN-WAN communication
 */
#ifndef FRAME_TYPES_H
#define FRAME_TYPES_H

#include <stdint.h>
#include <stdbool.h>

// ===== Frame Type Prefixes =====
#define PREFIX_RTC      "RT"    // RTC Config Response
#define PREFIX_CONFIG   "CF"    // Configuration Data
#define PREFIX_DATA     "DT"    // Data Packet

// ===== Handler Type Identifiers =====
#define HANDLER_TYPE_CAN  "CAN"
#define HANDLER_TYPE_LOR  "LOR"  // LoRa
#define HANDLER_TYPE_ZIG  "ZIG"  // ZigBee

// ===== Frame Types (Single Byte) =====
typedef enum {
    FRAME_TYPE_HANDSHAKE = 0x01,
    FRAME_TYPE_ACK       = 0x02,
    FRAME_TYPE_RTC       = 0x03,  // "RT" prefix
    FRAME_TYPE_CONFIG    = 0x04,  // "CF" prefix
    FRAME_TYPE_DATA      = 0x05,  // "DT" prefix
    FRAME_TYPE_FOTA      = 0x06,  // CFFW - Firmware Update
} frame_type_t;

// ===== ACK Types =====
typedef enum {
    ACK_TYPE_HANDSHAKE      = 0x10,
    ACK_TYPE_RECEIVED_OK    = 0x11,
    ACK_TYPE_INTERNET_OK    = 0x12,
    ACK_TYPE_NO_INTERNET    = 0x13,
    ACK_TYPE_OK             = 0x14,
    ACK_TYPE_TIMEOUT        = 0x15,
} ack_type_t;

// ===== Handler ID =====
typedef enum {
    HANDLER_CAN   = 0x01,
    HANDLER_LORA  = 0x02,
    HANDLER_ZIGBEE = 0x03,
} handler_id_t;

// ===== Data Structures per Specification =====

/**
 * @brief RTC Config Response (Prefix "RT")
 * Format: [RT][dd/mm/yyyy-hh:mm:ss][network_status]
 */
typedef struct __attribute__((packed)) {
    uint8_t prefix[2];          // "RT"
    char rtc_string[20];        // "dd/mm/yyyy-hh:mm:ss\0"
    uint8_t network_status;     // 1 = connected, 0 = disconnected
} rtc_config_response_t;

/**
 * @brief Config Data Packet (Prefix "CF")
 * Format: [CF][config_data][config_length]
 */
typedef struct __attribute__((packed)) {
    uint8_t prefix[2];          // "CF"
    uint16_t config_length;     // Length of config data
    uint8_t config_data[];      // Flexible array for config data
} config_data_t;

/**
 * @brief Data Packet (Prefix "DT")
 * Format: [DT][handler_type(3)][data_length][data_payload]
 */
typedef struct __attribute__((packed)) {
    uint8_t prefix[2];          // "DT"
    uint8_t handler_type[3];    // "CAN", "LOR", "ZIG"
    uint16_t data_length;       // Length of data payload
    uint8_t data_payload[];     // Flexible array for payload
} data_packet_t;

// ===== Size Definitions =====
#define RTC_RESPONSE_SIZE       sizeof(rtc_config_response_t)
#define DATA_PACKET_HEADER_SIZE (2 + 3 + 2)  // prefix + handler_type + data_length
#define CONFIG_HEADER_SIZE      (2 + 2)      // prefix + config_length

#endif // FRAME_TYPES_H
