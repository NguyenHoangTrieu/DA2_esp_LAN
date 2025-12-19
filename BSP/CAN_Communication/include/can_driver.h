/**
 * @file can_driver.h
 * @brief Lightweight ESP32-S3 TWAI CAN Driver Library (ESP-IDF v6.0)
 * @note Uses new esp_twai and esp_twai_onchip API
 */

#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "stack_handler.h"

/* ===== Hardware Configuration - Dual Stack Support ===== */

// Stack 1 configuration (TWAI)
#define CAN_TWAI_TX_PIN_STACK_1     17
#define CAN_TWAI_RX_PIN_STACK_1     18

// Stack 2 configuration (TWAI)
#define CAN_TWAI_TX_PIN_STACK_2     15
#define CAN_TWAI_RX_PIN_STACK_2     16

/* ===== Default Configuration ===== */
#define CAN_DEFAULT_BITRATE         500000  // 500 kbps

#define MAX_WHITELISTED_IDS 500
/* Return Status Codes */
typedef enum {
  CAN_OK = 0,
  CAN_ERR_INVALID_CONFIG,
  CAN_ERR_DRIVER_INSTALL,
  CAN_ERR_DRIVER_START,
  CAN_ERR_TX_FAILED,
  CAN_ERR_TX_TIMEOUT,
  CAN_ERR_RX_NO_DATA,
  CAN_ERR_INVALID_PARAM,
  CAN_ERR_BUS_OFF,
  CAN_ERR_NOT_INITIALIZED
} can_status_t;

/* Operating Modes */
typedef enum {
  CAN_MODE_NORMAL = 0,
  CAN_MODE_NO_ACK,  // Self-test mode (no ACK required)
  CAN_MODE_LOOPBACK // Listen-only mode
} can_operating_mode_t;

/* CAN Bus States */
typedef enum {
  CAN_BUS_RUNNING = 0,
  CAN_BUS_WARNING,
  CAN_BUS_ERROR_PASSIVE,
  CAN_BUS_BUS_OFF,
  CAN_BUS_RECOVERING,
  CAN_BUS_UNKNOWN
} can_bus_state_t;

/* CAN Message Structure */
typedef struct {
  uint16_t id;     // Standard 11-bit ID
  uint8_t data[8]; // Data bytes
  uint8_t len;     // Data Length Code (0-8)
  bool rtr;        // Remote Transmission Request flag
} can_message_t;

/* CAN Hardware Configuration Structure (REQ-DAT-001) */
typedef struct {
  uint32_t baud_rate;
  can_operating_mode_t operating_mode;
} can_config_t;

/* External Global Configuration (REQ-DAT-001) */
extern can_config_t g_can_config;

/* External Global Whitelist Configuration (REQ-DAT-002, REQ-DAT-003) */
extern uint16_t g_can_whitelist[];
extern uint16_t g_can_whitelist_count;

/**
 * @brief Initialize and configure the TWAI CAN driver
 * @return can_status_t status code
 */
can_status_t can_driver_init(void);

/**
 * @brief Transmit a CAN message (Standard Frame, non-blocking)
 * @param id Standard 11-bit CAN ID
 * @param data Pointer to data buffer
 * @param len Data length (0-8 bytes)
 * @return can_status_t status code
 */
can_status_t can_transmit(uint16_t id, const uint8_t *data, uint8_t len);

/**
 * @brief Receive a CAN message (polling, non-blocking)
 * @param msg Pointer to message structure to fill
 * @return can_status_t CAN_OK if message received, CAN_ERR_RX_NO_DATA if no
 * data
 */
can_status_t can_receive(can_message_t *msg);

/**
 * @brief Check current TWAI bus status
 * @return can_bus_state_t current bus state
 */
can_bus_state_t can_check_bus_status(void);

/**
 * @brief Initiate recovery from Bus-Off state
 * @return can_status_t status code
 */
can_status_t can_initiate_recovery(void);

/**
 * @brief Deinitialize the CAN driver
 * @return can_status_t status code
 */
can_status_t can_driver_deinit(void);

#endif // CAN_DRIVER_H
