/**
 * @file e32_config.h
 * @brief E32 LoRa Module Configuration Definitions
 *
 * Based on E32-433T30D User Manual v1.9
 * SX1278 433MHz LoRa Module
 */

#ifndef E32_CONFIG_H
#define E32_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ===== E32 Module Constants =====
#define E32_DEFAULT_FREQUENCY_MHZ 433
#define E32_MIN_FREQUENCY_MHZ 410
#define E32_MAX_FREQUENCY_MHZ 441
#define E32_MAX_PACKET_SIZE 512
#define E32_TRANSPARENT_MAX_SIZE 58

// ===== Command Prefixes =====
#define E32_CMD_SET_PARAM_SAVE 0xC0 // Save parameters to flash
#define E32_CMD_READ_PARAM 0xC1     // Read current parameters
#define E32_CMD_SET_PARAM_TEMP 0xC2 // Set parameters (no save)
#define E32_CMD_READ_VERSION 0xC3   // Read version info
#define E32_CMD_RESET 0xC4          // Reset module

// ===== Operating Modes =====
typedef enum {
  E32_MODE_NORMAL = 0,       // M1=0, M0=0: Normal mode (TX/RX)
  E32_MODE_WAKEUP = 1,       // M1=0, M0=1: Wake-up mode (with preamble)
  E32_MODE_POWER_SAVING = 2, // M1=1, M0=0: Power-saving mode
  E32_MODE_SLEEP = 3         // M1=1, M0=1: Sleep/Config mode
} e32_mode_t;

// ===== UART Parity =====
typedef enum {
  E32_UART_PARITY_8N1 = 0b00,
  E32_UART_PARITY_8O1 = 0b01,
  E32_UART_PARITY_8E1 = 0b10,
  E32_UART_PARITY_8N1_ALT = 0b11 // Same as 8N1
} e32_uart_parity_t;

// ===== UART Baud Rate =====
typedef enum {
  E32_UART_BAUD_1200 = 0b000,
  E32_UART_BAUD_2400 = 0b001,
  E32_UART_BAUD_4800 = 0b010,
  E32_UART_BAUD_9600 = 0b011, // Default
  E32_UART_BAUD_19200 = 0b100,
  E32_UART_BAUD_38400 = 0b101,
  E32_UART_BAUD_57600 = 0b110,
  E32_UART_BAUD_115200 = 0b111
} e32_uart_baud_t;

// ===== Air Data Rate =====
typedef enum {
  E32_AIR_RATE_0_3K = 0b000, // 0.3 kbps
  E32_AIR_RATE_1_2K = 0b001, // 1.2 kbps
  E32_AIR_RATE_2_4K = 0b010, // 2.4 kbps (Default)
  E32_AIR_RATE_4_8K = 0b011, // 4.8 kbps
  E32_AIR_RATE_9_6K = 0b100, // 9.6 kbps
  E32_AIR_RATE_19_2K = 0b101 // 19.2 kbps
} e32_air_rate_t;

// ===== Transmission Power =====
typedef enum {
  E32_POWER_30DBM = 0b00, // 30 dBm (1W) - Default
  E32_POWER_27DBM = 0b01, // 27 dBm (~500mW)
  E32_POWER_24DBM = 0b10, // 24 dBm (~250mW)
  E32_POWER_21DBM = 0b11  // 21 dBm (~125mW)
} e32_tx_power_t;

// ===== Transmission Mode =====
typedef enum {
  E32_TRANS_TRANSPARENT = 0, // Transparent transmission
  E32_TRANS_FIXED = 1        // Fixed transmission (with address)
} e32_transmission_mode_t;

// ===== IO Drive Mode =====
typedef enum {
  E32_IO_OPEN_COLLECTOR = 0, // TXD/AUX open-collector, RXD open-collector
  E32_IO_PUSH_PULL = 1       // TXD/AUX push-pull, RXD pull-up (Default)
} e32_io_drive_mode_t;

// ===== Wireless Wake-up Time =====
typedef enum {
  E32_WAKEUP_250MS = 0b000,  // 250ms (Default)
  E32_WAKEUP_500MS = 0b001,  // 500ms
  E32_WAKEUP_750MS = 0b010,  // 750ms
  E32_WAKEUP_1000MS = 0b011, // 1000ms
  E32_WAKEUP_1250MS = 0b100, // 1250ms
  E32_WAKEUP_1500MS = 0b101, // 1500ms
  E32_WAKEUP_1750MS = 0b110, // 1750ms
  E32_WAKEUP_2000MS = 0b111  // 2000ms
} e32_wakeup_time_t;

// ===== FEC (Forward Error Correction) =====
typedef enum {
  E32_FEC_OFF = 0, // FEC disabled (higher speed, shorter range)
  E32_FEC_ON = 1   // FEC enabled (lower speed, longer range) - Default
} e32_fec_t;

// ===== Parameter Structure =====
typedef struct __attribute__((packed)) {
  uint8_t head;   // Command header (0xC0 or 0xC2)
  uint8_t addh;   // Address high byte (0x00-0xFF)
  uint8_t addl;   // Address low byte (0x00-0xFF)
  uint8_t sped;   // UART & Air settings
  uint8_t chan;   // Channel (0x00-0x1F)
  uint8_t option; // Transmission options
} e32_params_t;

// ===== Version Information =====
typedef struct {
  uint8_t model;    // Product model
  uint8_t version;  // Firmware version
  uint8_t features; // Interface type & max power
} e32_version_t;

// ===== SPED Byte Bit Fields =====
#define E32_SPED_PARITY_SHIFT 6
#define E32_SPED_PARITY_MASK 0xC0
#define E32_SPED_UART_BAUD_SHIFT 3
#define E32_SPED_UART_BAUD_MASK 0x38
#define E32_SPED_AIR_RATE_SHIFT 0
#define E32_SPED_AIR_RATE_MASK 0x07

// ===== OPTION Byte Bit Fields =====
#define E32_OPTION_TRANS_MODE_SHIFT 7
#define E32_OPTION_TRANS_MODE_MASK 0x80
#define E32_OPTION_IO_DRIVE_SHIFT 6
#define E32_OPTION_IO_DRIVE_MASK 0x40
#define E32_OPTION_WAKEUP_SHIFT 3
#define E32_OPTION_WAKEUP_MASK 0x38
#define E32_OPTION_FEC_SHIFT 2
#define E32_OPTION_FEC_MASK 0x04
#define E32_OPTION_POWER_SHIFT 0
#define E32_OPTION_POWER_MASK 0x03

// ===== Helper Macros =====
#define E32_BUILD_ADDRESS(high, low) (((uint16_t)(high) << 8) | (low))
#define E32_GET_ADDH(addr) ((uint8_t)((addr) >> 8))
#define E32_GET_ADDL(addr) ((uint8_t)((addr) & 0xFF))

// ===== Special Addresses =====
#define E32_ADDR_BROADCAST 0xFFFF // Broadcast address
#define E32_ADDR_MONITOR 0xFFFF   // Monitor address

// ===== Timing Constants (ms) =====
#define E32_RESET_TIME_MS 50
#define E32_MODE_SWITCH_TIME_MS 2
#define E32_AUX_HIGH_TIME_MS 2
#define E32_CONFIG_COMMAND_TIMEOUT_MS 1000
#define E32_BYTE_TIME_MS 10 // Time for 3-byte termination

// ===== Default Configuration =====
#define E32_DEFAULT_ADDH 0x00
#define E32_DEFAULT_ADDL 0x00
#define E32_DEFAULT_CHANNEL 0x17 // 433MHz
#define E32_DEFAULT_UART_PARITY E32_UART_PARITY_8N1
#define E32_DEFAULT_UART_BAUD E32_UART_BAUD_9600
#define E32_DEFAULT_AIR_RATE E32_AIR_RATE_2_4K
#define E32_DEFAULT_TRANS_MODE E32_TRANS_TRANSPARENT
#define E32_DEFAULT_IO_DRIVE E32_IO_PUSH_PULL
#define E32_DEFAULT_WAKEUP_TIME E32_WAKEUP_250MS
#define E32_DEFAULT_FEC E32_FEC_ON
#define E32_DEFAULT_TX_POWER E32_POWER_30DBM

// ===== Helper Functions =====

/**
 * @brief Build SPED byte from individual settings
 */
static inline uint8_t e32_build_sped(e32_uart_parity_t parity,
                                     e32_uart_baud_t baud,
                                     e32_air_rate_t air_rate) {
  return ((parity << E32_SPED_PARITY_SHIFT) & E32_SPED_PARITY_MASK) |
         ((baud << E32_SPED_UART_BAUD_SHIFT) & E32_SPED_UART_BAUD_MASK) |
         ((air_rate << E32_SPED_AIR_RATE_SHIFT) & E32_SPED_AIR_RATE_MASK);
}

/**
 * @brief Build OPTION byte from individual settings
 */
static inline uint8_t e32_build_option(e32_transmission_mode_t trans_mode,
                                       e32_io_drive_mode_t io_drive,
                                       e32_wakeup_time_t wakeup, e32_fec_t fec,
                                       e32_tx_power_t power) {
  return ((trans_mode << E32_OPTION_TRANS_MODE_SHIFT) &
          E32_OPTION_TRANS_MODE_MASK) |
         ((io_drive << E32_OPTION_IO_DRIVE_SHIFT) & E32_OPTION_IO_DRIVE_MASK) |
         ((wakeup << E32_OPTION_WAKEUP_SHIFT) & E32_OPTION_WAKEUP_MASK) |
         ((fec << E32_OPTION_FEC_SHIFT) & E32_OPTION_FEC_MASK) |
         ((power << E32_OPTION_POWER_SHIFT) & E32_OPTION_POWER_MASK);
}

/**
 * @brief Extract UART parity from SPED byte
 */
static inline e32_uart_parity_t e32_get_parity(uint8_t sped) {
  return (e32_uart_parity_t)((sped & E32_SPED_PARITY_MASK) >>
                             E32_SPED_PARITY_SHIFT);
}

/**
 * @brief Extract UART baud rate from SPED byte
 */
static inline e32_uart_baud_t e32_get_uart_baud(uint8_t sped) {
  return (e32_uart_baud_t)((sped & E32_SPED_UART_BAUD_MASK) >>
                           E32_SPED_UART_BAUD_SHIFT);
}

/**
 * @brief Extract air data rate from SPED byte
 */
static inline e32_air_rate_t e32_get_air_rate(uint8_t sped) {
  return (e32_air_rate_t)((sped & E32_SPED_AIR_RATE_MASK) >>
                          E32_SPED_AIR_RATE_SHIFT);
}

/**
 * @brief Extract transmission mode from OPTION byte
 */
static inline e32_transmission_mode_t e32_get_trans_mode(uint8_t option) {
  return (e32_transmission_mode_t)((option & E32_OPTION_TRANS_MODE_MASK) >>
                                   E32_OPTION_TRANS_MODE_SHIFT);
}

/**
 * @brief Extract TX power from OPTION byte
 */
static inline e32_tx_power_t e32_get_tx_power(uint8_t option) {
  return (e32_tx_power_t)((option & E32_OPTION_POWER_MASK) >>
                          E32_OPTION_POWER_SHIFT);
}

#ifdef __cplusplus
}
#endif

#endif // E32_CONFIG_H
