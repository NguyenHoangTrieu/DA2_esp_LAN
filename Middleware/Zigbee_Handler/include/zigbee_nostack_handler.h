/**
 * @file zigbee_nostack_handler.h
 * @brief Zigbee No-Stack Handler API
 *
 * Provides simple API for Zigbee communication without mesh networking.
 * No RTOS inside - pure logic only.
 * Similar to lora_tdma_handler.h architecture.
 */

#ifndef ZIGBEE_NOSTACK_HANDLER_H
#define ZIGBEE_NOSTACK_HANDLER_H

#include "zigbee_cc_comm.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ===== Configuration ===== */

/**
 * @brief Maximum payload size for Zigbee frame
 */
#define ZIGBEE_NOSTACK_MAX_PAYLOAD (1024)

/**
 * @brief Frame header size
 * [PREAMBLE(1)][SRC(2)][DST(2)][LEN(1)] = 6 bytes
 */
#define ZIGBEE_NOSTACK_HEADER_SIZE (6)

/**
 * @brief Broadcast address
 */
#define ZIGBEE_NOSTACK_ADDR_BROADCAST (0x0000)

/* ===== Frame Types ===== */

/**
 * @brief Zigbee frame structure
 */
typedef struct {
  uint8_t preamble;                            /**< Frame preamble (0xA5) */
  uint16_t src_id;                             /**< Source node ID */
  uint16_t dst_id;                             /**< Destination node ID */
  uint16_t len;                                 /**< Payload length */
  uint8_t payload[ZIGBEE_NOSTACK_MAX_PAYLOAD]; /**< Payload data */
} zigbee_nostack_frame_t;

/* ===== Statistics ===== */

/**
 * @brief Handler statistics
 */
typedef struct {
  uint32_t tx_ok;     /**< Successfully sent frames */
  uint32_t rx_ok;     /**< Successfully received frames */
  uint32_t rx_error;  /**< RX parse/CRC errors */
  uint32_t crc_error; /**< CRC mismatch errors */
} zigbee_nostack_stats_t;

/* ===== Configuration ===== */

/**
 * @brief Handler configuration
 */
typedef struct {
  uint16_t node_id;    /**< This node's ID */
  uint16_t gateway_id; /**< Gateway ID (for nodes) */
} zigbee_nostack_config_t;

/**
 * @brief Global default configuration
 * Application can modify before calling zigbee_nostack_handler_init()
 */
extern zigbee_nostack_config_t g_zigbee_nostack_cfg;

/* ===== Context Structure ===== */

struct zigbee_nostack_handler_ctx_s;
typedef struct zigbee_nostack_handler_ctx_s zigbee_nostack_handler_ctx_t;

/**
 * @brief RX callback function type
 */
typedef void (*zigbee_nostack_rx_cb_t)(const zigbee_nostack_frame_t *frame);

/**
 * @brief Handler context (internal state)
 */
struct zigbee_nostack_handler_ctx_s {
  zigbee_nostack_config_t cfg;   /**< Configuration */
  zigbee_nostack_stats_t stats;  /**< Statistics */
  zigbee_cc_comm_handle_t radio; /**< CC2530 UART handle */
  zigbee_nostack_rx_cb_t rx_cb;  /**< RX callback */
};

/* ===== API Functions ===== */

/**
 * @brief Initialize handler context
 *
 * @param ctx Pointer to context (allocated by caller)
 * @param radio CC2530 communication handle
 */
void zigbee_nostack_handler_init(zigbee_nostack_handler_ctx_t *ctx,
                                 zigbee_cc_comm_handle_t radio);

/**
 * @brief Register RX callback
 *
 * @param ctx Pointer to context
 * @param cb Callback function
 */
void zigbee_nostack_handler_register_rx_callback(
    zigbee_nostack_handler_ctx_t *ctx, zigbee_nostack_rx_cb_t cb);

/**
 * @brief Send data frame
 *
 * @param ctx Pointer to context
 * @param dst_id Destination node ID (ZIGBEE_NOSTACK_ADDR_BROADCAST for
 * broadcast)
 * @param payload Payload data
 * @param len Payload length (max ZIGBEE_NOSTACK_MAX_PAYLOAD)
 * @return true on success, false on error
 */
bool zigbee_nostack_handler_send(zigbee_nostack_handler_ctx_t *ctx,
                                 uint16_t dst_id, const uint8_t *payload,
                                 uint16_t len);

/**
 * @brief Handle received raw bytes from CC2530
 *
 * Call this when data is available from UART.
 * Parses frame and calls RX callback if valid.
 *
 * @param ctx Pointer to context
 * @param buf Raw buffer
 * @param len Buffer length
 */
void zigbee_nostack_handler_handle_rx(zigbee_nostack_handler_ctx_t *ctx,
                                      const uint8_t *buf, uint8_t len);

/**
 * @brief Get statistics snapshot
 *
 * @param ctx Pointer to context
 * @param out Output statistics structure
 */
void zigbee_nostack_handler_get_stats(zigbee_nostack_handler_ctx_t *ctx,
                                      zigbee_nostack_stats_t *out);

/**
 * @brief Reset statistics counters
 *
 * @param ctx Pointer to context
 */
void zigbee_nostack_handler_reset_stats(zigbee_nostack_handler_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* ZIGBEE_NOSTACK_HANDLER_H */
