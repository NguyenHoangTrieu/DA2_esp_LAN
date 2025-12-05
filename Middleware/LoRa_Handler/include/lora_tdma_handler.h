#ifndef LORA_HANDLER_H
#define LORA_HANDLER_H

#include "lora_e32_comm.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ----- Basic limits (from E32 datasheet) ----- */
/* TDMA header: type(1) + slot(1) + src(2) + dst(2) + len(1) = 7 bytes */
#define LORA_HANDLER_HEADER_SIZE        7u
/* E32 transparent max is 58 bytes, payload is 58 - header */
#define LORA_HANDLER_MAX_PAYLOAD       (E32_TRANSPARENT_MAX_SIZE - LORA_HANDLER_HEADER_SIZE)

/* Simple software broadcast ID (logical addressing) */
#define LORA_HANDLER_ADDR_BROADCAST    0xFFFF

/* Max length of crypto key (simple XOR stream) */
#define LORA_HANDLER_CRYPTO_KEY_MAX_LEN 16u

/* How many TDMA frames until sensor loses sync without beacon */
#define LORA_HANDLER_BEACON_TIMEOUT_FRAMES  5u

/* ----- Roles & frame types ----- */

typedef enum {
    LORA_HANDLER_ROLE_GATEWAY = 0,
    LORA_HANDLER_ROLE_SENSOR  = 1
} lora_handler_role_t;

/* Very small frame type set */
typedef enum {
    LORA_HANDLER_FRAME_BEACON = 0x01,  /* time sync frame */
    LORA_HANDLER_FRAME_DATA   = 0x02   /* encrypted data frame */
} lora_handler_frame_type_t;

/* On-air TDMA frame (after crypto applied on payload) */
typedef struct {
    uint8_t  type;                     /* lora_handler_frame_type_t         */
    uint8_t  slot_id;                  /* TDMA slot index                   */
    uint16_t src_id;                   /* source node ID                    */
    uint16_t dst_id;                   /* destination node ID               */
    uint8_t  len;                      /* payload length in bytes           */
    uint8_t  payload[LORA_HANDLER_MAX_PAYLOAD]; /* (maybe encrypted)       */
} lora_handler_frame_t;

/* Basic stats for debugging */
typedef struct {
    uint32_t tx_ok;
    uint32_t rx_ok;
    uint32_t rx_error;
    uint32_t missed_beacon;
} lora_handler_stats_t;

/* Public TDMA configuration (shared & override-able by application) */
typedef struct {
    lora_handler_role_t role;          /* gateway or sensor                 */

    uint16_t node_id;                  /* this node's logical ID            */
    uint16_t gateway_id;               /* logical gateway ID (used by sensors) */

    uint8_t  num_slots;                /* number of slots per frame         */
    uint8_t  my_slot;                  /* TX slot for this node             */

    uint32_t slot_duration_ms;         /* slot duration in milliseconds     */
} lora_handler_config_t;

/* Public globals for configuration & crypto key.
 * Define them once in lora_handler.c, and you can modify them
 * from application code before calling lora_handler_init().
 */

/* Default TDMA configuration (no hardcoded values in logic). */
extern lora_handler_config_t g_lora_handler_cfg;

/* Default crypto key (XOR stream). Only first g_lora_handler_crypto_key_len bytes are used.
 */
extern uint8_t g_lora_handler_crypto_key[LORA_HANDLER_CRYPTO_KEY_MAX_LEN];
extern uint8_t g_lora_handler_crypto_key_len;

/* ----- Forward declarations for callbacks & context ----- */

struct lora_handler_ctx_s;
typedef struct lora_handler_ctx_s lora_handler_ctx_t;

/* Called when a valid, decrypted frame for this node is received. */
typedef void (*lora_handler_rx_cb_t)(const lora_handler_frame_t *frame);

/* Called each time TDMA enters a new slot. */
typedef void (*lora_handler_slot_cb_t)(uint8_t slot_id, bool is_tx_slot);

/* TDMA runtime context (one per node) */
struct lora_handler_ctx_s {
    lora_handler_config_t cfg;         /* local copy of config              */
    lora_handler_stats_t  stats;

    lora_e32_comm_handle_t radio;      /* underlying E32 driver handle      */

    /* TDMA timing state */
    uint8_t  current_slot;
    uint32_t frame_start_ms;
    uint32_t frame_counter;

    bool     started;
    bool     is_synced;                /* sensors: set true after beacon    */
    bool     beacon_sent;              /* gateway: beacon for this frame    */

    /* Simple TX buffer (single pending frame) */
    bool                tx_pending;
    lora_handler_frame_t tx_frame;

    /* Application callbacks */
    lora_handler_rx_cb_t   rx_cb;
    lora_handler_slot_cb_t slot_cb;
};

/* ====================== API ====================== */

/**
 * @brief Initialize handler context using global configuration & radio handle.
 * @param ctx   Pointer to context (allocated by caller)
 * @param radio LoRa E32 driver handle (from lora_e32_comm_init)
 */
void lora_handler_init(lora_handler_ctx_t *ctx, lora_e32_comm_handle_t radio);

/**
 * @brief Register RX callback for decrypted frames.
 */
void lora_handler_register_rx_callback(lora_handler_ctx_t *ctx,
                                       lora_handler_rx_cb_t cb);

/**
 * @brief Register slot callback, called at each slot boundary.
 */
void lora_handler_register_slot_callback(lora_handler_ctx_t *ctx,
                                         lora_handler_slot_cb_t cb);

/**
 * @brief Main TDMA scheduler. No RTOS.
 *
 * Call this from your main loop at a reasonable rate (e.g. every 1–10 ms).
 * 'now_ms' should be from a monotonic millisecond timer.
 */
void lora_handler_process(lora_handler_ctx_t *ctx, uint32_t now_ms);

/**
 * @brief Queue a data frame to send (will be encrypted and sent in TX slot).
 *
 * - For gateway: usually dst_id is a specific sensor or broadcast.
 * - For sensor: dst_id typically equals g_lora_handler_cfg.gateway_id.
 *
 * @return true if successfully queued, false if busy/invalid.
 */
bool lora_handler_send(lora_handler_ctx_t *ctx,
                       uint16_t             dst_id,
                       const uint8_t       *payload,
                       uint8_t              len);

/**
 * @brief Feed raw bytes received from E32 into handler.
 *
 * Typical flow in main loop:
 *   - call lora_e32_comm_receive() to get raw bytes
 *   - pass them here with the same 'now_ms' time base.
 */
void lora_handler_handle_rx(lora_handler_ctx_t *ctx,
                            const uint8_t       *buf,
                            uint8_t              len,
                            uint32_t             now_ms);

/**
 * @brief Get a snapshot of current stats.
 */
void lora_handler_get_stats(lora_handler_ctx_t *ctx,
                            lora_handler_stats_t *out);

/**
 * @brief Reset statistics counters.
 */
void lora_handler_reset_stats(lora_handler_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* LORA_HANDLER_H */
