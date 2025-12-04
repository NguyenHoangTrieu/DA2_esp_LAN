#ifndef LORA_TDMA_H
#define LORA_TDMA_H

#include "lora_e32_comm.h"   // Provides lora_comm_handle_t and E32_TRANSPARENT_MAX_SIZE
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Broadcast address for one-hop star topology */
#define LORA_TDMA_ADDR_BROADCAST 0xFFFF

/* TDMA header: type(1) + slot(1) + src(2) + dst(2) + len(1) = 7 bytes */
#define LORA_TDMA_HEADER_SIZE   7u

/* E32 transparent max is 58 bytes. Keep TDMA frame <= 58 bytes. */
#define LORA_TDMA_MAX_PAYLOAD  (E32_TRANSPARENT_MAX_SIZE - LORA_TDMA_HEADER_SIZE)

/* Node role in the star topology (no routing). */
typedef enum {
    LORA_TDMA_ROLE_GATEWAY = 0,
    LORA_TDMA_ROLE_SENSOR  = 1
} lora_tdma_role_t;

/* Simple frame types for TDMA link layer. */
typedef enum {
    LORA_TDMA_FRAME_BEACON    = 0x01,  /* gateway -> all sensors, time sync      */
    LORA_TDMA_FRAME_DATA_UP   = 0x02,  /* sensor -> gateway                       */
    LORA_TDMA_FRAME_DATA_DOWN = 0x03   /* gateway -> sensor                       */
    /* You can extend with ACK/CONTROL later if needed. */
} lora_tdma_frame_type_t;

/* One-hop TDMA frame (on-air payload). */
typedef struct {
    uint8_t  type;                         /* lora_tdma_frame_type_t                */
    uint8_t  slot_id;                      /* TDMA slot index                       */
    uint16_t src_id;                       /* source node ID                        */
    uint16_t dst_id;                       /* destination node ID                   */
    uint8_t  len;                          /* payload length in bytes               */
    uint8_t  payload[LORA_TDMA_MAX_PAYLOAD];
} lora_tdma_frame_t;

/* Static TDMA configuration for one node. */
typedef struct {
    lora_tdma_role_t role;                 /* gateway or sensor                     */

    uint16_t node_id;                      /* this node ID                          */
    uint16_t gateway_id;                   /* gateway ID (for sensors)              */

    uint8_t  num_slots;                    /* number of slots per frame             */
    uint8_t  my_slot;                      /* sensor TX slot; gateway can ignore    */

    uint32_t slot_duration_ms;             /* slot duration in milliseconds         */
    uint32_t frame_start_ms;               /* initial frame start (only gateway)    */
} lora_tdma_config_t;

/* Basic statistics for debugging and monitoring. */
typedef struct {
    uint32_t tx_ok;                        /* successfully transmitted frames       */
    uint32_t rx_ok;                        /* successfully parsed frames            */
    uint32_t rx_error;                     /* length / parse errors                 */
    uint32_t missed_beacon;               /* beacons not received (for sensors)    */
} lora_tdma_stats_t;

/* Application callback when a frame addressed to this node is received. */
typedef void (*lora_tdma_rx_cb_t)(const lora_tdma_frame_t *frame);

/* Application callback when entering a new slot. */
typedef void (*lora_tdma_slot_cb_t)(uint8_t slot_id, bool is_tx_slot);

/* TDMA runtime context. Allocate one per node (gateway or sensor). */
typedef struct {
    /* Static config + statistics */
    lora_tdma_config_t cfg;
    lora_tdma_stats_t  stats;

    /* Radio driver handle (from lora_comm_init) */
    lora_comm_handle_t radio;

    /* TDMA timing state */
    uint8_t  current_slot;                 /* current slot index in frame           */
    uint32_t frame_start_ms;               /* local frame start timestamp           */
    uint32_t frame_counter;                /* frame index (0,1,2,...)               */

    bool     started;                      /* true after TDMA timing is running     */
    bool     is_synced;                    /* sensors set true after first beacon   */
    bool     beacon_sent;                  /* gateway: beacon already sent this frame */

    /* Simple single-frame TX buffer */
    bool              tx_pending;
    lora_tdma_frame_t tx_frame;

    /* Application callbacks */
    lora_tdma_rx_cb_t   rx_cb;
    lora_tdma_slot_cb_t slot_cb;
} lora_tdma_ctx_t;

/* ====================== API ====================== */

/**
 * @brief Initialize TDMA context.
 *
 * This does NOT start any task or timer. You must call lora_tdma_process()
 * periodically with current time in milliseconds.
 *
 * @param ctx   Pointer to TDMA context
 * @param cfg   Static configuration (copied into ctx)
 * @param radio LoRa radio handle returned by lora_comm_init()
 */
void lora_tdma_init(lora_tdma_ctx_t *ctx,
                    const lora_tdma_config_t *cfg,
                    lora_comm_handle_t radio);

/**
 * @brief Register RX callback for application.
 */
void lora_tdma_register_rx_callback(lora_tdma_ctx_t *ctx,
                                    lora_tdma_rx_cb_t cb);

/**
 * @brief Register slot callback for application.
 *
 * Called each time TDMA enters a new slot.
 */
void lora_tdma_register_slot_callback(lora_tdma_ctx_t *ctx,
                                      lora_tdma_slot_cb_t cb);

/**
 * @brief Main TDMA scheduler.
 *
 * Call this periodically (e.g. every 1–10 ms) from your main loop or RTOS task.
 * 'now_ms' must use the same time base as cfg.frame_start_ms.
 */
void lora_tdma_process(lora_tdma_ctx_t *ctx, uint32_t now_ms);

/**
 * @brief Sensor: queue an uplink frame to gateway.
 *
 * The frame will be sent automatically when the node is in its TX slot.
 *
 * @return true if frame was queued, false otherwise
 *         (e.g. invalid role, length too big, or previous frame pending).
 */
bool lora_tdma_sensor_send(lora_tdma_ctx_t   *ctx,
                           const uint8_t     *payload,
                           uint8_t            len);

/**
 * @brief Gateway: queue a downlink frame to a specific sensor.
 *
 * The frame will be sent in the current slot as soon as possible.
 */
bool lora_tdma_gateway_send_to(lora_tdma_ctx_t *ctx,
                               uint16_t         dst_id,
                               const uint8_t   *payload,
                               uint8_t          len);

/**
 * @brief Gateway-only: send a beacon immediately (slot 0 recommended).
 *
 * Usually this is called automatically by lora_tdma_process() when gateway
 * enters slot 0 of a new frame, but you can call it manually if needed.
 */
bool lora_tdma_send_beacon(lora_tdma_ctx_t *ctx);

/**
 * @brief Feed received raw bytes from radio into TDMA layer.
 *
 * Call this from your radio RX task:
 *   - Receive bytes from lora_comm_receive()
 *   - Pass them to this function along with reception timestamp.
 */
void lora_tdma_handle_rx(lora_tdma_ctx_t *ctx,
                         const uint8_t   *buf,
                         uint8_t          len,
                         uint32_t         now_ms);

/* Stats helpers */
void lora_tdma_get_stats (lora_tdma_ctx_t *ctx, lora_tdma_stats_t *out);
void lora_tdma_reset_stats(lora_tdma_ctx_t *ctx);

#ifdef __cplusplus
}
#endif

#endif /* LORA_TDMA_H */
