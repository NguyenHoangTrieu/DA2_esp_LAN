/**
 * @file lora_handler.c
 * @brief TDMA + simple crypto handler using LoRa E32 broadcast driver.
 *
 * - All modules use RF broadcast (E32 address 0xFFFF, same channel).
 * - TDMA in software with num_slots, slot_duration_ms.
 * - Frames carry src_id/dst_id for logical addressing.
 * - Payload is XOR-encrypted using a simple key g_lora_handler_crypto_key[].
 *
 * WARNING: XOR is NOT secure cryptography. This is only a lightweight
 * obfuscation for demo/embedded use. Do not use it for real security.
 */

#include "lora_tdma_handler.h"
#include <string.h>

/* ===== Global configuration defaults (can be modified by application) ===== */

/* Default TDMA configuration.
 * Application may change fields before calling lora_handler_init().
 */
lora_handler_config_t g_lora_handler_cfg = {.role = LORA_HANDLER_ROLE_SENSOR,
                                            .node_id = 0x0001,
                                            .gateway_id = 0x0001,
                                            .num_slots = 8,
                                            .my_slot = 0,
                                            .slot_duration_ms = 200};

/* Default crypto key: simple XOR stream.
 * Only first g_lora_handler_crypto_key_len bytes are used.
 * You can override content/length before lora_handler_init().
 */
uint8_t g_lora_handler_crypto_key[LORA_HANDLER_CRYPTO_KEY_MAX_LEN] = {
    0x10, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF,
    0x01, 0x22, 0x43, 0x64, 0x85, 0xA6, 0xC7, 0xE8};

uint8_t g_lora_handler_crypto_key_len = 16;

/* ===== Internal helpers ===== */

/* Simple XOR "cipher" (same for encrypt/decrypt).
 * data is modified in-place.
 */
static void lora_handler_xor_crypt(uint8_t *data, uint8_t len) {
  if (!data || len == 0) {
    return;
  }
  if (g_lora_handler_crypto_key_len == 0 ||
      g_lora_handler_crypto_key_len > LORA_HANDLER_CRYPTO_KEY_MAX_LEN) {
    /* If key length invalid, do nothing. */
    return;
  }

  for (uint8_t i = 0; i < len; ++i) {
    uint8_t k = g_lora_handler_crypto_key[i % g_lora_handler_crypto_key_len];
    data[i] ^= k;
  }
}

/* Serialize frame to on-air buffer.
 * Payload MUST be already encrypted if needed.
 */
static bool lora_handler_build_raw(const lora_handler_frame_t *frame,
                                   uint8_t *buf, uint8_t *out_len) {
  if (!frame || !buf || !out_len) {
    return false;
  }

  if (frame->len > LORA_HANDLER_MAX_PAYLOAD) {
    return false;
  }

  buf[0] = (uint8_t)frame->type;
  buf[1] = frame->slot_id;

  buf[2] = (uint8_t)(frame->src_id >> 8);
  buf[3] = (uint8_t)(frame->src_id & 0xFF);

  buf[4] = (uint8_t)(frame->dst_id >> 8);
  buf[5] = (uint8_t)(frame->dst_id & 0xFF);

  buf[6] = frame->len;

  if (frame->len > 0) {
    memcpy(&buf[7], frame->payload, frame->len);
  }

  *out_len = (uint8_t)(LORA_HANDLER_HEADER_SIZE + frame->len);
  if (*out_len > E32_TRANSPARENT_MAX_SIZE) {
    return false;
  }

  return true;
}

/* Parse on-air buffer into frame (payload still encrypted at this point). */
static bool lora_handler_parse_raw(lora_handler_frame_t *frame,
                                   const uint8_t *buf, uint8_t len) {
  if (!frame || !buf) {
    return false;
  }

  if (len < LORA_HANDLER_HEADER_SIZE) {
    return false;
  }

  uint8_t payload_len = buf[6];
  if (payload_len > LORA_HANDLER_MAX_PAYLOAD) {
    return false;
  }

  if ((uint8_t)(LORA_HANDLER_HEADER_SIZE + payload_len) != len) {
    return false;
  }

  frame->type = (lora_handler_frame_type_t)buf[0];
  frame->slot_id = buf[1];
  frame->src_id = (uint16_t)((buf[2] << 8) | buf[3]);
  frame->dst_id = (uint16_t)((buf[4] << 8) | buf[5]);
  frame->len = payload_len;

  if (payload_len > 0) {
    memcpy(frame->payload, &buf[7], payload_len);
  }

  return true;
}

/* Internal: send one frame over E32 (always broadcast on RF). */
static bool lora_handler_send_now(lora_handler_ctx_t *ctx,
                                  lora_handler_frame_t *frame) {
  if (!ctx || !ctx->radio || !frame) {
    return false;
  }

  uint8_t raw[LORA_HANDLER_HEADER_SIZE + LORA_HANDLER_MAX_PAYLOAD];
  uint8_t raw_len = 0;

  /* Encrypt payload for DATA frames only.
   * Beacon payload (if any) can stay clear.
   */
  if (frame->type == LORA_HANDLER_FRAME_DATA && frame->len > 0) {
    lora_handler_xor_crypt(frame->payload, frame->len);
  }

  if (!lora_handler_build_raw(frame, raw, &raw_len)) {
    return false;
  }

  lora_e32_comm_status_t st =
      lora_e32_comm_send_broadcast(ctx->radio, raw, (size_t)raw_len);

  if (st == LORA_E32_COMM_OK) {
    ctx->stats.tx_ok++;
    return true;
  }
  return false;
}

/* ----- Small helpers to determine TX slot ----- */

static bool lora_handler_is_tx_slot(const lora_handler_ctx_t *ctx,
                                    uint8_t slot_id) {
  if (!ctx) {
    return false;
  }

  if (ctx->cfg.role == LORA_HANDLER_ROLE_GATEWAY) {
    /* Gateway is allowed to TX in any slot (except you may restrict if needed)
     */
    return true;
  }

  /* Sensor TX only in its own slot */
  return (slot_id == ctx->cfg.my_slot);
}

/* ===== Public API implementation ===== */

void lora_handler_init(lora_handler_ctx_t *ctx, lora_e32_comm_handle_t radio) {
  if (!ctx)
    return;

  memset(ctx, 0, sizeof(*ctx));

  ctx->cfg = g_lora_handler_cfg; /* copy global config */
  ctx->radio = radio;

  ctx->current_slot = 0;
  ctx->frame_start_ms = 0;
  ctx->frame_counter = 0;

  ctx->started = false;
  ctx->is_synced = (ctx->cfg.role == LORA_HANDLER_ROLE_GATEWAY);
  ctx->beacon_sent = false;

  ctx->tx_pending = false;
  ctx->rx_cb = NULL;
  ctx->slot_cb = NULL;

  memset(&ctx->stats, 0, sizeof(ctx->stats));
}

void lora_handler_register_rx_callback(lora_handler_ctx_t *ctx,
                                       lora_handler_rx_cb_t cb) {
  if (!ctx)
    return;
  ctx->rx_cb = cb;
}

void lora_handler_register_slot_callback(lora_handler_ctx_t *ctx,
                                         lora_handler_slot_cb_t cb) {
  if (!ctx)
    return;
  ctx->slot_cb = cb;
}

bool lora_handler_send(lora_handler_ctx_t *ctx, uint16_t dst_id,
                       const uint8_t *payload, uint8_t len) {
  if (!ctx || !payload) {
    return false;
  }

  if (len == 0 || len > LORA_HANDLER_MAX_PAYLOAD) {
    return false;
  }

  if (ctx->tx_pending) {
    /* Simple implementation: only one pending frame at a time */
    return false;
  }

  lora_handler_frame_t *f = &ctx->tx_frame;
  memset(f, 0, sizeof(*f));

  f->type = LORA_HANDLER_FRAME_DATA;
  f->slot_id = ctx->cfg.my_slot; /* will be sent in my TX slot */
  f->src_id = ctx->cfg.node_id;
  f->dst_id = dst_id;
  f->len = len;
  memcpy(f->payload, payload, len);

  ctx->tx_pending = true;
  return true;
}

/* Gateway: send beacon at slot 0 start (no payload). */
static void lora_handler_send_beacon_if_needed(lora_handler_ctx_t *ctx,
                                               uint8_t slot_id) {
  if (!ctx)
    return;
  if (ctx->cfg.role != LORA_HANDLER_ROLE_GATEWAY)
    return;

  if (slot_id != 0 || ctx->beacon_sent) {
    return;
  }

  lora_handler_frame_t f;
  memset(&f, 0, sizeof(f));

  f.type = LORA_HANDLER_FRAME_BEACON;
  f.slot_id = 0;
  f.src_id = ctx->cfg.node_id;
  f.dst_id = LORA_HANDLER_ADDR_BROADCAST;
  f.len = 0;

  if (lora_handler_send_now(ctx, &f)) {
    ctx->beacon_sent = true;
  }
}

void lora_handler_process(lora_handler_ctx_t *ctx, uint32_t now_ms) {
  if (!ctx)
    return;

  if (ctx->cfg.num_slots == 0 || ctx->cfg.slot_duration_ms == 0) {
    return;
  }

  /* Gateway starts timing on first call.
   * Sensor waits for first beacon (is_synced) before running TDMA.
   */
  if (!ctx->started) {
    if (ctx->cfg.role == LORA_HANDLER_ROLE_GATEWAY) {
      ctx->frame_start_ms = now_ms;
      ctx->current_slot = 0;
      ctx->frame_counter = 0;
      ctx->started = true;
      ctx->is_synced = true;
    } else {
      /* Sensor: wait for sync from beacon */
      if (!ctx->is_synced) {
        return;
      }
      /* If sensor was set to synced manually, start from now */
      ctx->frame_start_ms = now_ms;
      ctx->current_slot = 0;
      ctx->frame_counter = 0;
      ctx->started = true;
    }
  }

  if (!ctx->is_synced) {
    /* Sensor lost sync; do nothing until beacon */
    return;
  }

  uint32_t frame_len_ms =
      (uint32_t)ctx->cfg.num_slots * ctx->cfg.slot_duration_ms;
  if (frame_len_ms == 0) {
    return;
  }

  uint32_t delta_ms = now_ms - ctx->frame_start_ms;
  uint32_t frame_index = delta_ms / frame_len_ms;
  uint32_t frame_offset = delta_ms % frame_len_ms;
  uint8_t slot = (uint8_t)(frame_offset / ctx->cfg.slot_duration_ms);

  /* Beacon timeout for sensors: lose sync after N frames without beacon. */
  if (ctx->cfg.role == LORA_HANDLER_ROLE_SENSOR) {
    uint32_t timeout_ms =
        frame_len_ms * (uint32_t)LORA_HANDLER_BEACON_TIMEOUT_FRAMES;
    if (delta_ms > timeout_ms) {
      ctx->is_synced = false;
      ctx->started = false;
      ctx->stats.missed_beacon++;
      return;
    }
  }

  if (frame_index != ctx->frame_counter) {
    ctx->frame_counter = frame_index;
    if (ctx->cfg.role == LORA_HANDLER_ROLE_GATEWAY) {
      ctx->beacon_sent = false;
    }
  }

  if (slot != ctx->current_slot) {
    ctx->current_slot = slot;

    bool tx_slot = lora_handler_is_tx_slot(ctx, slot);

    if (ctx->slot_cb) {
      ctx->slot_cb(slot, tx_slot);
    }

    /* Gateway auto beacon at slot 0 */
    lora_handler_send_beacon_if_needed(ctx, slot);
  }

  /* Handle pending TX when in TX slot */
  if (ctx->tx_pending && lora_handler_is_tx_slot(ctx, ctx->current_slot)) {
    ctx->tx_frame.slot_id = ctx->current_slot; /* update to actual slot */
    if (lora_handler_send_now(ctx, &ctx->tx_frame)) {
      ctx->tx_pending = false;
    }
  }
}

void lora_handler_handle_rx(lora_handler_ctx_t *ctx, const uint8_t *buf,
                            uint8_t len, uint32_t now_ms) {
  if (!ctx || !buf || len == 0) {
    return;
  }

  lora_handler_frame_t frame;
  if (!lora_handler_parse_raw(&frame, buf, len)) {
    ctx->stats.rx_error++;
    return;
  }

  ctx->stats.rx_ok++;

  /* Handle beacon (time sync) */
  if (frame.type == LORA_HANDLER_FRAME_BEACON) {
    if (ctx->cfg.role == LORA_HANDLER_ROLE_SENSOR) {
      /* Simple sync: reset frame_start to now, slot = frame.slot_id */
      ctx->frame_start_ms = now_ms;
      ctx->current_slot = frame.slot_id;
      ctx->frame_counter = 0;
      ctx->is_synced = true;
      ctx->started = true;
    }

    if (ctx->rx_cb) {
      /* Beacon payload is clear, pass directly */
      ctx->rx_cb(&frame);
    }
    return;
  }

  /* Only DATA frames are encrypted */
  if (frame.type == LORA_HANDLER_FRAME_DATA && frame.len > 0) {
    lora_handler_xor_crypt(frame.payload, frame.len); /* decrypt in-place */
  }

  /* Logical address filtering */
  bool for_me = false;

  if (frame.dst_id == LORA_HANDLER_ADDR_BROADCAST) {
    for_me = true;
  } else if (frame.dst_id == ctx->cfg.node_id) {
    for_me = true;
  }

  if (!for_me) {
    return;
  }

  if (ctx->rx_cb) {
    ctx->rx_cb(&frame);
  }
}

void lora_handler_get_stats(lora_handler_ctx_t *ctx,
                            lora_handler_stats_t *out) {
  if (!ctx || !out)
    return;
  *out = ctx->stats;
}

void lora_handler_reset_stats(lora_handler_ctx_t *ctx) {
  if (!ctx)
    return;
  memset(&ctx->stats, 0, sizeof(ctx->stats));
}
