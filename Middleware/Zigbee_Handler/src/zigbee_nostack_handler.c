/**
 * @file zigbee_nostack_handler.c
 * @brief Zigbee No-Stack Handler Implementation (Pure API - No RTOS)
 */

#include "zigbee_nostack_handler.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ZIGBEE_NOSTACK_HANDLER";

/* ===== Global Configuration ===== */
zigbee_nostack_config_t g_zigbee_nostack_cfg = {.node_id = 0x0000,
                                                .gateway_id = 0x0000};

/* ===== Frame Constants ===== */
#define ZIGBEE_FRAME_PREAMBLE (0xA5)

/* ===== Internal Helpers ===== */

/**
 * @brief Calculate CRC16 (CCITT)
 */
static uint16_t calculate_crc16(const uint8_t *data, size_t length) {
  uint16_t crc = 0xFFFF;

  for (size_t i = 0; i < length; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = crc << 1;
      }
    }
  }

  return crc;
}

/**
 * @brief Build raw frame from structure
 */
static bool zigbee_nostack_build_raw(const zigbee_nostack_frame_t *frame,
                                     uint8_t *buf, uint8_t *out_len) {
  if (!frame || !buf || !out_len) {
    return false;
  }

  if (frame->len > ZIGBEE_NOSTACK_MAX_PAYLOAD) {
    return false;
  }

  size_t idx = 0;

  // Preamble
  buf[idx++] = ZIGBEE_FRAME_PREAMBLE;

  // Source address (little-endian)
  buf[idx++] = (frame->src_id & 0xFF);
  buf[idx++] = (frame->src_id >> 8);

  // Destination address (little-endian)
  buf[idx++] = (frame->dst_id & 0xFF);
  buf[idx++] = (frame->dst_id >> 8);

  // Payload length
  buf[idx++] = frame->len;

  // Payload
  if (frame->len > 0) {
    memcpy(&buf[idx], frame->payload, frame->len);
    idx += frame->len;
  }

  // Calculate CRC
  uint16_t crc = calculate_crc16(buf, idx);

  // CRC (little-endian)
  buf[idx++] = (crc & 0xFF);
  buf[idx++] = (crc >> 8);

  *out_len = (uint8_t)idx;
  return true;
}

/**
 * @brief Parse raw buffer into frame structure
 */
static bool zigbee_nostack_parse_raw(zigbee_nostack_frame_t *frame,
                                     const uint8_t *buf, uint8_t len) {
  if (!frame || !buf) {
    return false;
  }

  // Minimum: preamble + src + dst + len + crc = 8 bytes
  if (len < 8) {
    return false;
  }

  // Check preamble
  if (buf[0] != ZIGBEE_FRAME_PREAMBLE) {
    return false;
  }

  // Extract addresses
  frame->src_id = buf[1] | ((uint16_t)buf[2] << 8);
  frame->dst_id = buf[3] | ((uint16_t)buf[4] << 8);

  // Extract length
  frame->len = buf[5];

  // Check total length
  size_t expected_len = 6 + frame->len + 2; // header + payload + crc
  if (len != expected_len) {
    return false;
  }

  // Verify CRC
  uint16_t received_crc = buf[len - 2] | ((uint16_t)buf[len - 1] << 8);
  uint16_t calculated_crc = calculate_crc16(buf, len - 2);

  if (received_crc != calculated_crc) {
    ESP_LOGW(TAG, "CRC error: rx=0x%04X, calc=0x%04X", received_crc,
             calculated_crc);
    return false;
  }

  // Extract payload
  if (frame->len > 0) {
    memcpy(frame->payload, &buf[6], frame->len);
  }

  frame->preamble = buf[0];
  return true;
}

/* ===== Public API Implementation ===== */

void zigbee_nostack_handler_init(zigbee_nostack_handler_ctx_t *ctx,
                                 zigbee_cc_comm_handle_t radio) {
  if (!ctx) {
    return;
  }

  if (radio == NULL) {
    ESP_LOGE(TAG, "Radio handle is NULL!");
    return;
  }

  memset(ctx, 0, sizeof(*ctx));
  ctx->cfg = g_zigbee_nostack_cfg; /* Copy global config */
  ctx->radio = radio;
  ctx->rx_cb = NULL;
  memset(&ctx->stats, 0, sizeof(ctx->stats));

  ESP_LOGI(TAG, "Handler initialized: node_id=0x%04X", ctx->cfg.node_id);
}

void zigbee_nostack_handler_register_rx_callback(
    zigbee_nostack_handler_ctx_t *ctx, zigbee_nostack_rx_cb_t cb) {
  if (!ctx) {
    return;
  }
  ctx->rx_cb = cb;
}

bool zigbee_nostack_handler_send(zigbee_nostack_handler_ctx_t *ctx,
                                 uint16_t dst_id, const uint8_t *payload,
                                 uint8_t len) {
  if (!ctx || !ctx->radio || !payload) {
    return false;
  }

  if (len == 0 || len > ZIGBEE_NOSTACK_MAX_PAYLOAD) {
    ESP_LOGE(TAG, "Invalid payload length: %d", len);
    return false;
  }

  // Build frame
  zigbee_nostack_frame_t frame;
  memset(&frame, 0, sizeof(frame));
  frame.preamble = ZIGBEE_FRAME_PREAMBLE;
  frame.src_id = ctx->cfg.node_id;
  frame.dst_id = dst_id;
  frame.len = len;
  memcpy(frame.payload, payload, len);

  // Serialize to raw buffer
  uint8_t raw_buf[ZIGBEE_NOSTACK_HEADER_SIZE + ZIGBEE_NOSTACK_MAX_PAYLOAD +
                  2]; // +2 for CRC
  uint8_t raw_len = 0;

  if (!zigbee_nostack_build_raw(&frame, raw_buf, &raw_len)) {
    ESP_LOGE(TAG, "Failed to build frame");
    return false;
  }

  // Send via CC2530 UART
  esp_err_t ret = zigbee_cc_comm_write(ctx->radio, raw_buf, raw_len, 1000);

  if (ret == ESP_OK) {
    ctx->stats.tx_ok++;
    ESP_LOGI(TAG, "TX: dst=0x%04X, len=%d", dst_id, len);
    return true;
  } else {
    ESP_LOGE(TAG, "TX failed");
    return false;
  }
}

void zigbee_nostack_handler_handle_rx(zigbee_nostack_handler_ctx_t *ctx,
                                      const uint8_t *buf, uint8_t len) {
  if (!ctx || !buf || len == 0) {
    return;
  }

  zigbee_nostack_frame_t frame;
  ESP_LOGI(TAG, "Handling RX data, len=%d", len);
  ESP_LOGI(TAG, "RX Data: ");
  ESP_LOG_BUFFER_HEXDUMP(TAG, buf, len, ESP_LOG_INFO);
  if (!zigbee_nostack_parse_raw(&frame, buf, len)) {
    ctx->stats.rx_error++;
    ESP_LOGI(TAG, "RX parse error (len=%d)", len);
    return;
  }

  ctx->stats.rx_ok++;

  // Check if frame is for this node
  bool for_me = false;
  if (frame.dst_id == ZIGBEE_NOSTACK_ADDR_BROADCAST) {
    for_me = true;
  } else if (frame.dst_id == ctx->cfg.node_id) {
    for_me = true;
  }

  if (!for_me) {
    ESP_LOGI(TAG, "RX not for us: dst=0x%04X", frame.dst_id);
    return;
  }

  ESP_LOGI(TAG, "RX: src=0x%04X, dst=0x%04X, len=%d", frame.src_id,
           frame.dst_id, frame.len);

  // Call user callback
  if (ctx->rx_cb) {
    ctx->rx_cb(&frame);
  }
}

void zigbee_nostack_handler_get_stats(zigbee_nostack_handler_ctx_t *ctx,
                                      zigbee_nostack_stats_t *out) {
  if (!ctx || !out) {
    return;
  }
  *out = ctx->stats;
}

void zigbee_nostack_handler_reset_stats(zigbee_nostack_handler_ctx_t *ctx) {
  if (!ctx) {
    return;
  }
  memset(&ctx->stats, 0, sizeof(ctx->stats));
}
