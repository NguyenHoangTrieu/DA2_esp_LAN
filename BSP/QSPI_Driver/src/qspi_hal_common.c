// BSP/QSPI_Driver/src/qspi_hal_common.c

#include "qspi_hal.h"
#include <stddef.h>
#include <string.h>

// CRC-8 (polynomial 0x07, init 0x00)
uint8_t qspi_crc8(const uint8_t *data, size_t len) {
  if (data == NULL || len == 0)
    return 0;

  uint8_t crc = 0x00;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }
  return crc;
}

// CRC-16-CCITT (polynomial 0x1021, init 0xFFFF)
uint16_t qspi_crc16_ccitt(const uint8_t *data, size_t len) {
  if (data == NULL || len == 0)
    return 0xFFFF;

  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000)
        crc = (crc << 1) ^ 0x1021;
      else
        crc <<= 1;
    }
  }
  return crc;
}

bool qspi_validate_frame(const qspi_frame_t *frame) {
  if (frame == NULL || frame->payload == NULL)
    return false;

  if (frame->sync != QSPI_FRAME_SYNC)
    return false;

  // Validate length bounds
  if (frame->length == 0 ||
      frame->length > QSPI_DMA_BUFFER_SIZE - QSPI_HEADER_SIZE - 2)
    return false;

  // Validate header CRC
  uint8_t header[5];
  header[0] = (frame->sync >> 8) & 0xFF;
  header[1] = frame->sync & 0xFF;
  header[2] = frame->type;
  header[3] = (frame->length >> 8) & 0xFF;
  header[4] = frame->length & 0xFF;

  uint8_t calc_hcrc = qspi_crc8(header, 5);
  if (calc_hcrc != frame->header_crc)
    return false;

  // Validate payload CRC
  uint16_t calc_pcrc = qspi_crc16_ccitt(frame->payload, frame->length);
  if (calc_pcrc != frame->payload_crc)
    return false;

  return true;
}
