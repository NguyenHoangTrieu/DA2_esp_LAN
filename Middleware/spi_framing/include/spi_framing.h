/**
 * @file spi_framing.h
 * @brief Transparent framing layer between LAN and WAN MCUs.
 *
 * Wire format (little-endian, total overhead = SPI_FRAME_OVERHEAD = 11 bytes):
 *
 *   offset  field      bytes  notes
 *   ------  ---------  -----  ------------------------------------------------
 *   0       SOF        2      0x55 0xAA — sync, parser scans for this pair
 *   2       TYPE       1      see spi_frame_type_t
 *   3       SEQ        1      rolling 1-byte sequence; receiver detects gaps
 *   4       ACK_FOR    2      LE; SPI_FRAME_ACK_NONE (0xFFFF) = no ack piggyback.
 *                             Otherwise: cumulative ACK — "I have received every
 *                             frame from you up through this seq".  Slave sets
 *                             this on every TX to ack the last master seq it
 *                             successfully parsed; master sets 0xFFFF (no ack
 *                             from master to slave today).
 *   6       LEN        2      payload length, LE (0..SPI_FRAME_MAX_PAYLOAD)
 *   8       HDR_CRC    1      CRC-8/CCITT (poly 0x07) over bytes 0..7
 *   9       PAYLOAD    LEN    raw application payload (e.g. [CF][...] today)
 *   9+LEN   PAY_CRC    2      CRC-16/CCITT (poly 0x1021, init 0xFFFF) over
 *                             bytes 2..8+LEN  (type..end of payload)
 *
 * PAYLOAD is still the legacy CF/DT/DQ/CQ format so existing handler
 * dispatchers (which read payload[0..1]) keep working — TYPE-based dispatch
 * is a separate future change.
 *
 * Version history:
 *   v1 (P1): 9-byte overhead, no ACK field.
 *   v2 (P3.b): 11-byte overhead, adds ACK_FOR. Both MCUs must run v2.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_FRAME_SOF_LO        0x55u
#define SPI_FRAME_SOF_HI        0xAAu
#define SPI_FRAME_HDR_SIZE      9u  /* SOF(2)+TYPE(1)+SEQ(1)+ACK_FOR(2)+LEN(2)+HDRCRC(1) */
#define SPI_FRAME_TAIL_SIZE     2u  /* CRC16 */
#define SPI_FRAME_OVERHEAD      (SPI_FRAME_HDR_SIZE + SPI_FRAME_TAIL_SIZE)
#define SPI_FRAME_MAX_PAYLOAD   4096u
#define SPI_FRAME_MAX_SIZE      (SPI_FRAME_MAX_PAYLOAD + SPI_FRAME_OVERHEAD)

/** Sentinel for ACK_FOR when this frame carries no piggyback ack. */
#define SPI_FRAME_ACK_NONE      0xFFFFu

/**
 * TYPE field. P1 only uses USER_BLOB — the application-level kind (CF/DT/DQ)
 * still lives inside PAYLOAD's first two bytes. P3.b/P3.c may split this out.
 */
typedef enum {
    SPI_FT_IDLE      = 0x00,  /* filler: parser keeps scanning past this frame */
    SPI_FT_USER_BLOB = 0xC0,  /* opaque payload — caller is responsible        */
} spi_frame_type_t;

typedef enum {
    SPI_FRAME_OK = 0,
    SPI_FRAME_NO_SYNC,       /* no SOF found in buffer                          */
    SPI_FRAME_BAD_HDR_CRC,   /* SOF + header bytes present but HDR_CRC mismatched */
    SPI_FRAME_BAD_LEN,       /* LEN > SPI_FRAME_MAX_PAYLOAD                     */
    SPI_FRAME_TRUNCATED,     /* SOF + good header found, but buffer ends before
                                CRC16 — caller should keep accumulating bytes  */
    SPI_FRAME_BAD_PAYLOAD_CRC,
} spi_frame_status_t;

typedef struct {
    uint8_t        type;
    uint8_t        seq;
    uint16_t       ack_for;  /* SPI_FRAME_ACK_NONE if no piggyback ack          */
    uint16_t       len;
    const uint8_t *payload;  /* pointer into caller's buffer; valid until that
                                buffer is mutated or freed                     */
} spi_frame_view_t;

typedef struct {
    uint32_t frames_ok;
    uint32_t hdr_crc_fail;
    uint32_t payload_crc_fail;
    uint32_t bad_len;
    uint32_t resync_bytes;   /* bytes skipped while hunting for SOF             */
    uint32_t seq_gap;        /* monotonic gap detected by spi_frame_track_seq() */
} spi_frame_stats_t;

/* ============================================================================
 * Builder
 * ========================================================================= */

/**
 * Build one frame into @out.
 *
 * @param out      destination buffer (caller-owned, must be ≥ len + SPI_FRAME_OVERHEAD)
 * @param out_cap  capacity of @out
 * @param type     SPI_FT_*
 * @param seq      sequence byte (caller manages rolling counter)
 * @param ack_for  cumulative ACK — pass SPI_FRAME_ACK_NONE when this frame
 *                 carries no piggyback ack
 * @param payload  bytes to wrap (may be NULL only if len == 0)
 * @param len      payload length (0..SPI_FRAME_MAX_PAYLOAD)
 * @return         total bytes written, or 0 on error (cap too small / len too big)
 */
size_t spi_frame_build(uint8_t *out, size_t out_cap,
                       uint8_t type, uint8_t seq, uint16_t ack_for,
                       const uint8_t *payload, uint16_t len);

/* ============================================================================
 * Parser
 * ========================================================================= */

/**
 * Find and validate the next frame in @buf.
 *
 * Scans for SOF, validates HDR_CRC and PAY_CRC, populates @view on success.
 * Garbage bytes before the SOF are reported via stats->resync_bytes.
 *
 * @param buf        input bytes
 * @param len        bytes available
 * @param view       [out] frame metadata + pointer into @buf on success
 * @param status     [out] reason code; pass NULL to ignore
 * @param stats      [in/out] counters; pass NULL to skip
 * @param consumed   [out] number of bytes consumed (move buf forward by this).
 *                   On OK: SPI_FRAME_OVERHEAD + view->len. On hard error:
 *                   the offset past the failed SOF (so caller can rescan).
 *                   On TRUNCATED: 0 (caller should accumulate more bytes).
 *                   On NO_SYNC: @len (caller can discard buffer).
 * @return true if a valid frame was extracted into @view.
 */
bool spi_frame_find(const uint8_t *buf, size_t len,
                    spi_frame_view_t *view,
                    spi_frame_status_t *status,
                    spi_frame_stats_t *stats,
                    size_t *consumed);

/**
 * Convenience: walk a buffer that may contain multiple back-to-back frames
 * plus padding. Calls @cb once per valid frame found. Returns the number
 * of valid frames dispatched.
 */
typedef void (*spi_frame_cb_t)(const spi_frame_view_t *view, void *user);

uint32_t spi_frame_parse_stream(const uint8_t *buf, size_t len,
                                spi_frame_cb_t cb, void *user,
                                spi_frame_stats_t *stats);

/**
 * Maintain a 1-byte rolling sequence per direction and detect gaps.
 * Pass the previous received seq (or 0xFFFF on first call) and the new one;
 * returns the gap count (0 if contiguous) and updates @prev_seq in place.
 */
void spi_frame_track_seq(uint16_t *prev_seq, uint8_t new_seq,
                         spi_frame_stats_t *stats);

/* ============================================================================
 * CRC helpers (exposed for tests; safe to inline ignore)
 * ========================================================================= */

uint8_t  spi_frame_crc8(const uint8_t *data, size_t len);
uint16_t spi_frame_crc16(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif
