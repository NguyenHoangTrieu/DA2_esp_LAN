/**
 * @file spi_framing.c
 * @brief Implementation of the LAN<->WAN SPI framing layer. See header for
 *        the wire format.
 *
 * Design notes:
 *   - CRC8 uses CCITT polynomial 0x07 with init 0x00. No reflection. Computed
 *     in a tight loop (≤ 8 input bytes per call) so no LUT is justified.
 *   - CRC16 uses CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflect, no xorout.
 *     Computed via a 256-entry LUT (built lazily at first use) to keep ~4 KB
 *     payloads cheap.
 *   - The parser is allocation-free and reentrant.
 */

#include "spi_framing.h"

#include <string.h>

/* ----- CRC8/CCITT poly 0x07 ---------------------------------------------- */

uint8_t spi_frame_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x07) : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

/* ----- CRC16/CCITT-FALSE poly 0x1021, init 0xFFFF ------------------------ */

static uint16_t s_crc16_table[256];
static bool     s_crc16_table_ready = false;

static void crc16_table_init(void)
{
    for (uint32_t i = 0; i < 256; ++i) {
        uint16_t c = (uint16_t)(i << 8);
        for (int b = 0; b < 8; ++b) {
            c = (c & 0x8000) ? (uint16_t)((c << 1) ^ 0x1021) : (uint16_t)(c << 1);
        }
        s_crc16_table[i] = c;
    }
    s_crc16_table_ready = true;
}

uint16_t spi_frame_crc16(const uint8_t *data, size_t len)
{
    if (!s_crc16_table_ready) {
        crc16_table_init();
    }
    uint16_t crc = 0xFFFFu;
    for (size_t i = 0; i < len; ++i) {
        uint8_t idx = (uint8_t)((crc >> 8) ^ data[i]);
        crc = (uint16_t)((crc << 8) ^ s_crc16_table[idx]);
    }
    return crc;
}

/* ----- builder ------------------------------------------------------------ */

size_t spi_frame_build(uint8_t *out, size_t out_cap,
                       uint8_t type, uint8_t seq, uint16_t ack_for,
                       const uint8_t *payload, uint16_t len)
{
    if (out == NULL) return 0;
    if (len > SPI_FRAME_MAX_PAYLOAD) return 0;
    if (len > 0 && payload == NULL) return 0;
    size_t total = (size_t)len + SPI_FRAME_OVERHEAD;
    if (total > out_cap) return 0;

    out[0] = SPI_FRAME_SOF_LO;
    out[1] = SPI_FRAME_SOF_HI;
    out[2] = type;
    out[3] = seq;
    out[4] = (uint8_t)(ack_for & 0xFFu);
    out[5] = (uint8_t)((ack_for >> 8) & 0xFFu);
    out[6] = (uint8_t)(len & 0xFFu);
    out[7] = (uint8_t)((len >> 8) & 0xFFu);
    out[8] = spi_frame_crc8(out, 8);

    if (len > 0) {
        memcpy(&out[9], payload, len);
    }

    /* CRC16 over bytes 2..8+len (i.e. type..hdr_crc..payload).                */
    uint16_t crc = spi_frame_crc16(&out[2], 7u + (size_t)len);
    out[9u + len]      = (uint8_t)(crc & 0xFFu);
    out[9u + len + 1u] = (uint8_t)((crc >> 8) & 0xFFu);
    return total;
}

/* ----- parser ------------------------------------------------------------ */

/* Hunt for SOF (0x55 0xAA) starting at @from. Returns offset of SOF or
 * @len when not found. */
static size_t hunt_sof(const uint8_t *buf, size_t len, size_t from)
{
    if (from >= len) return len;
    for (size_t i = from; i + 1 < len; ++i) {
        if (buf[i] == SPI_FRAME_SOF_LO && buf[i + 1] == SPI_FRAME_SOF_HI) {
            return i;
        }
    }
    return len;
}

bool spi_frame_find(const uint8_t *buf, size_t len,
                    spi_frame_view_t *view,
                    spi_frame_status_t *status,
                    spi_frame_stats_t *stats,
                    size_t *consumed)
{
    spi_frame_status_t st = SPI_FRAME_NO_SYNC;
    size_t cons = 0;

    if (buf == NULL || view == NULL || len < SPI_FRAME_OVERHEAD) {
        if (consumed) *consumed = (buf && len) ? len : 0;
        if (status)   *status   = SPI_FRAME_NO_SYNC;
        return false;
    }

    size_t sof = hunt_sof(buf, len, 0);
    if (sof == len) {
        /* No sync at all — discard everything but the trailing byte that
         * might be a partial SOF_LO at the very end. */
        if (stats) stats->resync_bytes += (uint32_t)(len > 0 ? len - 1 : 0);
        if (consumed) *consumed = (len > 1) ? (len - 1) : 0;
        if (status)   *status   = SPI_FRAME_NO_SYNC;
        return false;
    }
    if (stats && sof > 0) {
        stats->resync_bytes += (uint32_t)sof;
    }

    /* Need at least the header */
    if (len - sof < SPI_FRAME_HDR_SIZE) {
        if (consumed) *consumed = sof;  /* keep SOF and accumulate more bytes */
        if (status)   *status   = SPI_FRAME_TRUNCATED;
        return false;
    }

    const uint8_t *hdr = &buf[sof];
    uint8_t hdr_crc_calc = spi_frame_crc8(hdr, 8);
    if (hdr_crc_calc != hdr[8]) {
        if (stats) stats->hdr_crc_fail++;
        /* skip past this SOF so caller can rescan */
        cons = sof + 2;
        st = SPI_FRAME_BAD_HDR_CRC;
        goto done;
    }

    uint16_t plen = (uint16_t)hdr[6] | ((uint16_t)hdr[7] << 8);
    if (plen > SPI_FRAME_MAX_PAYLOAD) {
        if (stats) stats->bad_len++;
        cons = sof + 2;
        st = SPI_FRAME_BAD_LEN;
        goto done;
    }

    size_t need = SPI_FRAME_OVERHEAD + (size_t)plen;
    if (len - sof < need) {
        /* Truncated — leave SOF in place so caller can accumulate. */
        if (consumed) *consumed = sof;
        if (status)   *status   = SPI_FRAME_TRUNCATED;
        return false;
    }

    /* Validate payload CRC (range: type..end of payload = 7+plen bytes) */
    uint16_t crc_calc = spi_frame_crc16(&hdr[2], 7u + (size_t)plen);
    uint16_t crc_wire = (uint16_t)hdr[9u + plen] | ((uint16_t)hdr[9u + plen + 1u] << 8);
    if (crc_calc != crc_wire) {
        if (stats) stats->payload_crc_fail++;
        cons = sof + 2;
        st = SPI_FRAME_BAD_PAYLOAD_CRC;
        goto done;
    }

    /* Success */
    view->type    = hdr[2];
    view->seq     = hdr[3];
    view->ack_for = (uint16_t)hdr[4] | ((uint16_t)hdr[5] << 8);
    view->len     = plen;
    view->payload = (plen > 0) ? &hdr[9] : NULL;
    if (stats) stats->frames_ok++;
    cons = sof + need;
    st = SPI_FRAME_OK;

done:
    if (consumed) *consumed = cons;
    if (status)   *status   = st;
    return (st == SPI_FRAME_OK);
}

uint32_t spi_frame_parse_stream(const uint8_t *buf, size_t len,
                                spi_frame_cb_t cb, void *user,
                                spi_frame_stats_t *stats)
{
    if (buf == NULL || cb == NULL || len == 0) return 0;

    uint32_t count = 0;
    size_t   off   = 0;
    while (off < len) {
        spi_frame_view_t view;
        spi_frame_status_t st;
        size_t cons = 0;
        bool ok = spi_frame_find(&buf[off], len - off, &view, &st, stats, &cons);
        if (ok) {
            cb(&view, user);
            count++;
        }
        if (cons == 0) {
            /* Truncated — no more progress possible in this buffer. */
            break;
        }
        off += cons;
    }
    return count;
}

void spi_frame_track_seq(uint16_t *prev_seq, uint8_t new_seq,
                         spi_frame_stats_t *stats)
{
    if (prev_seq == NULL) return;
    if (*prev_seq == 0xFFFFu) {
        *prev_seq = new_seq;
        return;
    }
    uint8_t expected = (uint8_t)(*prev_seq + 1u);
    if (new_seq != expected) {
        uint8_t gap = (uint8_t)(new_seq - expected);  /* modulo 256 */
        if (stats) stats->seq_gap += gap;
    }
    *prev_seq = new_seq;
}
