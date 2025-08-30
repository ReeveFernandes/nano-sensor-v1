#include "frame.h"
#include "crc16.h"

/**
 * @brief Build a framed packet with sync, version, type, length, payload, CRC.
 *
 * Whole-function summary:
 *   Writes a complete frame into out_buf. Validates buffer capacity first,
 *   then lays out all header fields, copies the payload, computes CRC over
 *   VER..PAYLOAD, appends CRC, and returns the total written length.
 *
 * Line-by-line comments explain each field and safety check.
 */
frame_result_t frame_encode(uint8_t type,
                            const uint8_t *payload,
                            uint16_t len,
                            uint8_t *out_buf,
                            uint16_t out_size,
                            uint16_t *out_len)
{
    /* Compute total size: 2(sync) + 1(ver) + 1(type) + 2(len) + payload + 2(crc) */
    uint16_t needed = 2 + 1 + 1 + 2 + len + 2;

    /* Guard: ensure caller’s out_buf is large enough */
    if (out_size < needed) {
        return FR_ERR_SMALL_OUTBUF; /* Not enough space to write the frame */
    }

    /* 0–1: sync bytes */
    out_buf[0] = FRAME_SYNC0;  /* 0xAA marks frame start (byte 0) */
    out_buf[1] = FRAME_SYNC1;  /* 0x55 marks frame start (byte 1) */

    /* 2: protocol version */
    out_buf[2] = FRAME_VERSION; /* Allows format evolution over time */

    /* 3: frame type */
    out_buf[3] = type;          /* e.g., FT_DATA, FT_ACK, ... */

    /* 4–5: payload length (big-endian) */
    out_buf[4] = (uint8_t)(len >> 8);  /* High byte of payload length */
    out_buf[5] = (uint8_t)(len & 0xFF);/* Low byte of payload length  */

    /* 6..(6+len-1): payload bytes (copy if present) */
    for (uint16_t i = 0; i < len; i++) {
        out_buf[6 + i] = payload ? payload[i] : 0; /* Default 0 if payload is NULL */
    }

    /* Compute CRC-16/CCITT-FALSE over VER..end_of_payload = bytes [2 .. 5+len] */
    uint16_t crc = crc16_ccitt_false(&out_buf[2], (size_t)(4 + len));

    /* Append CRC at the end (big-endian) */
    uint16_t pos = (uint16_t)(6 + len); /* first CRC byte index */
    out_buf[pos + 0] = (uint8_t)(crc >> 8);  /* CRC high byte */
    out_buf[pos + 1] = (uint8_t)(crc & 0xFF);/* CRC low byte  */

    /* Return total bytes written if requested */
    if (out_len) {
        *out_len = (uint16_t)(pos + 2);
    }
    return FR_OK; /* Success */
}

/**
 * @brief Parse a complete frame: check sync, length, CRC; extract type+payload.
 *
 * Whole-function summary:
 *   Validates buffer length, sync bytes, declared payload size, and CRC.
 *   Copies payload to caller-provided buffer and reports parsed type/length.
 *
 * Important: The entire frame must be present in in_buf (no partials).
 */
frame_result_t frame_decode(const uint8_t *in_buf,
                            uint16_t in_len,
                            uint8_t *type_out,
                            uint8_t *payload_out,
                            uint16_t max_len,
                            uint16_t *payload_len)
{
    /* Minimum complete frame: 2(sync) + 1(ver) + 1(type) + 2(len) + 2(crc) = 8 bytes */
    if (in_len < 8) {
        return FR_ERR_SHORT_INPUT; /* Too short to be a valid frame */
    }

    /* Verify sync markers */
    if (in_buf[0] != FRAME_SYNC0 || in_buf[1] != FRAME_SYNC1) {
        return FR_ERR_BAD_SYNC; /* Not at a frame boundary */
    }

    /* Read header fields */
    uint8_t ver  = in_buf[2];              /* Version byte (unused for now) */
    (void)ver;                              /* Suppress unused warning */
    uint8_t type = in_buf[3];              /* Frame type */
    uint16_t len = ((uint16_t)in_buf[4] << 8) | in_buf[5]; /* Payload length */

    /* Expected total size derived from header */
    uint16_t expected = (uint16_t)(6 + len + 2); /* header(6) + payload + crc(2) */
    if (in_len != expected) {
        return FR_ERR_LENGTH_MISMATCH; /* Caller didn’t pass exactly the declared size */
    }

    /* Compute CRC over bytes [2 .. 5+len] (VER..PAYLOAD) */
    uint16_t calc_crc = crc16_ccitt_false(&in_buf[2], (size_t)(4 + len));

    /* Extract received CRC from trailing two bytes (big-endian) */
    uint16_t rx_crc = ((uint16_t)in_buf[6 + len] << 8) | in_buf[6 + len + 1];

    /* Validate integrity */
    if (calc_crc != rx_crc) {
        return FR_ERR_BAD_CRC; /* Corruption or mismatched variant */
    }

    /* Ensure caller’s output buffer is large enough, then copy payload out */
    if (len > max_len) {
        return FR_ERR_LENGTH_MISMATCH; /* Caller provided too-small payload_out */
    }
    for (uint16_t i = 0; i < len; i++) {
        payload_out[i] = in_buf[6 + i]; /* Copy payload bytes */
    }

    /* Write outputs if requested */
    if (type_out)    *type_out = type;
    if (payload_len) *payload_len = len;

    return FR_OK; /* All checks passed */
}

/**
 * @brief Inspect an incoming buffer to determine total frame size from header.
 *
 * Whole-function summary:
 *   Without validating CRC, this checks sync and (if we have >=6 bytes)
 *   computes the total frame size to help callers buffer the full frame
 *   before calling frame_decode().
 */
frame_result_t frame_peek_length(const uint8_t *buf,
                                 uint16_t buf_len,
                                 uint16_t *frame_len_out)
{
    /* Need at least sync+ver+type+len = 6 bytes to know payload size */
    if (buf_len < 6) {
        return FR_ERR_SHORT_INPUT; /* Not enough data to read header */
    }

    /* Validate sync markers */
    if (buf[0] != FRAME_SYNC0 || buf[1] != FRAME_SYNC1) {
        return FR_ERR_BAD_SYNC; /* Not aligned at frame boundary */
    }

    /* Extract payload length (big-endian) and compute full frame size */
    uint16_t payload_len = ((uint16_t)buf[4] << 8) | buf[5];
    uint16_t total = (uint16_t)(6 + payload_len + 2); /* header + payload + crc */

    /* Return through output pointer if provided */
    if (frame_len_out) {
        *frame_len_out = total;
    }
    return FR_OK; /* Size determined successfully */
}