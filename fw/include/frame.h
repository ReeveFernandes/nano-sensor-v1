#ifndef FRAME_H
#define FRAME_H

#include <stdint.h>
#include <stddef.h>

/* ========================= Protocol Constants ========================= */
#define FRAME_SYNC0   0xAA   /* First sync byte to mark start-of-frame */
#define FRAME_SYNC1   0x55   /* Second sync byte to mark start-of-frame */
#define FRAME_VERSION 0x01   /* Protocol version byte embedded in each frame */

/* Frame type identifiers (extend as needed) */
typedef enum {
    FT_DATA   = 0x01,  /* Sensor data payload */
    FT_ACK    = 0x02,  /* Positive acknowledgment */
    FT_NACK   = 0x03,  /* Negative acknowledgment (with reason code payload) */
    FT_STATUS = 0x04,  /* Status summary payload */
    FT_LOG    = 0x05   /* Free-form log/diagnostic text */
} frame_type_t;

/* Practical payload cap (small to keep AVR RAM usage modest) */
#define FRAME_MAX_PAYLOAD 240

/* Return/result codes for framing functions */
typedef enum {
    FR_OK = 0,               /* Operation succeeded */
    FR_ERR_SMALL_OUTBUF,     /* Output buffer isn’t large enough */
    FR_ERR_BAD_SYNC,         /* Sync bytes are not present/incorrect */
    FR_ERR_SHORT_INPUT,      /* Input doesn’t contain a minimal frame */
    FR_ERR_LENGTH_MISMATCH,  /* Declared length doesn’t match provided bytes OR caller’s out buf too small */
    FR_ERR_BAD_CRC           /* CRC mismatch (corrupted or wrong data) */
} frame_result_t;

/* =========================== Public API ============================== */

/**
 * @brief Encode a payload into a wire frame with sync, header, and CRC.
 *
 * Whole-function summary:
 *   Builds a binary frame with this layout:
 *     [0]=0xAA [1]=0x55 [2]=VER [3]=TYPE [4]=LEN_HI [5]=LEN_LO
 *     [6..(6+len-1)]=PAYLOAD [6+len]=CRC_HI [7+len]=CRC_LO
 *   CRC-16/CCITT-FALSE is computed over bytes [2..(5+len)] (VER through end of payload).
 *
 * Parameters:
 *   type      - frame type (e.g., FT_DATA)
 *   payload   - pointer to payload bytes (may be NULL if len==0)
 *   len       - payload length (0..FRAME_MAX_PAYLOAD)
 *   out_buf   - destination buffer for the framed bytes
 *   out_size  - capacity of out_buf in bytes
 *   out_len   - [out] total frame length written to out_buf
 *
 * Returns:
 *   FR_OK on success; otherwise an error describing what went wrong.
 */
frame_result_t frame_encode(uint8_t type,
                            const uint8_t *payload,
                            uint16_t len,
                            uint8_t *out_buf,
                            uint16_t out_size,
                            uint16_t *out_len);

/**
 * @brief Decode a complete wire frame into type + payload (copying payload out).
 *
 * Whole-function summary:
 *   Validates a single, complete frame in memory. Checks sync, verifies that
 *   the length field matches the provided buffer size, recomputes CRC over
 *   VER..PAYLOAD, compares it to the trailing CRC, and on success copies the
 *   payload into `payload_out`.
 *
 * Parameters:
 *   in_buf      - pointer to bytes of a single complete frame
 *   in_len      - number of bytes in the frame buffer
 *   type_out    - [out] parsed frame type
 *   payload_out - [out] caller-provided buffer to receive payload bytes
 *   max_len     - capacity of payload_out in bytes
 *   payload_len - [out] actual number of payload bytes written
 *
 * Returns:
 *   FR_OK on success; error code otherwise (bad sync, CRC, etc.)
 */
frame_result_t frame_decode(const uint8_t *in_buf,
                            uint16_t in_len,
                            uint8_t *type_out,
                            uint8_t *payload_out,
                            uint16_t max_len,
                            uint16_t *payload_len);

/**
 * @brief Peek the total frame length from header without validating CRC.
 *
 * Whole-function summary:
 *   Given a buffer that starts at a frame boundary, this inspects the sync and
 *   length fields to compute the total frame size (header + payload + CRC),
 *   letting a caller know how many bytes must be received before decoding.
 *
 * Parameters:
 *   buf           - pointer to the beginning of a candidate frame
 *   buf_len       - number of valid bytes currently in buf
 *   frame_len_out - [out] total required bytes for the full frame
 *
 * Returns:
 *   FR_OK if sync is correct and header is present; otherwise an error.
 */
frame_result_t frame_peek_length(const uint8_t *buf,
                                 uint16_t buf_len,
                                 uint16_t *frame_len_out);

#endif /* FRAME_H */