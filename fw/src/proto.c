#include "proto.h"
#include "uart_avr.h"
#include "frame.h"
#include "cmd_parser.h"
#include <string.h>   // for memmove
#include <stdbool.h>  // for bool

/* ===================== Internal RX buffer ===================== */
/* We accumulate raw UART bytes here, resync if needed, and
 * process complete frames as they arrive. Keep it modest to
 * fit easily on an ATmega328P.
 */
#define RX_BUF_CAP 256
static uint8_t  s_rx[RX_BUF_CAP];   /* Raw bytes received from UART          */
static uint16_t s_rx_len = 0;       /* Number of valid bytes currently in s_rx */

/* To avoid depending on any non-existent peek API, we derive total length
 * directly from the 6-byte header when present:
 * Frame = [AA 55][VER][TYPE][LEN_HI LEN_LO] [PAYLOAD...] [CRC_HI CRC_LO]
 * Total length = 6 + payload_len + 2
 */

/**
 * @brief Reset the RX buffer to an empty state.
 *
 * Whole-function summary:
 *   Clears the receive length so subsequent polls start from index 0.
 *   This is called once at boot, and also used on overflow/error recovery.
 */
void proto_init(void) {
    s_rx_len = 0;                         // Mark buffer empty
}

/**
 * @brief Transmit a framed reply over UART (helper).
 *
 * Whole-function summary:
 *   Encodes [SYNC][VER][TYPE][LEN][PAYLOAD][CRC] into a small buffer
 *   and writes it out via UART in one shot.
 *
 * Line-by-line:
 *   - Encode with frame_encode().
 *   - If encoding fails (shouldn’t for our sizes), drop silently.
 *   - On success, transmit using uart_write().
 */
static void proto_send_reply(uint8_t reply_type,
                             const uint8_t *payload,
                             uint16_t plen)
{
    uint8_t out[96];                      // Plenty for small replies
    uint16_t out_len = 0;                 // Will hold encoded length
    if (frame_encode(reply_type, payload, plen, out, sizeof(out), &out_len) == FR_OK) {
        uart_write(out, out_len);         // Transmit the entire frame
    }
}

/**
 * @brief Handle one complete, validated inbound frame of type FT_DATA.
 *
 * Whole-function summary:
 *   Strictly decodes the frame (CRC checked). If the frame type is FT_DATA,
 *   treats the payload as a command (opcode + args), runs the command parser,
 *   and sends back a framed reply using the returned reply type (ACK/NACK/STATUS).
 *   Frames of other types are ignored.
 */
static void proto_handle_frame(const uint8_t *buf, uint16_t buf_len) {
    uint8_t  ftype = 0;                              // Inbound frame type
    uint8_t  payload[128];                           // Decoded payload storage
    uint16_t plen = 0;                               // Decoded payload length

    /* Decode and verify CRC/length */
    frame_result_t r = frame_decode(buf, buf_len, &ftype,
                                    payload, sizeof(payload), &plen);
    if (r != FR_OK) {
        return;                                      // CRC/length error → drop
    }
    if (ftype != FT_DATA) {
        return;                                      // Only accept command frames
    }

    /* Run the command and send reply */
    uint8_t  reply[128];
    uint16_t rlen = 0;
    uint8_t  reply_type = cmd_parse_and_apply(payload, plen, reply, &rlen);
    proto_send_reply(reply_type, reply, rlen);
}

/**
 * @brief Consume bytes from s_rx[] until we are aligned on sync (0xAA,0x55).
 *
 * Whole-function summary:
 *   Scans the buffer for the first occurrence of FRAME_SYNC0/1 and removes
 *   any preceding garbage by shifting the remaining bytes to the front.
 *
 * Line-by-line:
 *   - If fewer than 2 bytes, nothing to do.
 *   - If already aligned, return.
 *   - Search for the next sync pair; if found, compact; otherwise keep
 *     at most a trailing FRAME_SYNC0 (to catch a sync split across reads).
 */
static void proto_resync_to_sync(void) {
    if (s_rx_len < 2) return;             // Not enough data to check sync

    if (s_rx[0] == FRAME_SYNC0 && s_rx[1] == FRAME_SYNC1) {
        return;                            // Already aligned on sync
    }

    // Search for the next sync pair within the buffer
    uint16_t i = 1;
    while (i + 1 < s_rx_len) {
        if (s_rx[i] == FRAME_SYNC0 && s_rx[i + 1] == FRAME_SYNC1) break;
        i++;
    }

    if (i + 1 < s_rx_len) {
        // Found sync at index i → shift remaining bytes down
        memmove(s_rx, &s_rx[i], (size_t)(s_rx_len - i));
        s_rx_len = (uint16_t)(s_rx_len - i);
    } else {
        // No sync pair found: keep only a trailing SYNC0 if present
        if (s_rx[s_rx_len - 1] == FRAME_SYNC0) {
            s_rx[0] = FRAME_SYNC0;
            s_rx_len = 1;
        } else {
            s_rx_len = 0;
        }
    }
}

/**
 * @brief Poll UART, parse complete frames, run commands, and send replies.
 *
 * Whole-function summary:
 *   Non-blockingly ingests bytes from UART, realigns to the sync header
 *   (0xAA 0x55) if needed, and when a full frame is present it validates and
 *   decodes it, runs the command parser on the payload, and sends a framed
 *   reply. Dev echo / breadcrumbs are intentionally disabled here.
 */
void proto_poll_once(void) {
    /* 1) Ingest any available UART bytes into s_rx (bounded) */
    for (;;) {
        int ch = uart_getc_nonblocking();          // Read one byte if available
        if (ch < 0) break;                         // No more bytes this tick
        if (s_rx_len < RX_BUF_CAP) {
            s_rx[s_rx_len++] = (uint8_t)ch;        // Append to RX buffer
        } else {
            s_rx_len = 0;                          // Overflow → drop buffer to resync
            break;
        }
    }

    /* 2) Resync to the sync markers 0xAA 0x55 if needed */
    proto_resync_to_sync();                         // Compact to the next AA 55

    /* 3) Process complete frames at the front of s_rx */
    for (;;) {
        if (s_rx_len < 6) break;                    // Need header to know length
        if (!(s_rx[0] == FRAME_SYNC0 && s_rx[1] == FRAME_SYNC1)) {
            proto_resync_to_sync();
            if (s_rx_len < 6) break;
            if (!(s_rx[0] == FRAME_SYNC0 && s_rx[1] == FRAME_SYNC1)) break;
        }

        /* Payload length is big-endian in bytes [4..5] */
        uint16_t payload_len  = (uint16_t)((s_rx[4] << 8) | s_rx[5]);
        uint16_t total_needed = (uint16_t)(6 + payload_len + 2);   // hdr + payload + CRC

        if (s_rx_len < total_needed) break;         // Not enough yet → wait

        /* Handle the complete frame at the front */
        proto_handle_frame(s_rx, total_needed);

        /* Remove consumed frame by shifting remainder down */
        uint16_t remaining = (uint16_t)(s_rx_len - total_needed);
        if (remaining) {
            memmove(s_rx, &s_rx[total_needed], remaining);
        }
        s_rx_len = remaining;
    }
}