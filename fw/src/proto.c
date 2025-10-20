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
 * @brief Send current status as an FT_STATUS frame immediately.
 *
 * Whole-function summary:
 *   Builds a 3-byte payload [on][rate_hi][rate_lo] from the public globals
 *   (g_sampling_on, g_sample_rate_ms), frames it as FT_STATUS (0x04) with
 *   CRC16, and writes it out the UART.
 *
 * Line-by-line:
 *   - Collect state into payload.
 *   - frame_encode() builds the full wire frame into a small buffer.
 *   - uart_write() transmits the result.
 */
void proto_send_status_now(void)
{
    extern volatile bool     g_sampling_on;
    extern volatile uint16_t g_sample_rate_ms;

    uint8_t payload[3];
    payload[0] = g_sampling_on ? 1u : 0u;
    payload[1] = (uint8_t)(g_sample_rate_ms >> 8);
    payload[2] = (uint8_t)(g_sample_rate_ms & 0xFF);

    uint8_t  frame[16];
    uint16_t out_len = 0;
    if (frame_encode(FT_STATUS, payload, 3, frame, sizeof(frame), &out_len) == FR_OK && out_len) {
        uart_write(frame, out_len);
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
 * @brief Poll UART, resync to 0xAA55, handle any complete frames (strict, silent).
 *
 * Whole-function summary:
 *   Non-blockingly ingests UART bytes into s_rx, realigns to sync if needed,
 *   and when a complete frame is present calls proto_handle_frame(...) which
 *   runs the command parser and transmits exactly one reply frame.
 */
void proto_poll_once(void) {
    // Ingest available bytes
    for (;;) {
        int ch = uart_getc_nonblocking();
        if (ch < 0) break;
        if (s_rx_len < RX_BUF_CAP) {
            s_rx[s_rx_len++] = (uint8_t)ch;
        } else {
            s_rx_len = 0;  // overflow → drop and resync
            break;
        }
    }

    // Resync to sync bytes if needed
    proto_resync_to_sync();

    // Process complete frames at buffer head
    for (;;) {
        if (s_rx_len < 6) break; // need header
        if (!(s_rx[0] == FRAME_SYNC0 && s_rx[1] == FRAME_SYNC1)) {
            proto_resync_to_sync();
            if (s_rx_len < 6) break;
            if (!(s_rx[0] == FRAME_SYNC0 && s_rx[1] == FRAME_SYNC1)) break;
        }
        uint16_t plen = (uint16_t)((s_rx[4] << 8) | s_rx[5]);
        uint16_t need = (uint16_t)(6 + plen + 2);
        if (s_rx_len < need) break;

        // Handle one frame
        proto_handle_frame(s_rx, need);

        // Remove consumed frame
        uint16_t remaining = (uint16_t)(s_rx_len - need);
        if (remaining) {
            memmove(s_rx, &s_rx[need], remaining);
        }
        s_rx_len = remaining;
    }
}

/**
 * @brief Send one 16-bit sensor sample (big-endian) + 8-bit sequence as FT_DATA.
 *
 * Whole-function summary:
 *   Builds a 3-byte payload: [sample_hi][sample_lo][seq] and frames it as
 *   FT_DATA (0x01) with CRC16, then writes it out the UART.
 *
 * Line-by-line:
 *   - Fill payload bytes from inputs (big-endian for the sample).
 *   - Call frame_encode() to build a full frame into a small buffer.
 *   - uart_write() transmits it in one go.
 */
void proto_send_data16(uint16_t sample, uint8_t seq)
{
    uint8_t payload[3];
    payload[0] = (uint8_t)(sample >> 8);
    payload[1] = (uint8_t)(sample & 0xFF);
    payload[2] = seq;

    uint8_t frame[16];
    uint16_t out_len = 0;
    frame_result_t r = frame_encode(FT_DATA, payload, 3, frame, sizeof(frame), &out_len);
    if (r == FR_OK && out_len > 0) {
        uart_write(frame, out_len);
    }
}

/**
 * @brief Send a single u16 sample as an FT_DATA frame.
 *
 * Whole-function summary:
 *   Builds a 2-byte payload [hi, lo] from the 16-bit sample value and
 *   frames it as FT_DATA (0x01) with CRC16, then transmits over UART.
 *
 * Parameters:
 *   sample_u16 - sensor value to transmit (e.g., 0..1023 from ADC).
 */
void proto_send_sample_u16(uint16_t sample_u16) {
    uint8_t payload[2];
    payload[0] = (uint8_t)(sample_u16 >> 8);
    payload[1] = (uint8_t)(sample_u16 & 0xFF);

    uint8_t  frame[16];
    uint16_t out_len = 0;
    if (frame_encode(FT_DATA, payload, 2, frame, sizeof(frame), &out_len) == FR_OK && out_len) {
        uart_write(frame, out_len);
    }
}
