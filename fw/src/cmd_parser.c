#include "cmd_parser.h"
#include <string.h>   // for memcpy

/**
 * @brief Parse a command payload and build a reply payload.
 *
 * Whole-function summary:
 *   Implements PING, START, STOP, SET_RATE <u16 ms>, GET_STATUS. Validates
 *   lengths and argument ranges. On success returns FT_ACK/FT_STATUS and, on
 *   error, FT_NACK with a 1-byte reason code (ERR_BAD_LEN / ERR_BAD_ARG).
 *
 * Line-by-line comments describe each branch for clarity and easy testing.
 */
uint8_t cmd_parse_and_apply(const uint8_t *in, uint16_t in_len,
                            uint8_t *out, uint16_t *out_len)
{
    /* Guard: must have at least an opcode byte */
    if (in_len < 1) {
        out[0] = ERR_BAD_LEN; *out_len = 1; return FT_NACK;
    }

    /* First byte is the opcode */
    uint8_t op = in[0];

    /* Handle each opcode */
    switch (op) {
        case OP_PING: {
            /* PING has no args; reply ACK with "PONG" (debug-friendly) */
            const char *pong = "PONG";               // 4 ASCII bytes
            memcpy(out, pong, 4);                    // Copy reply payload
            *out_len = 4;                            // Reply length = 4
            return FT_ACK;                           // Frame type for caller
        }

        case OP_START: {
            /* START has no args; enable sampling */
            if (in_len != 1) { out[0] = ERR_BAD_LEN; *out_len = 1; return FT_NACK; }
            g_sampling_on = true;                    // Flip state on
            *out_len = 0;                            // ACK with empty payload
            return FT_ACK;                           // Success
        }

        case OP_STOP: {
            /* STOP has no args; disable sampling */
            if (in_len != 1) { out[0] = ERR_BAD_LEN; *out_len = 1; return FT_NACK; }
            g_sampling_on = false;                   // Flip state off
            *out_len = 0;                            // ACK with empty payload
            return FT_ACK;                           // Success
        }

        case OP_SET_RATE: {
            /* SET_RATE expects exactly 2 bytes after opcode (u16 ms, big-endian) */
            if (in_len != 3) {                       // opcode + 2 bytes
                out[0] = ERR_BAD_LEN; *out_len = 1; return FT_NACK;
            }
            /* Extract ms value (big-endian) */
            uint16_t ms = ((uint16_t)in[1] << 8) | in[2];

            /* Validate range */
            if (ms < CMD_RATE_MIN_MS || ms > CMD_RATE_MAX_MS) {
                out[0] = ERR_BAD_ARG; *out_len = 1; return FT_NACK;
            }

            /* Apply config change */
            g_sample_rate_ms = ms;                   // Update device state
            *out_len = 0;                            // Empty ACK payload
            return FT_ACK;                           // Success
        }

        case OP_GET_STATUS: {
            /* GET_STATUS has no args; emit [sampling_on][rate_hi][rate_lo] */
            if (in_len != 1) { out[0] = ERR_BAD_LEN; *out_len = 1; return FT_NACK; }
            out[0] = g_sampling_on ? 1u : 0u;        // 1 if ON, else 0
            out[1] = (uint8_t)(g_sample_rate_ms >> 8);   // Rate high byte
            out[2] = (uint8_t)(g_sample_rate_ms & 0xFF); // Rate low byte
            *out_len = 3;                            // Payload length = 3
            return FT_STATUS;                        // Distinct reply type
        }

        default: {
            /* Unknown opcode → NACK with ERR_BAD_ARG */
            out[0] = ERR_BAD_ARG; *out_len = 1; return FT_NACK;
        }
    }
}