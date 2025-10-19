#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize the UART protocol handler (clears RX buffer).
 *
 * Whole-function summary:
 *   Resets the internal receive buffer so the protocol loop starts from a
 *   clean state. Call once at boot before proto_poll_once().
 */
void proto_init(void);

/**
 * @brief Poll UART, parse complete frames, run commands, and send replies.
 *
 * Whole-function summary:
 *   - Non-blockingly reads bytes from UART into an internal buffer.
 *   - Resynchronizes on 0xAA 0x55 sync if garbage appears.
 *   - When a full frame is buffered, validates CRC and extracts payload.
 *   - Expects inbound frames of type FT_DATA whose payload is a command
 *     (first byte = opcode). Runs cmd_parse_and_apply() to build a reply
 *     payload and immediately sends a framed reply (FT_ACK/FT_STATUS/FT_NACK).
 *
 * Notes:
 *   - Safe to call frequently in the main loop.
 *   - Uses small static buffers to avoid heap usage on AVR.
 */
void proto_poll_once(void);

#endif /* PROTO_H */