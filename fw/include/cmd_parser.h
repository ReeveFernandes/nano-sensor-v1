#ifndef CMD_PARSER_H
#define CMD_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include "frame.h"  // for FT_ACK, FT_NACK, FT_STATUS

/* ===================== Opcodes (1-byte) ===================== */
enum {
    OP_PING      = 0x01,  /* No args                 */
    OP_START     = 0x02,  /* No args                 */
    OP_STOP      = 0x03,  /* No args                 */
    OP_SET_RATE  = 0x04,  /* Args: u16 ms (big-end)  */
    OP_GET_STATUS= 0x05   /* No args                 */
};

/* ===================== NACK reason codes ==================== */
enum {
    ERR_BAD_LEN  = 1,     /* Wrong argument length    */
    ERR_BAD_ARG  = 2      /* Argument out of range    */
};

/* ===================== Device state (extern) =================
 * These are defined in your main/device module at link time.
 * In unit tests, we define them inside the test file.
 */
extern volatile uint16_t g_sample_rate_ms;
extern volatile bool     g_sampling_on;

/* Bounds for SET_RATE command (adjust as you like) */
#define CMD_RATE_MIN_MS  ((uint16_t)10)
#define CMD_RATE_MAX_MS  ((uint16_t)10000)

/**
 * @brief Parse a command payload and build a reply payload.
 *
 * Whole-function summary:
 *   Parses the incoming command payload (first byte opcode, following bytes
 *   are arguments). Applies changes to device state (start/stop/rate) and
 *   writes a reply payload + reply frame "type" (FT_ACK, FT_NACK, FT_STATUS).
 *
 * Parameters:
 *   in       - pointer to input bytes (first byte is opcode)
 *   in_len   - number of input bytes
 *   out      - [out] buffer where reply payload bytes will be written
 *   out_len  - [out] number of reply payload bytes written
 *
 * Returns:
 *   FT_ACK / FT_NACK / FT_STATUS (use as your frame type when sending).
 */
uint8_t cmd_parse_and_apply(const uint8_t *in, uint16_t in_len,
                            uint8_t *out, uint16_t *out_len);

#endif /* CMD_PARSER_H */