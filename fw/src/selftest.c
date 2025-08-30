#include "selftest.h"
#include "crc16.h"
#include "frame.h"
#include "cmd_parser.h"
#include <string.h>   // for memcpy, strlen
#include <stdio.h>    // for snprintf

/* Provide default device-state storage if not linked elsewhere.
 * In unit tests we also define these; the multiple-definition is avoided
 * by making these weak. If your toolchain doesn't support 'weak', you can
 * remove these and let tests provide the symbols.
 */
#ifdef __GNUC__
__attribute__((weak)) volatile uint16_t g_sample_rate_ms = 1000;
__attribute__((weak)) volatile bool     g_sampling_on     = false;
#endif

/**
 * @brief Append a string to a bounded buffer (helper).
 *
 * Whole-function summary:
 *   Safely appends a C-string message to log_buf, respecting capacity.
 *   If log_buf is NULL or full, this becomes a no-op.
 *
 * Line-by-line:
 *   - Track current length with strnlen.
 *   - Use snprintf to append, clamping to remaining space.
 */
static void log_append(char *log_buf, size_t log_cap, const char *msg) {
    if (!log_buf || log_cap == 0) return;                  // No buffer → nothing to do
    size_t used = strnlen(log_buf, log_cap);               // Current used bytes
    if (used >= log_cap) return;                           // Already full
    (void)snprintf(log_buf + used, log_cap - used, "%s", msg); // Append (auto-clamped)
}

/**
 * @brief Run a lightweight, host-side self-test of core modules.
 *
 * Whole-function summary:
 *   1) CRC: verify CCITT-FALSE vector "123456789" → 0x29B1.
 *   2) Frame: encode+decode a small payload and compare bytes.
 *   3) Command: feed OP_PING to parser, expect FT_ACK + "PONG",
 *      then frame+decode the reply once to exercise framing path.
 *
 * Returns true on success; on failure returns false and writes a reason
 * into err_buf (if provided). Also writes a success log with "SELF-TEST PASS".
 */
bool selftest_run(char *log_buf, size_t log_cap, char *err_buf, size_t err_cap) {
    /* ---------------- 1) CRC known-vector ---------------- */
    {
        const char *s = "123456789";                                    // Canonical CCITT-FALSE vector
        uint16_t crc = crc16_ccitt_false((const uint8_t*)s, strlen(s)); // Compute CRC over bytes
        if (crc != 0x29B1) {                                            // Expect 0x29B1
            if (err_buf && err_cap) snprintf(err_buf, err_cap, "CRC fail: got 0x%04X", crc);
            return false;                                               // Stop on failure
        }
        log_append(log_buf, log_cap, "[OK] CRC vector\n");
    }

    /* ---------------- 2) Frame round-trip ---------------- */
    {
        const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF}; // Sample payload bytes
        uint8_t frame[64];                                   // Outgoing frame buffer
        uint16_t flen = 0;                                   // Will hold frame length

        frame_result_t er = frame_encode(FT_DATA, payload, sizeof(payload),
                                         frame, sizeof(frame), &flen);  // Encode frame
        if (er != FR_OK) {
            if (err_buf && err_cap) snprintf(err_buf, err_cap, "frame_encode err %d", (int)er);
            return false;
        }

        uint8_t out_type = 0;
        uint8_t out_payload[16] = {0};
        uint16_t out_plen = 0;
        frame_result_t dr = frame_decode(frame, flen, &out_type,
                                         out_payload, sizeof(out_payload), &out_plen); // Decode back
        if (dr != FR_OK || out_type != FT_DATA || out_plen != sizeof(payload) ||
            memcmp(payload, out_payload, sizeof(payload)) != 0) {
            if (err_buf && err_cap) snprintf(err_buf, err_cap, "frame round-trip failed");
            return false;
        }
        log_append(log_buf, log_cap, "[OK] Frame round-trip\n");
    }

    /* ---------------- 3) Command parser PING ------------- */
    {
        const uint8_t in[] = { OP_PING };                // Command payload: only opcode
        uint8_t reply[16] = {0};
        uint16_t rlen = 0;
        uint8_t rtype = cmd_parse_and_apply(in, sizeof(in), reply, &rlen); // Run parser
        if (rtype != FT_ACK || rlen != 4 ||
            reply[0] != 'P' || reply[1] != 'O' || reply[2] != 'N' || reply[3] != 'G') {
            if (err_buf && err_cap) snprintf(err_buf, err_cap, "PING parser failed");
            return false;
        }
        log_append(log_buf, log_cap, "[OK] CMD PING→ACK 'PONG'\n");

        /* Frame and decode the reply once to exercise framing path end-to-end */
        uint8_t fbuf[64];
        uint16_t flen = 0;
        if (frame_encode(rtype, reply, rlen, fbuf, sizeof(fbuf), &flen) != FR_OK) {
            if (err_buf && err_cap) snprintf(err_buf, err_cap, "frame_encode(reply) failed");
            return false;
        }
        uint8_t t2=0, pout[16]={0}; uint16_t plen2=0;
        if (frame_decode(fbuf, flen, &t2, pout, sizeof(pout), &plen2) != FR_OK ||
            t2 != FT_ACK || plen2 != rlen || memcmp(pout, reply, rlen) != 0) {
            if (err_buf && err_cap) snprintf(err_buf, err_cap, "frame_decode(reply) failed");
            return false;
        }
        log_append(log_buf, log_cap, "[OK] Framed ACK decode\n");
    }

    /* ---------------- Final success mark ----------------- */
    log_append(log_buf, log_cap, "SELF-TEST PASS\n");     // CI will grep for this
    return true;                                          // All checks succeeded
}