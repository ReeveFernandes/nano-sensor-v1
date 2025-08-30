#ifndef SELFTEST_H
#define SELFTEST_H

#include <stddef.h>
#include <stdbool.h>

/**
 * @brief Run a lightweight, host-side self-test of core modules.
 *
 * Whole-function summary:
 *   Runs three checks: (1) CRC known vector, (2) frame encode/decode
 *   round-trip, and (3) command parser PING→ACK with "PONG" payload
 *   (optionally framing the reply). Writes human-readable notes into
 *   log_buf and "SELF-TEST PASS" if all checks succeed; on failure,
 *   writes a short reason into err_buf and returns false.
 *
 * Parameters:
 *   log_buf  - [out] buffer to receive a short success log (may be NULL).
 *   log_cap  - capacity of log_buf in bytes (0 if log_buf is NULL).
 *   err_buf  - [out] buffer to receive a short failure reason (may be NULL).
 *   err_cap  - capacity of err_buf in bytes (0 if err_buf is NULL).
 *
 * Returns:
 *   true on success (all subtests passed); false and sets err_buf on failure.
 */
bool selftest_run(char *log_buf, size_t log_cap, char *err_buf, size_t err_cap);

#endif /* SELFTEST_H */