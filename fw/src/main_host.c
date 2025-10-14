#include <stdio.h>      // for printf
#include <string.h>     // for strlen
#include <stdbool.h>    // for bool
#include "selftest.h"   // for selftest_run()

/**
 * @brief Minimal host entry point that runs the core self-test and prints results.
 *
 * Whole-function summary:
 *   Allocates small log/error buffers, invokes selftest_run(), and prints either
 *   the success log (which will include "SELF-TEST PASS") or an error string.
 *   Returns 0 on success and 1 on failure so that CI can fail fast if needed.
 *
 * Line-by-line notes:
 *   - Allocate log/err buffers; zero-init so we can safely strlen/printf.
 *   - Call selftest_run() to exercise CRC, framing, and PING→ACK parser path.
 *   - On success: print the accumulated log to stdout.
 *   - On failure: print a clear error message to stderr and return non-zero.
 */
int main(void) {
    char log_buf[512] = {0};   // Collects readable success notes & "SELF-TEST PASS"
    char err_buf[256] = {0};   // Receives short failure reason if something breaks

    // Run the consolidated self-test of core modules (CRC, frame, parser)
    bool ok = selftest_run(log_buf, sizeof log_buf, err_buf, sizeof err_buf);

    if (ok) {
        // Print the human-friendly log. CI will grep for "SELF-TEST PASS".
        // We use stdout for normal success output.
        printf("%s", log_buf);
        return 0; // Success exit code
    } else {
        // Print a clear failure reason to stderr so it's easy to spot in logs.
        fprintf(stderr, "SELF-TEST FAIL: %s\n", err_buf[0] ? err_buf : "(no detail)");
        return 1; // Non-zero so scripts/CI can fail fast
    }
}