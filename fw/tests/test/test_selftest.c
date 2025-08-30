#include "unity.h"
#include "crc16.h"
#include "frame.h"
#include "cmd_parser.h"
#include "selftest.h"
#include <string.h>   // for strstr

/* Provide device-state storage for tests if not provided elsewhere */
volatile uint16_t g_sample_rate_ms = 1000;
volatile bool     g_sampling_on     = false;

/**
 * @brief selftest_run should return true and include "SELF-TEST PASS" in the log.
 */
void test_selftest_run_passes_and_logs(void) {
    char log[256] = {0};            // Buffer to receive success notes
    char err[128] = {0};            // Buffer to receive error reason (unused on success)
    bool ok = selftest_run(log, sizeof log, err, sizeof err);
    TEST_ASSERT_TRUE(ok);           // Expect overall success
    TEST_ASSERT_NOT_NULL(strstr(log, "SELF-TEST PASS")); // Must contain success mark
}