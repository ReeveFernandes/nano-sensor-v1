#include "unity.h"
#include "wdt.h"

/**
 * @brief On init, no fault should be latched and all tasks start "just petted."
 */
void test_wdt_init_no_fault(void) {
    const uint16_t periods[] = {100, 200};      // Task 0: 100ms, Task 1: 200ms
    wdt_init(periods, 2, /*now=*/1000);         // Initialize at t=1000ms

    int8_t which = -1;                          // Will receive missed task index
    TEST_ASSERT_FALSE(wdt_fault_occurred(&which)); // No fault right after init
    TEST_ASSERT_EQUAL_INT8(-1, which);          // No offending task recorded
}

/**
 * @brief If tasks pet within 2× period, no fault should occur.
 */
void test_wdt_pet_within_window(void) {
    const uint16_t periods[] = {100};           // One task, 100ms period
    wdt_init(periods, 1, 0);                    // Start at t=0

    wdt_service_tick(150);                      // At t=150, window is 200 → ok
    TEST_ASSERT_FALSE(wdt_fault_occurred(NULL));// No fault yet

    wdt_pet(0, 180);                            // Task pets again at t=180
    wdt_service_tick(260);                      // At t=260, elapsed=80 → ok
    TEST_ASSERT_FALSE(wdt_fault_occurred(NULL));// Still no fault
}

/**
 * @brief If no pet occurs beyond 2× period, a fault should latch with task index.
 */
void test_wdt_miss_triggers_fault(void) {
    const uint16_t periods[] = {100, 50};       // Task0:100ms, Task1:50ms
    wdt_init(periods, 2, 0);                    // Start at t=0

    // Let time advance without pets; task1 has the tighter window (2×50=100)
    wdt_service_tick(101);                      // At t=101, task1 elapsed=101 → miss
    int8_t which = -1;                          // Capture offender
    TEST_ASSERT_TRUE(wdt_fault_occurred(&which));
    TEST_ASSERT_EQUAL_INT8(1, which);           // Task index 1 should be at fault
}

/**
 * @brief After a fault latches, it stays latched until re-init.
 */
void test_wdt_fault_latches_until_reinit(void) {
    const uint16_t periods[] = {100};
    wdt_init(periods, 1, 0);                    // t=0
    wdt_service_tick(250);                      // 250 > 200 → fault
    TEST_ASSERT_TRUE(wdt_fault_occurred(NULL)); // Fault present

    wdt_pet(0, 260);                            // Pet after fault (should not clear it)
    wdt_service_tick(300);                      // Service again
    TEST_ASSERT_TRUE(wdt_fault_occurred(NULL)); // Still faulted

    // Reinitialize to clear fault
    wdt_init(periods, 1, 1000);                 // Re-init at t=1000
    TEST_ASSERT_FALSE(wdt_fault_occurred(NULL));// Fault cleared
}

/**
 * @brief Invalid task IDs are ignored safely.
 */
void test_wdt_invalid_task_id_ignored(void) {
    const uint16_t periods[] = {100};
    wdt_init(periods, 1, 0);                    // One valid task: ID=0

    wdt_pet(7, 10);                              // Out-of-range; should be ignored
    wdt_service_tick(50);                        // Within window → no fault
    TEST_ASSERT_FALSE(wdt_fault_occurred(NULL)); // Still healthy
}