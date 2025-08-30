#ifndef WDT_H
#define WDT_H

#include <stdint.h>
#include <stdbool.h>

/* ===================== Config ===================== */
#define WDT_MAX_TASKS 8  /* Maximum number of tasks the watchdog tracks */

/**
 * @brief Initialize the software watchdog with per-task periods.
 *
 * Whole-function summary:
 *   Stores each task's expected period (ms) and primes the "last pet" time
 *   to now_ms so no immediate fault occurs. Clears any latched fault state.
 *
 * @param periods_ms  Array of task periods in milliseconds.
 * @param count       Number of tasks (<= WDT_MAX_TASKS).
 * @param now_ms      Current time in milliseconds (monotonic tick).
 */
void wdt_init(const uint16_t *periods_ms, uint8_t count, uint32_t now_ms);

/**
 * @brief Record a heartbeat ("pet") from a task at time now_ms.
 *
 * Whole-function summary:
 *   Updates the last-pet timestamp for the specified task, indicating the
 *   task made forward progress. Invalid task IDs are ignored safely.
 *
 * @param task_id   Zero-based task index.
 * @param now_ms    Current time in milliseconds.
 */
void wdt_pet(uint8_t task_id, uint32_t now_ms);

/**
 * @brief Service the watchdog and detect missed heartbeats.
 *
 * Whole-function summary:
 *   Compares time-since-last-pet for each task against 2× its declared period.
 *   If any task exceeds its window, latches a global fault and records which
 *   task missed. Once faulted, it stays faulted until wdt_init() is called.
 *
 * @param now_ms  Current time in milliseconds.
 */
void wdt_service_tick(uint32_t now_ms);

/**
 * @brief Query if a fault has occurred; optionally get which task missed.
 *
 * Whole-function summary:
 *   Returns true if any task missed its window since the last init. If
 *   which_task is non-NULL, writes the missed task ID (or -1 if none).
 *
 * @param which_task  [out opt] Pointer to receive missed task ID, or NULL.
 * @return true if fault latched, false otherwise.
 */
bool wdt_fault_occurred(int8_t *which_task);

#endif /* WDT_H */