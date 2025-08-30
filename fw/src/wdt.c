#include "wdt.h"

/* ===================== Internal State ===================== */
static uint16_t  s_period_ms[WDT_MAX_TASKS];  /* Declared period per task (ms)      */
static uint32_t  s_last_pet_ms[WDT_MAX_TASKS];/* Last heartbeat time per task (ms)  */
static uint8_t   s_task_count = 0;            /* Number of registered tasks         */
static bool      s_fault = false;             /* Latched fault flag                 */
static int8_t    s_fault_task = -1;           /* Which task missed (-1 if none)     */

/**
 * @brief Initialize the software watchdog with per-task periods.
 *
 * Whole-function summary:
 *   Copies caller-supplied task periods, sets each task's last-pet time to
 *   now_ms, and clears any prior fault. Limits task count to WDT_MAX_TASKS.
 *
 * Line-by-line comments explain each assignment and guard.
 */
void wdt_init(const uint16_t *periods_ms, uint8_t count, uint32_t now_ms) {
    if (count > WDT_MAX_TASKS) {            // If caller asked for too many tasks...
        count = WDT_MAX_TASKS;              // ...clamp to our maximum.
    }
    s_task_count = count;                   // Remember how many tasks we track.
    for (uint8_t i = 0; i < count; i++) {   // For each task index...
        s_period_ms[i]  = periods_ms[i];    // ...store its declared period (ms).
        s_last_pet_ms[i]= now_ms;           // ...pretend it just petted at init.
    }
    s_fault = false;                        // Clear any previously latched fault.
    s_fault_task = -1;                      // No offending task yet.
}

/**
 * @brief Record a heartbeat ("pet") from a task at time now_ms.
 *
 * Whole-function summary:
 *   Updates the task's last-pet time so the service routine knows it is alive.
 *   Invalid task IDs are ignored to keep callers simple and safe.
 */
void wdt_pet(uint8_t task_id, uint32_t now_ms) {
    if (task_id >= s_task_count) {          // If task_id is out of range...
        return;                             // ...ignore silently (safe no-op).
    }
    s_last_pet_ms[task_id] = now_ms;        // Store the time this task last petted.
}

/**
 * @brief Service the watchdog and detect missed heartbeats.
 *
 * Whole-function summary:
 *   Checks each task's elapsed time since its last pet. If any elapsed time
 *   exceeds 2× the task's declared period, we latch a fault and record the
 *   offending task index. Once faulted, we remain faulted until re-init.
 */
void wdt_service_tick(uint32_t now_ms) {
    if (s_fault) {                          // If already faulted...
        return;                             // ...do nothing (fault stays latched).
    }
    for (uint8_t i = 0; i < s_task_count; i++) {           // Scan all tasks...
        uint32_t elapsed = now_ms - s_last_pet_ms[i];      // ...time since last pet.
        uint32_t window  = (uint32_t)s_period_ms[i] * 2u;  // ...allowed window (2×).
        if (elapsed > window) {                             // If task missed window...
            s_fault = true;                                 // ...latch global fault.
            s_fault_task = (int8_t)i;                       // ...record which task missed.
            break;                                          // ...stop scanning further.
        }
    }
}

/**
 * @brief Query if a fault has occurred; optionally get which task missed.
 *
 * Whole-function summary:
 *   Returns whether any task has missed its heartbeat window. If caller
 *   provides which_task, writes the index of the missed task (or -1 if none).
 */
bool wdt_fault_occurred(int8_t *which_task) {
    if (which_task) {                       // If caller wants the task index...
        *which_task = s_fault_task;         // ...write back the offending task.
    }
    return s_fault;                         // True if a fault is currently latched.
}