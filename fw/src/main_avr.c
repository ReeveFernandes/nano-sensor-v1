#include <avr/io.h>       // MCU register definitions
#include <util/delay.h>   // Optional small delays
#include <string.h>       // strlen
#include <stdbool.h>      // bool
#include "selftest.h"     // selftest_run()
#include "uart_avr.h"     // UART HAL

/**
 * @brief AVR entry point: run core self-test and print the log over UART.
 *
 * Whole-function summary:
 *   Initializes UART0 (115200, 8N1), runs the consolidated self-test that
 *   exercises CRC, framing, and the PING command, then transmits the resulting
 *   human-readable log over serial. If the test fails, prints a clear error
 *   line. Loops forever afterwards so a serial terminal can read the output.
 *
 * Line-by-line:
 *   - Initialize UART for 115200 baud.
 *   - Run selftest_run() to validate core modules at boot.
 *   - Print either the success log (includes "SELF-TEST PASS") or an error.
 *   - Idle loop; in future steps we’ll add command parsing and periodic data.
 */
int main(void) {
    uart_init(115200);                              // Set up UART0 for 115200 baud, 8N1

    _delay_ms(50);

    char log_buf[512] = {0};                        // Buffer to collect success notes
    char err_buf[128] = {0};                        // Buffer to capture failure reason
    bool ok = selftest_run(log_buf, sizeof log_buf, err_buf, sizeof err_buf); // Run self-test

    uart_puts("\r\n[BOOT] Nano Sentinel starting...\r\n"); // Friendly banner on boot

    if (ok) {                                       // If self-test passed...
        uart_puts(log_buf);                         // ...print the success log (ends with SELF-TEST PASS)
    } else {                                        // If self-test failed...
        uart_puts("SELF-TEST FAIL: ");              // ...print a clear failure line
        uart_puts(err_buf[0] ? err_buf : "(no detail)"); // ...with reason if available
        uart_puts("\r\n");
    }

    uart_puts("[BOOT] READY\r\n");                  // Indicate boot sequence complete

    for (;;) {                                      // Idle forever (placeholder for tasks/RTOS)
        _delay_ms(500);                             // Small delay to avoid busy-spinning at 100% CPU
        // Future: poll for incoming bytes, parse frames, respond to commands, etc.
    }
}