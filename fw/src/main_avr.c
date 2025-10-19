/**
 * @brief Firmware entry point: init UART, run self-test, guard RX pin, poll protocol.
 *
 * Whole-function summary:
 *   Brings up USART0 at 115200 (8N1) with RX+TX enabled, prints the boot banner,
 *   runs the self-test (using the new selftest_run(log,log_cap,err,err_cap) API),
 *   dumps UART/GPIO registers once for visibility, then enforces that PD0 (UART RX)
 *   stays as INPUT with pull-up while the protocol handler polls for framed commands.
 *
 * Notes:
 *   - Requires: uart_init(), uart_puts(), uart_debug_dump(), proto_init(), proto_poll_once().
 *   - Includes "proto.h" to avoid implicit-function warnings for proto_*.
 */

#include <avr/io.h>          // For DDRD/PORTD/UCSR0B bits
#include <util/delay.h>      // For _delay_ms
#include <stdbool.h>
#include <stddef.h>
#include "uart_avr.h"        // UART init/IO
#include "selftest.h"        // selftest_run(...)
#include "proto.h"           // proto_init(), proto_poll_once()

/* Forward declaration for the debug print helper implemented in uart_avr.c */
void uart_debug_dump(void);

int main(void) {
    /* Initialize UART at 115200 baud, 8-N-1, RX+TX enabled */
    uart_init(115200);                     // Hardware UART0 configured (RXEN0|TXEN0)
    _delay_ms(50);                         // Small settle for the USB-serial adapter

    /* Boot banner */
    uart_puts("[BOOT] Nano Sentinel starting...\n");

    /* Run self-test using the newer API that takes log and error buffers */
    char log_buf[256] = {0};               // Collects human-friendly test transcript
    char err_buf[128] = {0};               // Collects first error message (if any)
    bool ok = selftest_run(log_buf, sizeof(log_buf), err_buf, sizeof(err_buf));

    /* Print any self-test log lines (if the function filled the buffer) */
    if (log_buf[0] != '\0') {
        uart_puts(log_buf);                // Assumes NUL-terminated text in log_buf
    }

    /* Report PASS/FAIL and any error message */
    if (ok) {
        uart_puts("SELF-TEST PASS\r\n");
    } else {
        uart_puts("SELF-TEST FAIL\r\n");
        if (err_buf[0] != '\0') {
            uart_puts("[ERR] ");
            uart_puts(err_buf);
            uart_puts("\r\n");
        }
    }

    /* One-time UART/GPIO register dump to verify RX is enabled and PD0 is input */
    uart_debug_dump();                     // Prints UCSR0A/B/C, UBRR, DDRD, PORTD

    /* Sanity-guard the UART RX pin: PD0 must be INPUT with pull-up (idle high) */
    DDRD  &= ~_BV(DDD0);                   // Ensure PD0 is input
    PORTD |=  _BV(PORTD0);                 // Enable pull-up on PD0

    /* Initialize the framed-protocol handler (clears internal RX buffer) */
    proto_init();

    /* Main loop: keep RX enabled, keep PD0 guarded, and poll the protocol */
    while (1) {
        /* Reassert guards each tick in case later code changes pins/registers */
        DDRD  &= ~_BV(DDD0);               // PD0 stays input
        PORTD |=  _BV(PORTD0);             // Pull-up stays on (UART idle high)
        if (!(UCSR0B & _BV(RXEN0))) {      // If RX ever got disabled…
            UCSR0B |= _BV(RXEN0);          // …re-enable it
        }

        /* Non-blocking protocol poll: ingests UART bytes, parses frames, replies */
        proto_poll_once();
    }
}