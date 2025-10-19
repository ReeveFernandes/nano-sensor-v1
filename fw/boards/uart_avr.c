#include "uart_avr.h"
#include <avr/io.h>

/**
 * @brief Initialize UART0 for 115200 (or given baud), 8N1, RX+TX enabled.
 *
 * Whole-function summary:
 *   Sets ATmega328P USART0 to double-speed mode for better 115200 accuracy,
 *   configures PD1 (TX) as output and PD0 (RX) as input with pull-up,
 *   enables both receiver and transmitter, sets 8 data bits / no parity /
 *   1 stop bit, and flushes any stale RX data.
 *
 * Line-by-line:
 *   - Configure pins: PD1=output (TX), PD0=input (RX) + pull-up to keep idle high.
 *   - Enable double-speed (U2X0=1) so UBRR calc uses F_CPU/(8*baud) - 1.
 *   - Compute and set UBRR0H/L from F_CPU and baud.
 *   - Enable RXEN0|TXEN0 to turn on receiver and transmitter.
 *   - Set UCSZ01|UCSZ00 for 8 data bits, no parity, 1 stop (8N1).
 *   - Drain any pending RX bytes from UDR0 so we start clean.
 */
void uart_init(uint32_t baud)
{
    /* --- Pins: PD1 (TX) output, PD0 (RX) input with pull-up --- */
    DDRD  |= _BV(DDD1);        // TX pin as output
    DDRD  &= ~_BV(DDD0);       // RX pin as input
    PORTD |= _BV(PORTD0);      // Enable pull-up on RX (idle high for UART)

    /* --- Double-speed mode for accurate 115200 at 16 MHz --- */
    UCSR0A = _BV(U2X0);

    /* --- Baud divisor (rounded) for U2X0=1: UBRR = F_CPU/(8*baud) - 1 --- */
    uint32_t ubrr = (F_CPU + (baud * 4UL)) / (8UL * baud) - 1UL;
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFF);

    /* --- Enable receiver and transmitter (no interrupts) --- */
    UCSR0B = _BV(RXEN0) | _BV(TXEN0);

    /* --- 8 data bits, no parity, 1 stop (8N1) --- */
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);

    /* --- Flush stale RX bytes --- */
    while (UCSR0A & _BV(RXC0)) { (void)UDR0; }
}

/**
 * @brief Print key UART and GPIO registers once to verify RX is enabled.
 *
 * Whole-function summary:
 *   Dumps UCSR0A/B/C, UBRR0H/L, DDRD, and PORTD in hex so we can confirm
 *   the USART is configured for RX and that PD0 (RX pin) is input with pull-up.
 *
 * Line-by-line:
 *   - Small hex helper prints a single byte as "0xHH".
 *   - Print each register label and its current value, then CRLF.
 */
void uart_debug_dump(void)
{
    // ---- helper: print one byte as hex without printf ----
    auto void put_hex8(uint8_t v) {
        const char *hx = "0123456789ABCDEF";
        uart_putc('0'); uart_putc('x');
        uart_putc(hx[(v >> 4) & 0xF]);
        uart_putc(hx[v & 0xF]);
    };

    uart_puts("[DBG] UCSR0A="); put_hex8(UCSR0A);
    uart_puts(" UCSR0B=");     put_hex8(UCSR0B);
    uart_puts(" UCSR0C=");     put_hex8(UCSR0C);
    uart_puts(" UBRR0H=");     put_hex8(UBRR0H);
    uart_puts(" UBRR0L=");     put_hex8(UBRR0L);
    uart_puts(" DDRD=");       put_hex8(DDRD);
    uart_puts(" PORTD=");      put_hex8(PORTD);
    uart_puts("\r\n");
}

/**
 * @brief Transmit one byte over UART (blocking until TX register is ready).
 *
 * Whole-function summary:
 *   Waits for UDRE0 (data register empty) then writes UDR0 with the byte.
 *   This blocks briefly at 115200 baud but keeps the code simple and robust.
 */
void uart_putc(uint8_t c) {
    while ((UCSR0A & (1 << UDRE0)) == 0) {       // Wait until transmit buffer is empty
        ;                                        // (spin)
    }
    UDR0 = c;                                    // Write byte to UART data register
}

/**
 * @brief Transmit a buffer of bytes over UART (blocking).
 *
 * Whole-function summary:
 *   Sends 'len' bytes from 'data' by repeatedly calling uart_putc.
 */
void uart_write(const uint8_t *data, uint16_t len) {
    for (uint16_t i = 0; i < len; i++) {         // Iterate over each byte in the buffer
        uart_putc(data[i]);                      // Send one byte
    }
}

/**
 * @brief Transmit a C-string over UART, normalizing '\n' to "\r\n".
 *
 * Whole-function summary:
 *   Sends characters from a NUL-terminated string and converts any lone
 *   newline ('\n') into the CRLF pair ("\r\n") so terminals display each
 *   line starting at column 0. This produces clean, left-aligned logs on
 *   macOS/Linux serial monitors and avoids the "stair-step" indentation.
 *
 * Line-by-line:
 *   - Walk each character in the input string.
 *   - If the char is '\n', send '\r' first, then '\n'.
 *   - Otherwise send the character as-is.
 */
void uart_puts(const char *s) {
    // Loop until we reach the string terminator
    while (*s) {
        // If this character is a newline, emit CRLF for clean terminals
        if (*s == '\n') {
            uart_putc('\r');           // Carriage return: move to column 0
            uart_putc('\n');           // Line feed: move to next line
            s++;                       // Advance to next character
        } else {
            uart_putc((uint8_t)*s++);  // Send character as-is
        }
    }
}

/**
 * @brief Non-blocking read of one byte from UART0.
 *
 * Whole-function summary:
 *   Returns the next received byte if available without blocking; otherwise
 *   returns -1 to indicate “no data”. Safe to call frequently in a poll loop.
 *
 * Line-by-line:
 *   - Check RXC0 flag (receive complete) in UCSR0A.
 *   - If set, read UDR0 and return as int [0..255].
 *   - If not set, return -1.
 */
int uart_getc_nonblocking(void)
{
    if (UCSR0A & _BV(RXC0)) {        // Data received?
        return (int)UDR0;            // Return byte (promoted to int)
    }
    return -1;                       // No byte available
}

/**
 * @brief Returns true if at least one byte has been received and is ready.
 *
 * Whole-function summary:
 *   Convenience wrapper around the RXC0 flag so callers can test availability.
 */
bool uart_available(void) {
    return (UCSR0A & (1 << RXC0)) != 0;          // True if data is waiting
}