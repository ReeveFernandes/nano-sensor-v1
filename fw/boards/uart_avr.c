#include "uart_avr.h"
#include <avr/io.h>

/**
 * @brief Initialize AVR UART0 for the requested baud rate (8N1).
 *
 * Whole-function summary:
 *   Computes the UBRR value for 16x UART mode, programs the baud registers,
 *   sets frame format to 8N1, and enables RX & TX.
 *
 * Line-by-line notes clarify each register write.
 */
void uart_init(uint32_t baud) {
    #ifndef F_CPU
    #define F_CPU 16000000UL                       // Default to 16 MHz (Arduino Nano)
    #endif

    // Enable double-speed mode (U2X0=1) for better accuracy at high baud rates (e.g., 115200)
    UCSR0A = (1 << U2X0);

    // Rounded UBRR for U2X mode: UBRR = (F_CPU/(8*baud)) - 1, with +4*baud to round
    uint16_t ubrr = (uint16_t)((F_CPU + (baud * 4UL)) / (8UL * baud) - 1UL);

    UBRR0H = (uint8_t)(ubrr >> 8);                 // Set high byte of baud divider
    UBRR0L = (uint8_t)(ubrr & 0xFF);               // Set low byte of baud divider

    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);        // 8 data bits, no parity, 1 stop (8N1)
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);          // Enable RX and TX
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
 * @brief Non-blocking read of one byte; returns -1 if no byte available.
 *
 * Whole-function summary:
 *   Checks RXC0 (Receive Complete). If set, reads UDR0 and returns it.
 *   Otherwise, returns -1 immediately so callers can poll without blocking.
 */
int uart_getc_nonblocking(void) {
    if (UCSR0A & (1 << RXC0)) {                  // If a byte has been received...
        return (int)UDR0;                        // ...read and return it (0..255)
    }
    return -1;                                   // No data available
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