#ifndef UART_AVR_H
#define UART_AVR_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize AVR UART0 for the requested baud rate (8N1).
 *
 * Whole-function summary:
 *   Configures the ATmega328P (Arduino Nano) UART0 for 8 data bits, no parity,
 *   1 stop bit at the given baud rate. Uses normal 16x mode for reliability.
 *
 * Notes:
 *   - Assumes F_CPU is defined (16 MHz on a classic Nano).
 *   - Pins: PD0 = RX, PD1 = TX (handled by the UART peripheral).
 */
void uart_init(uint32_t baud);

/**
 * @brief Transmit one byte over UART (blocking until TX register is ready).
 *
 * Whole-function summary:
 *   Waits for the UART data register to become empty, then writes one byte.
 *   Safe to call from simple main loops; no interrupts required.
 */
void uart_putc(uint8_t c);

/**
 * @brief Transmit a buffer of bytes over UART (blocking).
 *
 * Whole-function summary:
 *   Sends 'len' bytes from 'data' by repeatedly calling uart_putc.
 */
void uart_write(const uint8_t *data, uint16_t len);

/**
 * @brief Transmit a C-string over UART (blocking).
 *
 * Whole-function summary:
 *   Sends characters from a NUL-terminated string until the terminator.
 */
void uart_puts(const char *s);

/**
 * @brief Non-blocking read of one byte; returns -1 if no byte available.
 *
 * Whole-function summary:
 *   Checks the UART RX flag; if a byte has arrived, returns it as 0..255.
 *   If no byte is available, returns -1 immediately.
 */
int uart_getc_nonblocking(void);

/**
 * @brief Returns true if at least one byte has been received and is ready.
 */
bool uart_available(void);

#endif /* UART_AVR_H */