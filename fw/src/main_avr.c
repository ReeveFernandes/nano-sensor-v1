/**
 * @brief Minimal loop + timed streaming: service frames and send FT_DATA at rate.
 *
 * Whole-function summary:
 *   - Initializes UART @115200, ADC on A0, and the framed protocol.
 *   - Continuously services incoming frames (PING/START/STOP/SET_RATE/GET_STATUS).
 *   - When g_sampling_on is true, every g_sample_rate_ms it reads ADC0 and sends
 *     one FT_DATA frame containing the 16-bit sample (big-endian [hi, lo]).
 *
 * Notes:
 *   - No boot banners or debug prints (wire stays binary-clean).
 *   - Uses a simple 1 ms software tick via _delay_ms(1).
 */
#include <avr/io.h>
#include <util/delay.h>
#include <stdbool.h>
#include "uart_avr.h"
#include "proto.h"
#include "cmd_parser.h"
#include "adc_avr.h"

// Provide storage for externs so GET_STATUS/SET_RATE work.
volatile uint16_t g_sample_rate_ms = 200;   // default 200 ms for a lively plot
volatile bool     g_sampling_on     = false;

// Forward from proto.c
void proto_send_sample_u16(uint16_t sample_u16);

int main(void) {
    // 1) UART up
    uart_init(115200);
    _delay_ms(10);

    // 2) ADC on A0
    adc_init();

    // 3) Guard RX pin and ensure RX stays enabled
    DDRD  &= ~(1 << DDD0);    // PD0 as input
    PORTD |=  (1 << PORTD0);  // pull-up ON
    UCSR0B |=  (1 << RXEN0);  // RX enable

    // 4) Protocol init
    proto_init();

    // 5) Simple ms ticker and next-due schedule
    uint32_t ms_tick = 0;
    uint32_t next_due = 0;

    while (1) {
        // keep RX healthy
        DDRD  &= ~(1 << DDD0);
        PORTD |=  (1 << PORTD0);
        if (!(UCSR0B & (1 << RXEN0))) UCSR0B |= (1 << RXEN0);

        // service inbound framed commands
        proto_poll_once();

        // 1 ms tick
        _delay_ms(1);
        ms_tick++;

        // If streaming enabled, send a sample whenever due
        if (g_sampling_on) {
            if (ms_tick >= next_due) {
                uint16_t s = adc_read10();              // 0..1023
                proto_send_sample_u16(s);               // FT_DATA: [hi, lo]
                // schedule next send (guard against rate=0 by enforcing >=10ms)
                uint16_t period = (g_sample_rate_ms < 10) ? 10 : g_sample_rate_ms;
                next_due = ms_tick + period;
            }
        } else {
            // when stopped, follow status rate if someone changes it before starting
            next_due = ms_tick + g_sample_rate_ms;
        }
    }
}