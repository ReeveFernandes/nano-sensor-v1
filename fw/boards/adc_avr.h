#ifndef ADC_AVR_H
#define ADC_AVR_H

/**
 * @file adc_avr.h
 * @brief Minimal ADC driver for ATmega328P (Arduino-class AVRs), channel ADC0 (A0).
 *
 * Whole-file summary:
 *   Provides two inline helpers: adc_init() to configure the ADC
 *   and adc_read10() to fetch a single 10-bit sample from ADC0.
 *   Targets 16 MHz CPU clock; uses AVcc reference and 125 kHz ADC clock.
 */

#include <stdint.h>
#include <avr/io.h>  /* <-- REQUIRED: brings in ADMUX/ADCSRA/ADCL/ADCH, etc. */

/**
 * @brief Initialize ADC0 with AVcc reference and 125 kHz ADC clock.
 *
 * Whole-function summary:
 *   - Select AVcc as voltage reference (REFS0=1).
 *   - Select input channel ADC0 (MUX[3:0]=0000).
 *   - Enable ADC and set prescaler to /128 (ADPS2:0=111) for ~125 kHz at 16 MHz F_CPU.
 */
static inline void adc_init(void) {
    ADMUX  = (1 << REFS0);                                               /* AVcc ref, ADC0 channel */
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);   /* enable + prescale /128 */
}

/**
 * @brief Read one 10-bit sample from ADC0 (blocking).
 *
 * Whole-function summary:
 *   Starts a conversion (ADSC=1), waits until complete (ADSC=0),
 *   then reads ADCL first, then ADCH to latch the 10-bit result.
 *
 * @return 10-bit sample in range 0..1023
 */
static inline uint16_t adc_read10(void) {
    ADCSRA |= (1 << ADSC);                  /* start conversion */
    while (ADCSRA & (1 << ADSC)) { ; }      /* wait complete    */
    uint8_t low  = ADCL;                    /* read low first   */
    uint8_t high = ADCH;                    /* then high        */
    return (uint16_t)((high << 8) | low);
}

#endif /* ADC_AVR_H */