#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>
#include <stddef.h>

/**
 * @brief Compute CRC-16/CCITT-FALSE over a byte buffer.
 *
 * Polynomial: 0x1021, Init: 0xFFFF, RefIn: false, RefOut: false, XorOut: 0x0000
 * Use for link-layer frame checks on our serial protocol.
 *
 * @param data Pointer to input bytes.
 * @param len  Number of bytes to process.
 * @return 16-bit CRC value.
 */
uint16_t crc16_ccitt_false(const uint8_t *data, size_t len);

/**
 * @brief Update CCITT-FALSE CRC with one byte (streaming mode).
 *
 * Whole-function summary:
 *   Given an existing CCITT-FALSE CRC and the next input byte, returns
 *   the updated CRC. Use this when bytes arrive one at a time (UART ISR).
 *
 * Line-by-line:
 *   - XOR the input byte into high byte of CRC.
 *   - For each of 8 bits, shift left; if MSB was 1, XOR with 0x1021.
 *   - Return the updated CRC.
 */
static inline uint16_t crc16_ccitt_false_update(uint16_t crc, uint8_t byte) {
    crc ^= (uint16_t)byte << 8;               // Mix byte into high CRC byte
    for (int b = 0; b < 8; b++) {             // Process 8 bits MSB-first
        if (crc & 0x8000) {                   // If MSB set before shift...
            crc = (crc << 1) ^ 0x1021;        // shift and apply polynomial
        } else {
            crc = (crc << 1);                 // else just shift
        }
    }
    return crc;                               // Updated 16-bit CRC
}

#endif /* CRC16_H */