#include "crc16.h"

/**
 * @brief Bitwise CRC-16/CCITT-FALSE implementation over a buffer.
 *
 * Whole-function summary:
 *   Computes CCITT-FALSE (poly 0x1021, init 0xFFFF, no final XOR, no reflection)
 *   across the provided byte buffer. Compact and table-free.
 *
 * Line-by-line:
 *   - Start crc at 0xFFFF per spec.
 *   - XOR each byte into the CRC's high byte.
 *   - For each of 8 bits, left-shift and conditionally XOR with 0x1021.
 *   - Return the final 16-bit CRC.
 */
uint16_t crc16_ccitt_false(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;                 // Init for CCITT-FALSE
    for (size_t i = 0; i < len; i++) {     // Walk every byte
        crc ^= ((uint16_t)data[i]) << 8;   // XOR byte into high CRC byte
        for (int b = 0; b < 8; b++) {      // 8 bit-steps MSB-first
            if (crc & 0x8000) {            // If MSB is 1
                crc = (crc << 1) ^ 0x1021; // shift and apply polynomial
            } else {
                crc = (crc << 1);          // else just shift
            }
        }
    }
    return crc;                            // Done
}