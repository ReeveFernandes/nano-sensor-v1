#include "unity.h"
#include "crc16.h"
#include <string.h>

/**
 * @brief Known vector test for CCITT-FALSE.
 * "123456789" => 0x29B1
 */
void test_crc16_known_vector(void) {
    const char *s = "123456789";                         // Canonical vector
    uint16_t crc = crc16_ccitt_false((const uint8_t*)s, strlen(s)); // Compute CRC
    TEST_ASSERT_EQUAL_HEX16(0x29B1, crc);                // Expect 0x29B1
}

/**
 * @brief Streaming update should match whole-buffer CRC result.
 */
void test_crc16_streaming_matches_bulk(void) {
    const uint8_t data[] = {0xDE,0xAD,0xBE,0xEF};        // Sample buffer
    // Bulk compute
    uint16_t bulk = crc16_ccitt_false(data, sizeof(data));
    // Streaming compute (init 0xFFFF per CCITT-FALSE)
    uint16_t s = 0xFFFF;
    for (size_t i = 0; i < sizeof(data); i++) {
        s = crc16_ccitt_false_update(s, data[i]);
    }
    TEST_ASSERT_EQUAL_HEX16(bulk, s);                    // Should match
}