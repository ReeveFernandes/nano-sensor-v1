#include "unity.h"
#include "crc16.h"
#include "frame.h"
#include <string.h>

/**
 * @brief Round-trip test: encode payload then decode and verify fields match.
 */
void test_frame_round_trip(void) {
    const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};   /* Sample payload */
    uint8_t buf[64];                                      /* Output frame buffer */
    uint16_t frame_len = 0;                               /* Will hold frame size */

    /* Encode a DATA frame */
    frame_result_t er = frame_encode(FT_DATA, payload, sizeof(payload),
                                     buf, sizeof(buf), &frame_len);
    TEST_ASSERT_EQUAL(FR_OK, er);                         /* Encoding should pass */
    TEST_ASSERT_TRUE(frame_len >= 8);                     /* At least minimal size */

    /* Decode it back */
    uint8_t out_type = 0;
    uint8_t out_payload[16] = {0};
    uint16_t out_plen = 0;
    frame_result_t dr = frame_decode(buf, frame_len,
                                     &out_type, out_payload, sizeof(out_payload), &out_plen);
    TEST_ASSERT_EQUAL(FR_OK, dr);                         /* Decoding should pass */
    TEST_ASSERT_EQUAL_UINT8(FT_DATA, out_type);           /* Type preserved */
    TEST_ASSERT_EQUAL_UINT16(sizeof(payload), out_plen);  /* Length preserved */
    TEST_ASSERT_EQUAL_UINT8_ARRAY(payload, out_payload, sizeof(payload)); /* Bytes match */
}

/**
 * @brief CRC failure: corrupt a byte and expect FR_ERR_BAD_CRC.
 */
void test_frame_crc_failure(void) {
    const uint8_t payload[] = {1,2,3,4,5};
    uint8_t buf[64];
    uint16_t frame_len = 0;

    TEST_ASSERT_EQUAL(FR_OK, frame_encode(FT_STATUS, payload, sizeof(payload),
                                          buf, sizeof(buf), &frame_len));

    /* Flip the last CRC byte to force a CRC mismatch */
    buf[frame_len - 1] ^= 0xFF;

    uint8_t out_type;
    uint8_t out_payload[16];
    uint16_t out_plen;
    TEST_ASSERT_EQUAL(FR_ERR_BAD_CRC, frame_decode(buf, frame_len,
                        &out_type, out_payload, sizeof(out_payload), &out_plen));
}

/**
 * @brief Bad sync: first two bytes aren’t 0xAA,0x55 → FR_ERR_BAD_SYNC.
 */
void test_frame_bad_sync(void) {
    uint8_t bogus[8] = {0};               /* Minimal-length buffer but wrong sync */
    bogus[0] = 0x00; bogus[1] = 0x00;     /* Should be 0xAA, 0x55 */
    bogus[2] = FRAME_VERSION;
    bogus[3] = FT_ACK;
    bogus[4] = 0x00; bogus[5] = 0x00;     /* len=0 */
    bogus[6] = 0x00; bogus[7] = 0x00;     /* dummy CRC (won’t be checked due to sync fail) */

    uint8_t type;
    uint8_t payload[1];
    uint16_t plen;
    TEST_ASSERT_EQUAL(FR_ERR_BAD_SYNC, frame_decode(bogus, sizeof(bogus),
                        &type, payload, sizeof(payload), &plen));
}

/**
 * @brief Length mismatch: drop a byte from the encoded frame and expect mismatch.
 */
void test_frame_length_mismatch(void) {
    const uint8_t payload[] = {9,8,7,6};
    uint8_t buf[64];
    uint16_t frame_len = 0;

    TEST_ASSERT_EQUAL(FR_OK, frame_encode(FT_LOG, payload, sizeof(payload),
                                          buf, sizeof(buf), &frame_len));

    /* Call decode with one fewer byte than declared */
    uint8_t out_type;
    uint8_t out_payload[16];
    uint16_t out_plen;
    TEST_ASSERT_EQUAL(FR_ERR_LENGTH_MISMATCH, frame_decode(buf, frame_len - 1,
                        &out_type, out_payload, sizeof(out_payload), &out_plen));
}

/**
 * @brief Peek total length from header so a stream reader knows how many bytes to wait for.
 */
void test_frame_peek_length(void) {
    const uint8_t payload[] = {0xAA, 0xBB};
    uint8_t buf[32];
    uint16_t frame_len = 0;

    TEST_ASSERT_EQUAL(FR_OK, frame_encode(FT_ACK, payload, sizeof(payload),
                                          buf, sizeof(buf), &frame_len));

    uint16_t peek_len = 0;
    TEST_ASSERT_EQUAL(FR_OK, frame_peek_length(buf, 6, &peek_len)); /* header present */
    TEST_ASSERT_EQUAL_UINT16(frame_len, peek_len);                  /* Should match real frame size */
}