#include "unity.h"
#include "cmd_parser.h"

/* Provide storage for the externs declared in cmd_parser.h (for tests) */
volatile uint16_t g_sample_rate_ms = 1000; /* default 1000 ms */
volatile bool     g_sampling_on     = false;

/**
 * @brief PING should return FT_ACK with "PONG" payload.
 */
void test_cmd_ping_ack_pong(void) {
    const uint8_t in[] = { OP_PING };      // Input payload: [opcode]
    uint8_t out[8] = {0};                  // Output buffer for reply payload
    uint16_t out_len = 0;                  // Will capture reply length

    uint8_t t = cmd_parse_and_apply(in, sizeof(in), out, &out_len);
    TEST_ASSERT_EQUAL_UINT8(FT_ACK, t);    // Expect ACK frame type
    TEST_ASSERT_EQUAL_UINT16(4, out_len);  // "PONG" is 4 bytes
    TEST_ASSERT_EQUAL_UINT8('P', out[0]);
    TEST_ASSERT_EQUAL_UINT8('O', out[1]);
    TEST_ASSERT_EQUAL_UINT8('N', out[2]);
    TEST_ASSERT_EQUAL_UINT8('G', out[3]);
}

/**
 * @brief START then STOP toggles g_sampling_on.
 */
void test_cmd_start_stop_toggle(void) {
    /* START */
    const uint8_t start_in[] = { OP_START };
    uint8_t out[4] = {0};
    uint16_t out_len = 0;
    uint8_t t1 = cmd_parse_and_apply(start_in, sizeof(start_in), out, &out_len);
    TEST_ASSERT_EQUAL_UINT8(FT_ACK, t1);
    TEST_ASSERT_EQUAL_UINT16(0, out_len);
    TEST_ASSERT_TRUE(g_sampling_on);

    /* STOP */
    const uint8_t stop_in[] = { OP_STOP };
    uint8_t t2 = cmd_parse_and_apply(stop_in, sizeof(stop_in), out, &out_len);
    TEST_ASSERT_EQUAL_UINT8(FT_ACK, t2);
    TEST_ASSERT_EQUAL_UINT16(0, out_len);
    TEST_ASSERT_FALSE(g_sampling_on);
}

/**
 * @brief SET_RATE with a good value should ACK and update g_sample_rate_ms.
 */
void test_cmd_set_rate_good(void) {
    const uint16_t new_ms = 500;               // within [10..10000]
    const uint8_t in[] = { OP_SET_RATE, (uint8_t)(new_ms >> 8), (uint8_t)(new_ms & 0xFF) };
    uint8_t out[4] = {0};
    uint16_t out_len = 0;

    uint8_t t = cmd_parse_and_apply(in, sizeof(in), out, &out_len);
    TEST_ASSERT_EQUAL_UINT8(FT_ACK, t);
    TEST_ASSERT_EQUAL_UINT16(0, out_len);
    TEST_ASSERT_EQUAL_UINT16(new_ms, g_sample_rate_ms);
}

/**
 * @brief SET_RATE wrong length → FT_NACK + ERR_BAD_LEN.
 */
void test_cmd_set_rate_bad_len(void) {
    const uint8_t in[] = { OP_SET_RATE /* missing 2-byte arg */ };
    uint8_t out[4] = {0};
    uint16_t out_len = 0;

    uint8_t t = cmd_parse_and_apply(in, sizeof(in), out, &out_len);
    TEST_ASSERT_EQUAL_UINT8(FT_NACK, t);
    TEST_ASSERT_EQUAL_UINT16(1, out_len);
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_LEN, out[0]);
}

/**
 * @brief SET_RATE out of range → FT_NACK + ERR_BAD_ARG.
 */
void test_cmd_set_rate_bad_arg(void) {
    const uint16_t too_small = 5;              // below CMD_RATE_MIN_MS (10)
    const uint8_t in[] = { OP_SET_RATE, (uint8_t)(too_small >> 8), (uint8_t)(too_small & 0xFF) };
    uint8_t out[4] = {0};
    uint16_t out_len = 0;

    uint8_t t = cmd_parse_and_apply(in, sizeof(in), out, &out_len);
    TEST_ASSERT_EQUAL_UINT8(FT_NACK, t);
    TEST_ASSERT_EQUAL_UINT16(1, out_len);
    TEST_ASSERT_EQUAL_UINT8(ERR_BAD_ARG, out[0]);
}

/**
 * @brief GET_STATUS returns FT_STATUS with [on][rate_hi][rate_lo].
 */
void test_cmd_get_status_payload(void) {
    g_sampling_on = true;                      // set known state
    g_sample_rate_ms = 750;                    // 0x02EE
    const uint8_t in[] = { OP_GET_STATUS };
    uint8_t out[8] = {0};
    uint16_t out_len = 0;

    uint8_t t = cmd_parse_and_apply(in, sizeof(in), out, &out_len);
    TEST_ASSERT_EQUAL_UINT8(FT_STATUS, t);
    TEST_ASSERT_EQUAL_UINT16(3, out_len);
    TEST_ASSERT_EQUAL_UINT8(1, out[0]);       // ON flag
    TEST_ASSERT_EQUAL_UINT8(0x02, out[1]);    // 750 >> 8
    TEST_ASSERT_EQUAL_UINT8(0xEE, out[2]);    // 750 & 0xFF
}