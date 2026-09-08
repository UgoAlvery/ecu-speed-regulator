#include "test_runner.h"
#include "protocol.h"
#include <string.h>

/* ========================================================================== */
/* CRC-16/CCITT (XMODEM)                                                      */
/* ========================================================================== */

static void test_crc_empty_returns_zero(void)
{
    uint8_t dummy = 0;
    TEST_ASSERT_EQUAL_UINT16(0x0000, protocol_compute_crc(&dummy, 0));
}

static void test_crc_known_vector(void)
{
    /* Vecteur de référence XMODEM : "123456789" → 0x31C3 */
    const uint8_t data[] = {'1','2','3','4','5','6','7','8','9'};
    TEST_ASSERT_EQUAL_UINT16(0x31C3, protocol_compute_crc(data, sizeof(data)));
}

static void test_crc_order_sensitive(void)
{
    /* CRC doit être différent si les octets sont inversés */
    const uint8_t a[] = {0x01, 0x02, 0x03};
    const uint8_t b[] = {0x03, 0x02, 0x01};
    TEST_ASSERT_TRUE(protocol_compute_crc(a, 3) != protocol_compute_crc(b, 3));
}

static void test_crc_single_bit_flip_detected(void)
{
    uint8_t a[] = {0xAA, 0xBB, 0xCC, 0xDD};
    uint8_t b[] = {0xAA, 0xBB, 0xCC, 0xDD};
    b[2] ^= 0x01;
    TEST_ASSERT_TRUE(protocol_compute_crc(a, 4) != protocol_compute_crc(b, 4));
}

/* ========================================================================== */
/* protocol_encode — gardes                                                   */
/* ========================================================================== */

static void test_encode_null_dst_returns_zero(void)
{
    TEST_ASSERT_EQUAL_INT(0, (int)protocol_encode(NULL, MSG_SETPOINT, NULL, 0));
}

static void test_encode_payload_too_large_returns_zero(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE] = {0};
    TEST_ASSERT_EQUAL_INT(
        0,
        (int)protocol_encode(buf, MSG_DBG, buf, PROTOCOL_MAX_PAYLOAD_SIZE + 1));
}

/* ========================================================================== */
/* protocol_encode — format de trame sans payload                             */
/* ========================================================================== */

static void test_encode_no_payload_total_length(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    /* AA + LEN(2) + TYPE(1) + CRC(2) = 6 */
    TEST_ASSERT_EQUAL_INT(6, (int)protocol_encode(buf, MSG_SETPOINT, NULL, 0));
}

static void test_encode_no_payload_start_byte(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    protocol_encode(buf, MSG_SETPOINT, NULL, 0);
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_START_BYTE, buf[0]);
}

static void test_encode_no_payload_len_field(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    protocol_encode(buf, MSG_SETPOINT, NULL, 0);
    /* LEN = 1 (TYPE seulement), little-endian */
    TEST_ASSERT_EQUAL_UINT8(0x01, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(0x00, buf[2]);
}

static void test_encode_no_payload_type_field(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    protocol_encode(buf, MSG_SPEED, NULL, 0);
    TEST_ASSERT_EQUAL_UINT8(MSG_SPEED, buf[3]);
}

static void test_encode_no_payload_crc_correct(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_SETPOINT, NULL, 0);
    /* CRC calculé sur tout sauf START (buf[1..len-3]) */
    uint16_t expected = protocol_compute_crc(&buf[1], len - 1 - 2);
    uint16_t encoded  = (uint16_t)buf[len - 2] | ((uint16_t)buf[len - 1] << 8);
    TEST_ASSERT_EQUAL_UINT16(expected, encoded);
}

/* ========================================================================== */
/* protocol_encode — format de trame avec payload                             */
/* ========================================================================== */

static void test_encode_with_payload_total_length(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    const uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF};
    /* AA + LEN(2) + TYPE(1) + 4 + CRC(2) = 10 */
    TEST_ASSERT_EQUAL_INT(10,
        (int)protocol_encode(buf, MSG_DBG, payload, sizeof(payload)));
}

static void test_encode_with_payload_len_field(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    const uint8_t payload[] = {0x01, 0x02, 0x03};
    protocol_encode(buf, MSG_DBG, payload, sizeof(payload));
    /* LEN = 1 + 3 = 4, little-endian */
    TEST_ASSERT_EQUAL_UINT8(0x04, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(0x00, buf[2]);
}

static void test_encode_with_payload_bytes(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    const uint8_t payload[] = {0xCA, 0xFE, 0xBA, 0xBE};
    protocol_encode(buf, MSG_OUTPUT, payload, sizeof(payload));
    TEST_ASSERT_EQUAL_UINT8(0xCA, buf[4]);
    TEST_ASSERT_EQUAL_UINT8(0xFE, buf[5]);
    TEST_ASSERT_EQUAL_UINT8(0xBA, buf[6]);
    TEST_ASSERT_EQUAL_UINT8(0xBE, buf[7]);
}

static void test_encode_max_payload_succeeds(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    uint8_t payload[PROTOCOL_MAX_PAYLOAD_SIZE];
    memset(payload, 0xAB, sizeof(payload));
    size_t len = protocol_encode(buf, MSG_DBG, payload, PROTOCOL_MAX_PAYLOAD_SIZE);
    TEST_ASSERT_TRUE(len > 0);
    /* Longueur totale : 1 + 2 + 1 + 128 + 2 = 134 */
    TEST_ASSERT_EQUAL_INT((int)PROTOCOL_MAX_FRAME_SIZE, (int)len);
}

/* ========================================================================== */
/* protocol_decode — gardes                                                   */
/* ========================================================================== */

static void test_decode_null_frame_returns_false(void)
{
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(NULL, 6, &f));
}

static void test_decode_null_out_returns_false(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_SETPOINT, NULL, 0);
    TEST_ASSERT_FALSE(protocol_decode(buf, len, NULL));
}

static void test_decode_zero_len_returns_false(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    protocol_encode(buf, MSG_SETPOINT, NULL, 0);
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(buf, 0, &f));
}

/* ========================================================================== */
/* protocol_decode — trames invalides                                         */
/* ========================================================================== */

static void test_decode_too_short_returns_false(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_SETPOINT, NULL, 0);
    ecu_frame_t f;
    /* Trame minimale = 6 octets ; passer 5 doit échouer */
    TEST_ASSERT_FALSE(protocol_decode(buf, len - 1, &f));
}

static void test_decode_wrong_start_byte_returns_false(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_SETPOINT, NULL, 0);
    buf[0] = 0xBB;
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(buf, len, &f));
}

static void test_decode_corrupted_crc_returns_false(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_SETPOINT, NULL, 0);
    buf[len - 1] ^= 0xFF;  /* retournement du dernier octet CRC */
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(buf, len, &f));
}

static void test_decode_frame_len_mismatch_returns_false(void)
{
    /* frame_len plus grand que ce qu'annonce LEN field */
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_SETPOINT, NULL, 0);
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(buf, len + 1, &f));
}

static void test_decode_inflated_len_field_returns_false(void)
{
    /* LEN field > frame_len réel */
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_SETPOINT, NULL, 0);
    buf[1] = 0xFF;
    buf[2] = 0x00;
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(buf, len, &f));
}

static void test_decode_payload_overflow_returns_false(void)
{
    /* Trame artisanale : LEN annonce PROTOCOL_MAX_PAYLOAD_SIZE+2,
       soit payload_len = PROTOCOL_MAX_PAYLOAD_SIZE+1 > max autorisé */
    uint8_t buf[200] = {0};
    const uint16_t bad_len  = PROTOCOL_MAX_PAYLOAD_SIZE + 2;   /* 130 */
    const size_t   flen     = 1u + 2u + (size_t)bad_len + 2u; /* 135 */
    buf[0] = PROTOCOL_START_BYTE;
    buf[1] = (uint8_t)(bad_len & 0xFF);
    buf[2] = (uint8_t)(bad_len >> 8);
    buf[3] = MSG_DBG;
    /* payload et CRC laissés à 0 — le rejet intervient avant la vérif CRC */
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(buf, flen, &f));
}

static void test_decode_single_bit_error_detected(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    const uint8_t payload[] = {0xAA, 0xBB, 0xCC};
    size_t len = protocol_encode(buf, MSG_SETPOINT, payload, sizeof(payload));
    buf[4] ^= 0x01;  /* 1 bit retourné dans le payload */
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(buf, len, &f));
}

/* ========================================================================== */
/* protocol_decode — trames valides                                           */
/* ========================================================================== */

static void test_decode_no_payload_returns_true(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_MODE_SET, NULL, 0);
    ecu_frame_t f;
    TEST_ASSERT_TRUE(protocol_decode(buf, len, &f));
}

static void test_decode_no_payload_type(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_MODE_SET, NULL, 0);
    ecu_frame_t f;
    protocol_decode(buf, len, &f);
    TEST_ASSERT_EQUAL_UINT8(MSG_MODE_SET, f.type);
}

static void test_decode_no_payload_len(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_MODE_SET, NULL, 0);
    ecu_frame_t f;
    protocol_decode(buf, len, &f);
    TEST_ASSERT_EQUAL_INT(0, (int)f.payload_len);
}

static void test_decode_with_payload_type(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    const uint8_t payload[] = {0x10, 0x20, 0x30};
    size_t len = protocol_encode(buf, MSG_SPEED, payload, sizeof(payload));
    ecu_frame_t f;
    protocol_decode(buf, len, &f);
    TEST_ASSERT_EQUAL_UINT8(MSG_SPEED, f.type);
}

static void test_decode_with_payload_len(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    const uint8_t payload[] = {0x10, 0x20, 0x30};
    size_t len = protocol_encode(buf, MSG_SPEED, payload, sizeof(payload));
    ecu_frame_t f;
    protocol_decode(buf, len, &f);
    TEST_ASSERT_EQUAL_INT(3, (int)f.payload_len);
}

static void test_decode_with_payload_bytes(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    const uint8_t payload[] = {0x10, 0x20, 0x30};
    size_t len = protocol_encode(buf, MSG_SPEED, payload, sizeof(payload));
    ecu_frame_t f;
    protocol_decode(buf, len, &f);
    TEST_ASSERT_EQUAL_UINT8(0x10, f.payload[0]);
    TEST_ASSERT_EQUAL_UINT8(0x20, f.payload[1]);
    TEST_ASSERT_EQUAL_UINT8(0x30, f.payload[2]);
}

/* ========================================================================== */
/* Aller-retour encode/decode                                                 */
/* ========================================================================== */

static void test_roundtrip_empty_payload(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_ALARM, NULL, 0);
    ecu_frame_t f;
    TEST_ASSERT_TRUE(protocol_decode(buf, len, &f));
    TEST_ASSERT_EQUAL_UINT8(MSG_ALARM, f.type);
    TEST_ASSERT_EQUAL_INT(0, (int)f.payload_len);
}

static void test_roundtrip_max_payload(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    uint8_t payload[PROTOCOL_MAX_PAYLOAD_SIZE];
    for (int i = 0; i < PROTOCOL_MAX_PAYLOAD_SIZE; i++)
        payload[i] = (uint8_t)i;
    size_t len = protocol_encode(buf, MSG_DBG, payload, PROTOCOL_MAX_PAYLOAD_SIZE);
    ecu_frame_t f;
    TEST_ASSERT_TRUE(protocol_decode(buf, len, &f));
    TEST_ASSERT_EQUAL_INT(MSG_DBG, (int)f.type);
    TEST_ASSERT_EQUAL_INT(PROTOCOL_MAX_PAYLOAD_SIZE, (int)f.payload_len);
    TEST_ASSERT_MEM_EQUAL(payload, f.payload, PROTOCOL_MAX_PAYLOAD_SIZE);
}

static void test_roundtrip_all_message_types(void)
{
    const uint8_t types[] = {
        MSG_SETPOINT, MSG_SPEED, MSG_MODE_SET,
        MSG_OUTPUT, MSG_STATS, MSG_ALARM, MSG_DBG
    };
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    ecu_frame_t f;
    for (size_t i = 0; i < sizeof(types); i++) {
        size_t len = protocol_encode(buf, types[i], NULL, 0);
        TEST_ASSERT_TRUE(protocol_decode(buf, len, &f));
        TEST_ASSERT_EQUAL_UINT8(types[i], f.type);
    }
}

/* ========================================================================== */
/* main                                                                       */
/* ========================================================================== */

int main(void)
{
    TEST_SUITE_BEGIN("protocol");

    RUN_TEST(test_crc_empty_returns_zero);
    RUN_TEST(test_crc_known_vector);
    RUN_TEST(test_crc_order_sensitive);
    RUN_TEST(test_crc_single_bit_flip_detected);

    RUN_TEST(test_encode_null_dst_returns_zero);
    RUN_TEST(test_encode_payload_too_large_returns_zero);
    RUN_TEST(test_encode_no_payload_total_length);
    RUN_TEST(test_encode_no_payload_start_byte);
    RUN_TEST(test_encode_no_payload_len_field);
    RUN_TEST(test_encode_no_payload_type_field);
    RUN_TEST(test_encode_no_payload_crc_correct);
    RUN_TEST(test_encode_with_payload_total_length);
    RUN_TEST(test_encode_with_payload_len_field);
    RUN_TEST(test_encode_with_payload_bytes);
    RUN_TEST(test_encode_max_payload_succeeds);

    RUN_TEST(test_decode_null_frame_returns_false);
    RUN_TEST(test_decode_null_out_returns_false);
    RUN_TEST(test_decode_zero_len_returns_false);
    RUN_TEST(test_decode_too_short_returns_false);
    RUN_TEST(test_decode_wrong_start_byte_returns_false);
    RUN_TEST(test_decode_corrupted_crc_returns_false);
    RUN_TEST(test_decode_frame_len_mismatch_returns_false);
    RUN_TEST(test_decode_inflated_len_field_returns_false);
    RUN_TEST(test_decode_payload_overflow_returns_false);
    RUN_TEST(test_decode_single_bit_error_detected);
    RUN_TEST(test_decode_no_payload_returns_true);
    RUN_TEST(test_decode_no_payload_type);
    RUN_TEST(test_decode_no_payload_len);
    RUN_TEST(test_decode_with_payload_type);
    RUN_TEST(test_decode_with_payload_len);
    RUN_TEST(test_decode_with_payload_bytes);

    RUN_TEST(test_roundtrip_empty_payload);
    RUN_TEST(test_roundtrip_max_payload);
    RUN_TEST(test_roundtrip_all_message_types);

    TEST_SUITE_SUMMARY();
}