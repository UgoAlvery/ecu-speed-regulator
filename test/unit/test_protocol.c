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
    TEST_ASSERT_EQUAL_INT(0, (int)protocol_encode(NULL, MSG_SETPOINT, 1, NULL, 0));
}

static void test_encode_payload_too_large_returns_zero(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE] = {0};
    TEST_ASSERT_EQUAL_INT(
        0,
        (int)protocol_encode(buf, MSG_DBG, 1, buf, PROTOCOL_MAX_PAYLOAD_SIZE + 1));
}

/* ========================================================================== */
/* protocol_encode — format de trame sans payload                             */
/* ========================================================================== */

static void test_encode_no_payload_total_length(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    /* AA + LEN(2) + TYPE(1) + NONCE(2) + CRC(2) = 8 */
    TEST_ASSERT_EQUAL_INT(8, (int)protocol_encode(buf, MSG_SETPOINT, 1, NULL, 0));
}

static void test_encode_no_payload_start_byte(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    protocol_encode(buf, MSG_SETPOINT, 1, NULL, 0);
    TEST_ASSERT_EQUAL_UINT8(PROTOCOL_START_BYTE, buf[0]);
}

static void test_encode_no_payload_len_field(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    protocol_encode(buf, MSG_SETPOINT, 1, NULL, 0);
    /* LEN = 1 (TYPE) + 2 (NONCE) = 3, little-endian */
    TEST_ASSERT_EQUAL_UINT8(0x03, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(0x00, buf[2]);
}

static void test_encode_no_payload_type_field(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    protocol_encode(buf, MSG_SPEED, 1, NULL, 0);
    TEST_ASSERT_EQUAL_UINT8(MSG_SPEED, buf[3]);
}

static void test_encode_nonce_field(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    protocol_encode(buf, MSG_SPEED, 0x1234, NULL, 0);
    /* NONCE little-endian, juste après TYPE */
    TEST_ASSERT_EQUAL_UINT8(0x34, buf[4]);
    TEST_ASSERT_EQUAL_UINT8(0x12, buf[5]);
}

static void test_encode_no_payload_crc_correct(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_SETPOINT, 1, NULL, 0);
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
    /* AA + LEN(2) + TYPE(1) + NONCE(2) + 4 + CRC(2) = 12 */
    TEST_ASSERT_EQUAL_INT(12,
        (int)protocol_encode(buf, MSG_DBG, 1, payload, sizeof(payload)));
}

static void test_encode_with_payload_len_field(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    const uint8_t payload[] = {0x01, 0x02, 0x03};
    protocol_encode(buf, MSG_DBG, 1, payload, sizeof(payload));
    /* LEN = 1 (TYPE) + 2 (NONCE) + 3 (PAYLOAD) = 6, little-endian */
    TEST_ASSERT_EQUAL_UINT8(0x06, buf[1]);
    TEST_ASSERT_EQUAL_UINT8(0x00, buf[2]);
}

static void test_encode_with_payload_bytes(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    const uint8_t payload[] = {0xCA, 0xFE, 0xBA, 0xBE};
    protocol_encode(buf, MSG_OUTPUT, 1, payload, sizeof(payload));
    /* PAYLOAD démarre après START+LEN(2)+TYPE+NONCE(2) = offset 6 */
    TEST_ASSERT_EQUAL_UINT8(0xCA, buf[6]);
    TEST_ASSERT_EQUAL_UINT8(0xFE, buf[7]);
    TEST_ASSERT_EQUAL_UINT8(0xBA, buf[8]);
    TEST_ASSERT_EQUAL_UINT8(0xBE, buf[9]);
}

static void test_encode_max_payload_succeeds(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    uint8_t payload[PROTOCOL_MAX_PAYLOAD_SIZE];
    memset(payload, 0xAB, sizeof(payload));
    size_t len = protocol_encode(buf, MSG_DBG, 1, payload, PROTOCOL_MAX_PAYLOAD_SIZE);
    TEST_ASSERT_TRUE(len > 0);
    /* Longueur totale : 1 + 2 + 1 + 2 + 128 + 2 = 136 */
    TEST_ASSERT_EQUAL_INT((int)PROTOCOL_MAX_FRAME_SIZE, (int)len);
}

/* ========================================================================== */
/* protocol_decode — gardes                                                   */
/* ========================================================================== */

static void test_decode_null_frame_returns_false(void)
{
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(NULL, 8, &f));
}

static void test_decode_null_out_returns_false(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_SETPOINT, 1, NULL, 0);
    TEST_ASSERT_FALSE(protocol_decode(buf, len, NULL));
}

static void test_decode_zero_len_returns_false(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    protocol_encode(buf, MSG_SETPOINT, 1, NULL, 0);
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(buf, 0, &f));
}

/* ========================================================================== */
/* protocol_decode — trames invalides                                         */
/* ========================================================================== */

static void test_decode_too_short_returns_false(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_SETPOINT, 1, NULL, 0);
    ecu_frame_t f;
    /* Trame minimale = 8 octets ; passer 7 doit échouer */
    TEST_ASSERT_FALSE(protocol_decode(buf, len - 1, &f));
}

static void test_decode_wrong_start_byte_returns_false(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_SETPOINT, 1, NULL, 0);
    buf[0] = 0xBB;
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(buf, len, &f));
}

static void test_decode_corrupted_crc_returns_false(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_SETPOINT, 1, NULL, 0);
    buf[len - 1] ^= 0xFF;  /* retournement du dernier octet CRC */
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(buf, len, &f));
}

static void test_decode_frame_len_mismatch_returns_false(void)
{
    /* frame_len plus grand que ce qu'annonce LEN field */
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_SETPOINT, 1, NULL, 0);
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(buf, len + 1, &f));
}

static void test_decode_inflated_len_field_returns_false(void)
{
    /* LEN field > frame_len réel */
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_SETPOINT, 1, NULL, 0);
    buf[1] = 0xFF;
    buf[2] = 0x00;
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(buf, len, &f));
}

static void test_decode_payload_overflow_returns_false(void)
{
    /* Trame artisanale : LEN annonce TYPE+NONCE+PROTOCOL_MAX_PAYLOAD_SIZE+1,
       soit payload_len = PROTOCOL_MAX_PAYLOAD_SIZE+1 > max autorisé */
    uint8_t buf[200] = {0};
    const uint16_t bad_len  = 1 + PROTOCOL_NONCE_SIZE + PROTOCOL_MAX_PAYLOAD_SIZE + 1; /* 132 */
    const size_t   flen     = 1u + 2u + (size_t)bad_len + 2u; /* 137 */
    buf[0] = PROTOCOL_START_BYTE;
    buf[1] = (uint8_t)(bad_len & 0xFF);
    buf[2] = (uint8_t)(bad_len >> 8);
    buf[3] = MSG_DBG;
    /* nonce, payload et CRC laissés à 0 — le rejet intervient avant la vérif CRC */
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(buf, flen, &f));
}

static void test_decode_single_bit_error_detected(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    const uint8_t payload[] = {0xAA, 0xBB, 0xCC};
    size_t len = protocol_encode(buf, MSG_SETPOINT, 1, payload, sizeof(payload));
    buf[6] ^= 0x01;  /* 1 bit retourné dans le premier octet du payload */
    ecu_frame_t f;
    TEST_ASSERT_FALSE(protocol_decode(buf, len, &f));
}

/* ========================================================================== */
/* protocol_decode — trames valides                                           */
/* ========================================================================== */

static void test_decode_no_payload_returns_true(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_MODE_SET, 1, NULL, 0);
    ecu_frame_t f;
    TEST_ASSERT_TRUE(protocol_decode(buf, len, &f));
}

static void test_decode_no_payload_type(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_MODE_SET, 1, NULL, 0);
    ecu_frame_t f;
    protocol_decode(buf, len, &f);
    TEST_ASSERT_EQUAL_UINT8(MSG_MODE_SET, f.type);
}

static void test_decode_no_payload_len(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_MODE_SET, 1, NULL, 0);
    ecu_frame_t f;
    protocol_decode(buf, len, &f);
    TEST_ASSERT_EQUAL_INT(0, (int)f.payload_len);
}

static void test_decode_nonce_field(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    size_t len = protocol_encode(buf, MSG_MODE_SET, 0xBEEF, NULL, 0);
    ecu_frame_t f;
    protocol_decode(buf, len, &f);
    TEST_ASSERT_EQUAL_UINT16(0xBEEF, f.nonce);
}

static void test_decode_with_payload_type(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    const uint8_t payload[] = {0x10, 0x20, 0x30};
    size_t len = protocol_encode(buf, MSG_SPEED, 1, payload, sizeof(payload));
    ecu_frame_t f;
    protocol_decode(buf, len, &f);
    TEST_ASSERT_EQUAL_UINT8(MSG_SPEED, f.type);
}

static void test_decode_with_payload_len(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    const uint8_t payload[] = {0x10, 0x20, 0x30};
    size_t len = protocol_encode(buf, MSG_SPEED, 1, payload, sizeof(payload));
    ecu_frame_t f;
    protocol_decode(buf, len, &f);
    TEST_ASSERT_EQUAL_INT(3, (int)f.payload_len);
}

static void test_decode_with_payload_bytes(void)
{
    uint8_t buf[PROTOCOL_MAX_FRAME_SIZE];
    const uint8_t payload[] = {0x10, 0x20, 0x30};
    size_t len = protocol_encode(buf, MSG_SPEED, 1, payload, sizeof(payload));
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
    size_t len = protocol_encode(buf, MSG_ALARM, 1, NULL, 0);
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
    size_t len = protocol_encode(buf, MSG_DBG, 1, payload, PROTOCOL_MAX_PAYLOAD_SIZE);
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
        size_t len = protocol_encode(buf, types[i], 1, NULL, 0);
        TEST_ASSERT_TRUE(protocol_decode(buf, len, &f));
        TEST_ASSERT_EQUAL_UINT8(types[i], f.type);
    }
}

/* ========================================================================== */
/* Nonce anti-rejeu — protocol_nonce_check_and_update                        */
/* ========================================================================== */

static void test_nonce_ctx_init_null_no_crash(void)
{
    protocol_nonce_ctx_init(NULL);
    TEST_ASSERT_TRUE(true);
}

static void test_nonce_ctx_init_resets_last_nonce(void)
{
    protocol_nonce_ctx_t ctx;
    ctx.last_nonce = 0xDEAD;
    protocol_nonce_ctx_init(&ctx);
    TEST_ASSERT_EQUAL_UINT16(0, ctx.last_nonce);
}

static void test_nonce_check_null_ctx_returns_false(void)
{
    TEST_ASSERT_FALSE(protocol_nonce_check_and_update(NULL, 1));
}

static void test_nonce_first_frame_accepted(void)
{
    protocol_nonce_ctx_t ctx;
    protocol_nonce_ctx_init(&ctx);
    TEST_ASSERT_TRUE(protocol_nonce_check_and_update(&ctx, 1));
}

static void test_nonce_zero_rejected_on_fresh_ctx(void)
{
    /* nonce=0 == last_nonce initial (0) : ni rejeu, ni "postérieur" — rejeté */
    protocol_nonce_ctx_t ctx;
    protocol_nonce_ctx_init(&ctx);
    TEST_ASSERT_FALSE(protocol_nonce_check_and_update(&ctx, 0));
}

static void test_nonce_monotonic_sequence_accepted(void)
{
    protocol_nonce_ctx_t ctx;
    protocol_nonce_ctx_init(&ctx);
    for (uint16_t n = 1; n <= 10; n++) {
        TEST_ASSERT_TRUE(protocol_nonce_check_and_update(&ctx, n));
    }
    TEST_ASSERT_EQUAL_UINT16(10, ctx.last_nonce);
}

static void test_nonce_duplicate_rejected(void)
{
    protocol_nonce_ctx_t ctx;
    protocol_nonce_ctx_init(&ctx);
    TEST_ASSERT_TRUE(protocol_nonce_check_and_update(&ctx, 5));
    TEST_ASSERT_FALSE(protocol_nonce_check_and_update(&ctx, 5));
}

static void test_nonce_older_rejected(void)
{
    protocol_nonce_ctx_t ctx;
    protocol_nonce_ctx_init(&ctx);
    TEST_ASSERT_TRUE(protocol_nonce_check_and_update(&ctx, 10));
    TEST_ASSERT_FALSE(protocol_nonce_check_and_update(&ctx, 3));
}

static void test_nonce_gap_accepted(void)
{
    /* Trame perdue en chemin : un saut dans la séquence reste accepté,
       l'anti-rejeu n'exige pas la continuité, juste la monotonie. */
    protocol_nonce_ctx_t ctx;
    protocol_nonce_ctx_init(&ctx);
    TEST_ASSERT_TRUE(protocol_nonce_check_and_update(&ctx, 1));
    TEST_ASSERT_TRUE(protocol_nonce_check_and_update(&ctx, 100));
}

static void test_nonce_rejection_does_not_update_ctx(void)
{
    protocol_nonce_ctx_t ctx;
    protocol_nonce_ctx_init(&ctx);
    TEST_ASSERT_TRUE(protocol_nonce_check_and_update(&ctx, 20));
    TEST_ASSERT_FALSE(protocol_nonce_check_and_update(&ctx, 20));
    TEST_ASSERT_EQUAL_UINT16(20, ctx.last_nonce);  /* inchangé par le rejet */
}

static void test_nonce_wraparound_tolerated(void)
{
    /* 0xFFFF → 0x0000 : "suivant" au sens circulaire, doit être accepté. */
    protocol_nonce_ctx_t ctx;
    protocol_nonce_ctx_init(&ctx);
    ctx.last_nonce = 0xFFFF;
    TEST_ASSERT_TRUE(protocol_nonce_check_and_update(&ctx, 0x0000));
}

static void test_nonce_half_window_ambiguity_rejected(void)
{
    /* Limite connue de la comparaison modulaire (comme les seq. num. TCP) :
       au-delà d'un demi-cycle (32768), un nonce "futur" légitime et un rejeu
       ancien deviennent indistinguables ; le choix est de rejeter (ctx à 0,
       nonce à 0x8000 est interprété comme "dans le passé"). */
    protocol_nonce_ctx_t ctx;
    protocol_nonce_ctx_init(&ctx);
    TEST_ASSERT_FALSE(protocol_nonce_check_and_update(&ctx, 0x8000));
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
    RUN_TEST(test_encode_nonce_field);
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
    RUN_TEST(test_decode_nonce_field);
    RUN_TEST(test_decode_with_payload_type);
    RUN_TEST(test_decode_with_payload_len);
    RUN_TEST(test_decode_with_payload_bytes);

    RUN_TEST(test_roundtrip_empty_payload);
    RUN_TEST(test_roundtrip_max_payload);
    RUN_TEST(test_roundtrip_all_message_types);

    RUN_TEST(test_nonce_ctx_init_null_no_crash);
    RUN_TEST(test_nonce_ctx_init_resets_last_nonce);
    RUN_TEST(test_nonce_check_null_ctx_returns_false);
    RUN_TEST(test_nonce_first_frame_accepted);
    RUN_TEST(test_nonce_zero_rejected_on_fresh_ctx);
    RUN_TEST(test_nonce_monotonic_sequence_accepted);
    RUN_TEST(test_nonce_duplicate_rejected);
    RUN_TEST(test_nonce_older_rejected);
    RUN_TEST(test_nonce_gap_accepted);
    RUN_TEST(test_nonce_rejection_does_not_update_ctx);
    RUN_TEST(test_nonce_wraparound_tolerated);
    RUN_TEST(test_nonce_half_window_ambiguity_rejected);

    TEST_SUITE_SUMMARY();
}
