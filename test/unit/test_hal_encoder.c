#include "test_runner.h"
#include "hal_encoder.h"
#include "hal_config.h"

#include <stdint.h>
#include <limits.h>

void hal_encoder_mock_set_count(int32_t count);
void hal_encoder_mock_reset_state(void);

static void reset(void)
{
    hal_encoder_mock_reset_state();
}

/* ── Tests ─────────────────────────────────────────────────────────────────*/

static void test_init_clears_count(void)
{
    reset();
    hal_encoder_mock_set_count(42);
    TEST_ASSERT_TRUE(hal_encoder_init());
    TEST_ASSERT_EQUAL_INT(0, hal_encoder_get_count());
}

static void test_get_count_returns_mock_value(void)
{
    reset();
    hal_encoder_init();
    hal_encoder_mock_set_count(100);
    TEST_ASSERT_EQUAL_INT(100, hal_encoder_get_count());
}

static void test_get_count_negative(void)
{
    reset();
    hal_encoder_init();
    hal_encoder_mock_set_count(-50);
    TEST_ASSERT_EQUAL_INT(-50, hal_encoder_get_count());
}

static void test_reset_clears_count(void)
{
    reset();
    hal_encoder_init();
    hal_encoder_mock_set_count(200);
    hal_encoder_reset();
    TEST_ASSERT_EQUAL_INT(0, hal_encoder_get_count());
}

static void test_get_count_before_init_returns_zero(void)
{
    reset();  /* s_initialized = false */
    TEST_ASSERT_EQUAL_INT(0, hal_encoder_get_count());
}

#if !defined(HAL_ENC_TEST_REALISTIC) && !defined(HAL_ENC_TEST_OVERFLOW)
/* Only meaningful when HAL_ENC_PULSES_PER_MOTOR_REV == 0 (default TODO state). */
static void test_speed_returns_zero_when_pulses_per_rev_is_zero(void)
{
    reset();
    hal_encoder_init();
    hal_encoder_mock_set_count(1000);
    const float speed = hal_encoder_get_speed_kmh(0.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, speed);
}
#endif

static void test_speed_returns_zero_when_dt_is_zero(void)
{
    reset();
    hal_encoder_init();
    hal_encoder_mock_set_count(500);
    const float speed = hal_encoder_get_speed_kmh(0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, speed);
}

static void test_speed_returns_zero_when_dt_is_negative(void)
{
    reset();
    hal_encoder_init();
    hal_encoder_mock_set_count(500);
    const float speed = hal_encoder_get_speed_kmh(-0.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, speed);
}

static void test_speed_resets_count_after_reading(void)
{
    /* get_speed_kmh must call reset internally, so successive calls return 0
     * when no new pulses have been injected. */
    reset();
    hal_encoder_init();
    hal_encoder_mock_set_count(1000);
    hal_encoder_get_speed_kmh(0.1f);  /* consumes count */
    /* Count should be reset now; a second call with no new pulses → 0. */
    const float speed2 = hal_encoder_get_speed_kmh(0.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, speed2);
}

static void test_cpr_gear_product_no_overflow(void)
{
    /* For realistic FIT0522 values (64 CPR, 120:1 gear), product is 7680 —
     * well within uint32_t. The formula uses uint64_t cast to protect against
     * large encoders; this test documents the expected range. */
    const uint64_t realistic = (uint64_t)64u * (uint64_t)120u;
    TEST_ASSERT_EQUAL_INT(7680, (int)realistic);
    TEST_ASSERT_TRUE(realistic <= (uint64_t)UINT32_MAX);
    /* 65536 * 65537 = 4 295 032 832 > UINT32_MAX: proves why uint64_t cast needed */
    const uint64_t extreme = (uint64_t)65536u * (uint64_t)65537u;
    TEST_ASSERT_TRUE(extreme > (uint64_t)UINT32_MAX);
}

/* ── Tests with real HAL constants (compiled only when non-zero) ───────────
 * HAL_ENC_TEST_REALISTIC: -DHAL_ENC_PULSES_PER_MOTOR_REV=64u
 *                          -DHAL_ENC_GEAR_RATIO=120u
 *                          -DHAL_ENC_WHEEL_CIRC_M=0.314f
 * HAL_ENC_TEST_OVERFLOW:  -DHAL_ENC_PULSES_PER_MOTOR_REV=65536u
 *                          -DHAL_ENC_GEAR_RATIO=65537u
 *                          -DHAL_ENC_WHEEL_CIRC_M=0.314f */

#ifdef HAL_ENC_TEST_REALISTIC

static void test_speed_one_full_output_revolution(void)
{
    /* count = CPR * gear → exactly 1 output rev/s at dt=1.0 s */
    reset();
    hal_encoder_init();
    hal_encoder_mock_set_count(
        (int32_t)HAL_ENC_PULSES_PER_MOTOR_REV * (int32_t)HAL_ENC_GEAR_RATIO);
    const float speed = hal_encoder_get_speed_kmh(1.0f);
    /* 1 rev/s × HAL_ENC_WHEEL_CIRC_M m × 3.6 = km/h */
    TEST_ASSERT_FLOAT_WITHIN(0.01f, HAL_ENC_WHEEL_CIRC_M * 3.6f, speed);
}

static void test_speed_half_revolution(void)
{
    reset();
    hal_encoder_init();
    hal_encoder_mock_set_count(
        (int32_t)HAL_ENC_PULSES_PER_MOTOR_REV * (int32_t)HAL_ENC_GEAR_RATIO / 2);
    const float speed = hal_encoder_get_speed_kmh(1.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, HAL_ENC_WHEEL_CIRC_M * 3.6f * 0.5f, speed);
}

#endif /* HAL_ENC_TEST_REALISTIC */

#ifdef HAL_ENC_TEST_OVERFLOW

static void test_speed_no_overflow_extreme_cpr(void)
{
    /* count = CPR value (fits in int32_t: 65536).
     * Correct   (uint64_t): 65536/(65536×65537)/1.0 × 0.314 × 3.6 ≈ 1.72e-5 km/h
     * Overflow (uint32_t): 65536/65536       /1.0 × 0.314 × 3.6 = 1.1304  km/h (1000× too high)
     * This test fails without the uint64_t cast in the speed formula. */
    reset();
    hal_encoder_init();
    hal_encoder_mock_set_count(HAL_ENC_PULSES_PER_MOTOR_REV); /* = 65536 */
    const float speed = hal_encoder_get_speed_kmh(1.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, speed); /* must be nearly 0, not ~1.1 km/h */
}

#endif /* HAL_ENC_TEST_OVERFLOW */

int main(void)
{
    TEST_SUITE_BEGIN("hal_encoder (mock)");
    RUN_TEST(test_init_clears_count);
    RUN_TEST(test_get_count_returns_mock_value);
    RUN_TEST(test_get_count_negative);
    RUN_TEST(test_reset_clears_count);
    RUN_TEST(test_get_count_before_init_returns_zero);
#if !defined(HAL_ENC_TEST_REALISTIC) && !defined(HAL_ENC_TEST_OVERFLOW)
    RUN_TEST(test_speed_returns_zero_when_pulses_per_rev_is_zero);
#endif
    RUN_TEST(test_speed_returns_zero_when_dt_is_zero);
    RUN_TEST(test_speed_returns_zero_when_dt_is_negative);
    RUN_TEST(test_speed_resets_count_after_reading);
    RUN_TEST(test_cpr_gear_product_no_overflow);
#ifdef HAL_ENC_TEST_REALISTIC
    RUN_TEST(test_speed_one_full_output_revolution);
    RUN_TEST(test_speed_half_revolution);
#endif
#ifdef HAL_ENC_TEST_OVERFLOW
    RUN_TEST(test_speed_no_overflow_extreme_cpr);
#endif
    TEST_SUITE_SUMMARY();
}
