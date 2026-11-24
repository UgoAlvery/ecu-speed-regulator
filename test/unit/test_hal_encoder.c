#include "test_runner.h"
#include "hal_encoder.h"
#include "hal_config.h"

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

static void test_speed_returns_zero_when_pulses_per_rev_is_zero(void)
{
    /* HAL_ENC_PULSES_PER_MOTOR_REV == 0 in hal_config.h (TODO(datasheet)). */
    reset();
    hal_encoder_init();
    hal_encoder_mock_set_count(1000);
    const float speed = hal_encoder_get_speed_kmh(0.1f);
    TEST_ASSERT_FLOAT_WITHIN(1e-6f, 0.0f, speed);
}

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

int main(void)
{
    TEST_SUITE_BEGIN("hal_encoder (mock)");
    RUN_TEST(test_init_clears_count);
    RUN_TEST(test_get_count_returns_mock_value);
    RUN_TEST(test_get_count_negative);
    RUN_TEST(test_reset_clears_count);
    RUN_TEST(test_get_count_before_init_returns_zero);
    RUN_TEST(test_speed_returns_zero_when_pulses_per_rev_is_zero);
    RUN_TEST(test_speed_returns_zero_when_dt_is_zero);
    RUN_TEST(test_speed_returns_zero_when_dt_is_negative);
    RUN_TEST(test_speed_resets_count_after_reading);
    TEST_SUITE_SUMMARY();
}
