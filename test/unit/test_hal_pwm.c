#include "test_runner.h"
#include "hal_pwm.h"
#include "hal_config.h"

/* hal_pwm_mock_* test hooks declared here (not in hal_pwm.h to keep the
 * interface clean; the mock .c file is included directly by the Makefile). */
void hal_pwm_mock_set_hw_protection(bool confirmed);
void hal_pwm_mock_reset(void);

/* ── Helpers ───────────────────────────────────────────────────────────────*/

static void reset(void)
{
    hal_pwm_mock_reset();
}

/* ── Tests ─────────────────────────────────────────────────────────────────*/

static void test_init_starts_safe(void)
{
    reset();
    TEST_ASSERT_TRUE(hal_pwm_init());
    TEST_ASSERT_FALSE(hal_pwm_is_enabled());
    TEST_ASSERT_EQUAL_INT(0, hal_pwm_get_duty_pct());
}

static void test_enable_blocked_without_hw_protection(void)
{
    /* HW_PROTECTION_CONFIRMED defaults to false in hal_config.h. */
    reset();
    hal_pwm_init();
    hal_pwm_mock_set_hw_protection(false);
    TEST_ASSERT_FALSE(hal_pwm_enable());
    TEST_ASSERT_FALSE(hal_pwm_is_enabled());
}

static void test_set_duty_fails_when_disabled(void)
{
    reset();
    hal_pwm_init();
    TEST_ASSERT_FALSE(hal_pwm_set_duty_pct(5000));
    TEST_ASSERT_EQUAL_INT(0, hal_pwm_get_duty_pct());
}

static void test_enable_succeeds_with_hw_protection(void)
{
    reset();
    hal_pwm_init();
    hal_pwm_mock_set_hw_protection(true);
    TEST_ASSERT_TRUE(hal_pwm_enable());
    TEST_ASSERT_TRUE(hal_pwm_is_enabled());
}

static void test_set_duty_works_when_enabled(void)
{
    reset();
    hal_pwm_init();
    hal_pwm_mock_set_hw_protection(true);
    hal_pwm_enable();
    TEST_ASSERT_TRUE(hal_pwm_set_duty_pct(5000));
    TEST_ASSERT_EQUAL_INT(5000, hal_pwm_get_duty_pct());
}

static void test_duty_clamped_to_10000(void)
{
    reset();
    hal_pwm_init();
    hal_pwm_mock_set_hw_protection(true);
    hal_pwm_enable();
    TEST_ASSERT_TRUE(hal_pwm_set_duty_pct(20000));
    TEST_ASSERT_EQUAL_INT(10000, hal_pwm_get_duty_pct());
}

static void test_disable_clears_duty_and_disarms(void)
{
    reset();
    hal_pwm_init();
    hal_pwm_mock_set_hw_protection(true);
    hal_pwm_enable();
    hal_pwm_set_duty_pct(7000);
    hal_pwm_disable();
    TEST_ASSERT_FALSE(hal_pwm_is_enabled());
    TEST_ASSERT_EQUAL_INT(0, hal_pwm_get_duty_pct());
}

static void test_set_duty_fails_after_disable(void)
{
    reset();
    hal_pwm_init();
    hal_pwm_mock_set_hw_protection(true);
    hal_pwm_enable();
    hal_pwm_disable();
    TEST_ASSERT_FALSE(hal_pwm_set_duty_pct(3000));
    TEST_ASSERT_EQUAL_INT(0, hal_pwm_get_duty_pct());
}

static void test_disable_simulates_failsafe_path(void)
{
    /* Verify the failsafe call sequence: enable, run motor, trigger failsafe
     * (hal_pwm_disable), confirm PWM is 0 and cannot be re-driven without
     * explicit re-enable. */
    reset();
    hal_pwm_init();
    hal_pwm_mock_set_hw_protection(true);
    hal_pwm_enable();
    hal_pwm_set_duty_pct(8000);
    /* Failsafe triggers: */
    hal_pwm_disable();
    /* Motor must be stopped: */
    TEST_ASSERT_EQUAL_INT(0, hal_pwm_get_duty_pct());
    TEST_ASSERT_FALSE(hal_pwm_is_enabled());
    /* Any subsequent drive attempt must fail: */
    TEST_ASSERT_FALSE(hal_pwm_set_duty_pct(1));
    TEST_ASSERT_EQUAL_INT(0, hal_pwm_get_duty_pct());
}

static void test_zero_duty_allowed_when_enabled(void)
{
    reset();
    hal_pwm_init();
    hal_pwm_mock_set_hw_protection(true);
    hal_pwm_enable();
    hal_pwm_set_duty_pct(5000);
    TEST_ASSERT_TRUE(hal_pwm_set_duty_pct(0));
    TEST_ASSERT_EQUAL_INT(0, hal_pwm_get_duty_pct());
    TEST_ASSERT_TRUE(hal_pwm_is_enabled()); /* enabled, just 0% duty */
}

static void test_enable_requires_init_first(void)
{
    /* Calling enable without init should fail. */
    hal_pwm_mock_reset();  /* clears s_initialized */
    hal_pwm_mock_set_hw_protection(true);
    TEST_ASSERT_FALSE(hal_pwm_enable());
}

int main(void)
{
    TEST_SUITE_BEGIN("hal_pwm (mock)");
    RUN_TEST(test_init_starts_safe);
    RUN_TEST(test_enable_blocked_without_hw_protection);
    RUN_TEST(test_set_duty_fails_when_disabled);
    RUN_TEST(test_enable_succeeds_with_hw_protection);
    RUN_TEST(test_set_duty_works_when_enabled);
    RUN_TEST(test_duty_clamped_to_10000);
    RUN_TEST(test_disable_clears_duty_and_disarms);
    RUN_TEST(test_set_duty_fails_after_disable);
    RUN_TEST(test_disable_simulates_failsafe_path);
    RUN_TEST(test_zero_duty_allowed_when_enabled);
    RUN_TEST(test_enable_requires_init_first);
    TEST_SUITE_SUMMARY();
}
