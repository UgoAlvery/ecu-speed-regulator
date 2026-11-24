#include "test_runner.h"
#include "hal_current.h"
#include "hal_config.h"

void hal_current_mock_set_current_ma(int32_t ma);
void hal_current_mock_set_read_error(bool error);
void hal_current_mock_reset_state(void);

static void reset(void)
{
    hal_current_mock_reset_state();
}

/* ── Tests ─────────────────────────────────────────────────────────────────*/

static void test_init_succeeds(void)
{
    reset();
    TEST_ASSERT_TRUE(hal_current_init());
}

static void test_read_before_init_returns_error(void)
{
    reset();  /* not initialized */
    TEST_ASSERT_EQUAL_INT(-1, hal_current_read_ma());
}

static void test_read_returns_set_value(void)
{
    reset();
    hal_current_init();
    hal_current_mock_set_current_ma(500);
    TEST_ASSERT_EQUAL_INT(500, hal_current_read_ma());
}

static void test_read_returns_zero(void)
{
    reset();
    hal_current_init();
    hal_current_mock_set_current_ma(0);
    TEST_ASSERT_EQUAL_INT(0, hal_current_read_ma());
}

static void test_read_error_flag_returns_minus_one(void)
{
    reset();
    hal_current_init();
    hal_current_mock_set_current_ma(300);
    hal_current_mock_set_read_error(true);
    TEST_ASSERT_EQUAL_INT(-1, hal_current_read_ma());
}

static void test_stall_disabled_when_threshold_zero(void)
{
    /* HAL_CURRENT_STALL_THRESHOLD_MA == 0 in hal_config.h (TODO(datasheet)).
     * Stall detection must be disabled regardless of current value. */
    reset();
    hal_current_init();
    hal_current_mock_set_current_ma(99999);
    TEST_ASSERT_FALSE(hal_current_is_stalled());
}

static void test_stall_not_triggered_on_read_error(void)
{
    /* A read error (I2C failure) must not be interpreted as a stall.
     * Fail-safe: don't cry stall when we can't read. */
    reset();
    hal_current_init();
    hal_current_mock_set_read_error(true);
    TEST_ASSERT_FALSE(hal_current_is_stalled());
}

int main(void)
{
    TEST_SUITE_BEGIN("hal_current (mock)");
    RUN_TEST(test_init_succeeds);
    RUN_TEST(test_read_before_init_returns_error);
    RUN_TEST(test_read_returns_set_value);
    RUN_TEST(test_read_returns_zero);
    RUN_TEST(test_read_error_flag_returns_minus_one);
    RUN_TEST(test_stall_disabled_when_threshold_zero);
    RUN_TEST(test_stall_not_triggered_on_read_error);
    TEST_SUITE_SUMMARY();
}
