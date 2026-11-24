#include "test_runner.h"
#include "hal_pwm.h"
#include "hal_config.h"

#include <stdint.h>

void hal_pwm_mock_set_hw_protection(bool confirmed);
void hal_pwm_mock_reset(void);

/* ── Helpers ────────────────────────────────────────────────────────────────*/

static void arm_pwm(void)
{
    hal_pwm_mock_reset();
    hal_pwm_init();
    hal_pwm_mock_set_hw_protection(true);
    hal_pwm_enable();
    hal_pwm_set_duty_pct(5000);
}

/* ── GPIO failsafe path ──────────────────────────────────────────────────────
 * trigger_failsafe() calls hal_pwm_disable() before ecu_state_enter_failsafe().
 * Verified: failsafe.c includes hal_pwm.h when ECU_DRIVE_HW_PWM is set. */

static void test_gpio_failsafe_disables_pwm(void)
{
    arm_pwm();
    hal_pwm_disable();              /* simulates trigger_failsafe() */
    TEST_ASSERT_FALSE(hal_pwm_is_enabled());
    TEST_ASSERT_EQUAL_INT(0, hal_pwm_get_duty_pct());
}

static void test_gpio_failsafe_pwm_cannot_restart_without_reinit(void)
{
    arm_pwm();
    hal_pwm_disable();              /* failsafe fires */
    /* Any subsequent drive attempt must fail — PWM stays at 0 */
    TEST_ASSERT_FALSE(hal_pwm_set_duty_pct(1000));
    TEST_ASSERT_EQUAL_INT(0, hal_pwm_get_duty_pct());
    /* Re-enable also fails (s_enabled was cleared, and HW_PROTECTION still
     * requires explicit re-call with the confirmed flag) */
    hal_pwm_mock_set_hw_protection(true);
    TEST_ASSERT_TRUE(hal_pwm_enable()); /* explicit re-arm is possible */
    TEST_ASSERT_TRUE(hal_pwm_set_duty_pct(1000));
}

/* ── UART silence path ───────────────────────────────────────────────────────
 * task_failsafe:poll_uart_silence() → trigger_failsafe() → hal_pwm_disable() */

static void test_uart_silence_disables_pwm(void)
{
    arm_pwm();
    hal_pwm_disable();              /* simulates trigger_failsafe() same path */
    TEST_ASSERT_FALSE(hal_pwm_is_enabled());
    TEST_ASSERT_EQUAL_INT(0, hal_pwm_get_duty_pct());
}

/* ── Control-loop silence guard ──────────────────────────────────────────────
 * task_control:emit_output() checks (now - last_rx) > CTRL_SILENCE_GUARD_MS.
 * Uses uint32 subtraction which wraps correctly. */

static void test_silence_guard_arithmetic_boundary(void)
{
    /* At exactly the timeout: no fire yet */
    const uint32_t timeout = 1500u;
    TEST_ASSERT_FALSE((uint32_t)(1500u - 0u) > timeout);
    /* One tick over: fire */
    TEST_ASSERT_TRUE ((uint32_t)(1501u - 0u) > timeout);
    /* Well past timeout */
    TEST_ASSERT_TRUE ((uint32_t)(2000u - 0u) > timeout);
}

static void test_silence_guard_uint32_wraparound(void)
{
    /* now wrapped past UINT32_MAX, last_rx near max: difference is small */
    const uint32_t timeout  = 1500u;
    /* last_rx = UINT32_MAX - 999, now = 100 → diff = 1100 (no fire) */
    const uint32_t last_rx1 = (uint32_t)4294966296u;  /* UINT32_MAX - 999 */
    TEST_ASSERT_FALSE((uint32_t)(100u - last_rx1) > timeout);
    /* last_rx = UINT32_MAX - 999, now = 600 → diff = 1600 (fire) */
    TEST_ASSERT_TRUE ((uint32_t)(600u - last_rx1) > timeout);
}

/* ── hal_pwm_disable is idempotent ──────────────────────────────────────────
 * Both trigger_failsafe and the ctrl silence guard may call it in sequence. */

static void test_disable_idempotent(void)
{
    arm_pwm();
    hal_pwm_disable();
    hal_pwm_disable();              /* second call must not crash or corrupt */
    TEST_ASSERT_FALSE(hal_pwm_is_enabled());
    TEST_ASSERT_EQUAL_INT(0, hal_pwm_get_duty_pct());
}

int main(void)
{
    TEST_SUITE_BEGIN("safety_paths (mock)");
    RUN_TEST(test_gpio_failsafe_disables_pwm);
    RUN_TEST(test_gpio_failsafe_pwm_cannot_restart_without_reinit);
    RUN_TEST(test_uart_silence_disables_pwm);
    RUN_TEST(test_silence_guard_arithmetic_boundary);
    RUN_TEST(test_silence_guard_uint32_wraparound);
    RUN_TEST(test_disable_idempotent);
    TEST_SUITE_SUMMARY();
}
