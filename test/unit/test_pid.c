#include "test_runner.h"
#include "pid.h"

/* ========================================================================== */
/* pid_init                                                                   */
/* ========================================================================== */

static void test_pid_init_null_no_crash(void)
{
    pid_init(NULL, 1.0f, 1.0f, 1.0f, 0.1f, 0.0f, 255.0f);
    TEST_ASSERT_TRUE(1);  /* pas de crash = succès */
}

static void test_pid_init_fields_set(void)
{
    pid_t pid;
    pid_init(&pid, 1.5f, 2.5f, 0.5f, 0.2f, -10.0f, 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 1.5f,  pid.kp);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 2.5f,  pid.ki);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.5f,  pid.kd);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.2f,  pid.dt);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, -10.0f, pid.out_min);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 10.0f,  pid.out_max);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f,  pid.integral);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f,  pid.prev_err);
}

static void test_pid_init_zero_dt_uses_default(void)
{
    pid_t pid;
    pid_init(&pid, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 255.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, PID_DEFAULT_DT, pid.dt);
}

static void test_pid_init_negative_dt_uses_default(void)
{
    pid_t pid;
    pid_init(&pid, 1.0f, 0.0f, 0.0f, -1.0f, 0.0f, 255.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, PID_DEFAULT_DT, pid.dt);
}

static void test_pid_init_default_uses_constants(void)
{
    pid_t pid;
    pid_init_default(&pid);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, PID_DEFAULT_KP,      pid.kp);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, PID_DEFAULT_KI,      pid.ki);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, PID_DEFAULT_KD,      pid.kd);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, PID_DEFAULT_DT,      pid.dt);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, PID_DEFAULT_OUT_MIN, pid.out_min);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, PID_DEFAULT_OUT_MAX, pid.out_max);
}

/* ========================================================================== */
/* pid_reset                                                                  */
/* ========================================================================== */

static void test_pid_reset_null_no_crash(void)
{
    pid_reset(NULL);
    TEST_ASSERT_TRUE(1);
}

static void test_pid_reset_clears_state(void)
{
    pid_t pid;
    pid_init_default(&pid);
    pid_compute(&pid, 50.0f, 0.0f);  /* accumule integral et prev_err */
    pid_reset(&pid);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, pid.integral);
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, pid.prev_err);
}

static void test_pid_reset_restores_first_step_behavior(void)
{
    pid_t pid;
    /* ki uniquement pour observer l'intégrale */
    pid_init(&pid, 0.0f, 1.0f, 0.0f, 0.1f, -1000.0f, 1000.0f);
    pid_compute(&pid, 10.0f, 0.0f);  /* integral = 1.0 après ce step */
    pid_compute(&pid, 10.0f, 0.0f);  /* integral = 2.0 */
    pid_reset(&pid);
    /* après reset, premier step donne la même sortie qu'au tout début */
    float out = pid_compute(&pid, 10.0f, 0.0f);
    /* integral_candidate = 0 + 10*0.1 = 1.0 ; output = 1.0*1.0 = 1.0 */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, out);
}

/* ========================================================================== */
/* pid_compute — gardes et cas triviaux                                       */
/* ========================================================================== */

static void test_pid_compute_null_returns_zero(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-5f, 0.0f, pid_compute(NULL, 50.0f, 30.0f));
}

static void test_pid_compute_zero_error(void)
{
    pid_t pid;
    pid_init_default(&pid);
    /* setpoint == measure → erreur nulle, sortie nulle */
    float out = pid_compute(&pid, 50.0f, 50.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out);
}

/* ========================================================================== */
/* pid_compute — termes proportionnel, intégral, dérivé isolés               */
/* ========================================================================== */

static void test_pid_compute_proportional_only(void)
{
    pid_t pid;
    /* ki=kd=0 : sortie = kp * erreur */
    pid_init(&pid, 2.0f, 0.0f, 0.0f, 0.1f, -1000.0f, 1000.0f);
    float out = pid_compute(&pid, 50.0f, 30.0f);  /* err = 20 */
    /* output = 2.0 * 20 = 40.0 */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 40.0f, out);
}

static void test_pid_compute_integral_accumulates_step1(void)
{
    pid_t pid;
    pid_init(&pid, 0.0f, 1.0f, 0.0f, 0.1f, -1000.0f, 1000.0f);
    float out = pid_compute(&pid, 10.0f, 0.0f);  /* err=10 */
    /* integral_candidate = 0 + 10*0.1 = 1.0 ; output = 1.0*1.0 = 1.0 */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, out);
}

static void test_pid_compute_integral_accumulates_step2(void)
{
    pid_t pid;
    pid_init(&pid, 0.0f, 1.0f, 0.0f, 0.1f, -1000.0f, 1000.0f);
    pid_compute(&pid, 10.0f, 0.0f);               /* step 1 : integral = 1.0 */
    float out = pid_compute(&pid, 10.0f, 0.0f);   /* step 2 */
    /* integral_candidate = 1.0 + 10*0.1 = 2.0 ; output = 2.0 */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 2.0f, out);
}

static void test_pid_compute_derivative_first_step(void)
{
    pid_t pid;
    /* kp=ki=0 : sortie = kd * (err - prev_err) / dt */
    pid_init(&pid, 0.0f, 0.0f, 1.0f, 0.1f, -1000.0f, 1000.0f);
    float out = pid_compute(&pid, 10.0f, 0.0f);  /* err=10, prev_err=0 */
    /* derivative = (10 - 0) / 0.1 = 100 ; output = 1.0 * 100 = 100.0 */
    TEST_ASSERT_FLOAT_WITHIN(1e-3f, 100.0f, out);
}

static void test_pid_compute_derivative_steady_state(void)
{
    pid_t pid;
    pid_init(&pid, 0.0f, 0.0f, 1.0f, 0.1f, -1000.0f, 1000.0f);
    pid_compute(&pid, 10.0f, 0.0f);              /* step 1 : prev_err = 10 */
    float out = pid_compute(&pid, 10.0f, 0.0f); /* step 2 : err=10, prev_err=10 */
    /* derivative = (10 - 10) / 0.1 = 0 ; output = 0 */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, out);
}

/* ========================================================================== */
/* pid_compute — saturation de la sortie                                      */
/* ========================================================================== */

static void test_pid_compute_clamps_to_max(void)
{
    pid_t pid;
    pid_init(&pid, 100.0f, 0.0f, 0.0f, 0.1f, 0.0f, 50.0f);
    float out = pid_compute(&pid, 10.0f, 0.0f);  /* output = 1000 → clamped */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 50.0f, out);
}

static void test_pid_compute_clamps_to_min(void)
{
    pid_t pid;
    pid_init(&pid, 100.0f, 0.0f, 0.0f, 0.1f, -50.0f, 255.0f);
    float out = pid_compute(&pid, 0.0f, 10.0f);  /* err=-10, output=-1000 → clamped */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -50.0f, out);
}

/* ========================================================================== */
/* pid_compute — anti-windup                                                  */
/* ========================================================================== */

static void test_pid_anti_windup_upper_freezes_integral(void)
{
    pid_t pid;
    /* kp élevé pour saturer immédiatement en sortie positive */
    pid_init(&pid, 10.0f, 1.0f, 0.0f, 0.1f, 0.0f, 20.0f);
    pid_compute(&pid, 10.0f, 0.0f);
    /* err=10 > 0 et output sature → intégrale gelée à sa valeur initiale (0) */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, pid.integral);
}

static void test_pid_anti_windup_upper_output_clamped(void)
{
    pid_t pid;
    pid_init(&pid, 10.0f, 1.0f, 0.0f, 0.1f, 0.0f, 20.0f);
    float out = pid_compute(&pid, 10.0f, 0.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 20.0f, out);
}

static void test_pid_anti_windup_lower_freezes_integral(void)
{
    pid_t pid;
    /* kp élevé pour saturer immédiatement en sortie négative */
    pid_init(&pid, 10.0f, 1.0f, 0.0f, 0.1f, -20.0f, 255.0f);
    pid_compute(&pid, 0.0f, 10.0f);
    /* err=-10 < 0 et output sature → intégrale gelée à 0 */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, pid.integral);
}

static void test_pid_anti_windup_lower_output_clamped(void)
{
    pid_t pid;
    pid_init(&pid, 10.0f, 1.0f, 0.0f, 0.1f, -20.0f, 255.0f);
    float out = pid_compute(&pid, 0.0f, 10.0f);
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, -20.0f, out);
}

static void test_pid_anti_windup_no_freeze_when_unsaturated(void)
{
    pid_t pid;
    /* out_max assez grand pour ne pas saturer */
    pid_init(&pid, 1.0f, 1.0f, 0.0f, 0.1f, -1000.0f, 1000.0f);
    pid_compute(&pid, 10.0f, 0.0f);
    /* integral doit avoir évolué normalement : 0 + 10*0.1 = 1.0 */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 1.0f, pid.integral);
}

/* ========================================================================== */
/* main                                                                       */
/* ========================================================================== */

int main(void)
{
    TEST_SUITE_BEGIN("pid");

    RUN_TEST(test_pid_init_null_no_crash);
    RUN_TEST(test_pid_init_fields_set);
    RUN_TEST(test_pid_init_zero_dt_uses_default);
    RUN_TEST(test_pid_init_negative_dt_uses_default);
    RUN_TEST(test_pid_init_default_uses_constants);

    RUN_TEST(test_pid_reset_null_no_crash);
    RUN_TEST(test_pid_reset_clears_state);
    RUN_TEST(test_pid_reset_restores_first_step_behavior);

    RUN_TEST(test_pid_compute_null_returns_zero);
    RUN_TEST(test_pid_compute_zero_error);
    RUN_TEST(test_pid_compute_proportional_only);
    RUN_TEST(test_pid_compute_integral_accumulates_step1);
    RUN_TEST(test_pid_compute_integral_accumulates_step2);
    RUN_TEST(test_pid_compute_derivative_first_step);
    RUN_TEST(test_pid_compute_derivative_steady_state);
    RUN_TEST(test_pid_compute_clamps_to_max);
    RUN_TEST(test_pid_compute_clamps_to_min);
    RUN_TEST(test_pid_anti_windup_upper_freezes_integral);
    RUN_TEST(test_pid_anti_windup_upper_output_clamped);
    RUN_TEST(test_pid_anti_windup_lower_freezes_integral);
    RUN_TEST(test_pid_anti_windup_lower_output_clamped);
    RUN_TEST(test_pid_anti_windup_no_freeze_when_unsaturated);

    TEST_SUITE_SUMMARY();
}