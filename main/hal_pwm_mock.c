#include "hal_pwm.h"
#include "hal_config.h"

#include <stdbool.h>
#include <stdint.h>

static bool     s_initialized    = false;
static bool     s_enabled        = false;
static uint16_t s_duty           = 0;
/* Runtime override for HW_PROTECTION_CONFIRMED (test use only). */
static bool     s_hw_protection  = HW_PROTECTION_CONFIRMED;

/* ── Test hook ─────────────────────────────────────────────────────────────
 * Allows unit tests to exercise both sides of the safety guard without
 * recompiling. Not present in hal_pwm_esp32.c. */
void hal_pwm_mock_set_hw_protection(bool confirmed)
{
    s_hw_protection = confirmed;
}

void hal_pwm_mock_reset(void)
{
    s_initialized   = false;
    s_enabled       = false;
    s_duty          = 0;
    s_hw_protection = HW_PROTECTION_CONFIRMED;
}

/* ── Interface ─────────────────────────────────────────────────────────────*/

bool hal_pwm_init(void)
{
    s_duty        = 0;
    s_enabled     = false;
    s_initialized = true;
    return true;
}

bool hal_pwm_enable(void)
{
    if (!s_initialized)       return false;
    if (!s_hw_protection)     return false;  /* safety guard */
    s_enabled = true;
    return true;
}

void hal_pwm_disable(void)
{
    s_duty    = 0;
    s_enabled = false;
}

bool hal_pwm_set_duty_pct(uint16_t duty_pct_x100)
{
    if (!s_enabled) return false;
    if (duty_pct_x100 > 10000u) duty_pct_x100 = 10000u;
    s_duty = duty_pct_x100;
    return true;
}

bool     hal_pwm_is_enabled(void)   { return s_enabled; }
uint16_t hal_pwm_get_duty_pct(void) { return s_duty;    }
