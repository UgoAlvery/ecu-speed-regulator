#ifndef HAL_PWM_H
#define HAL_PWM_H

/* PWM HAL — abstract interface for motor gate drive (GT1168 / LEDC).
 *
 * Implementations:
 *   hal_pwm_esp32.c  — ESP32 LEDC driver (compiled by CMakeLists.txt)
 *   hal_pwm_mock.c   — host mock for unit tests (compiled by test/unit/Makefile)
 *
 * Safety contract:
 *   - hal_pwm_init() sets duty to 0% and leaves the channel DISABLED.
 *   - hal_pwm_enable() refuses to arm if HW_PROTECTION_CONFIRMED is false.
 *   - hal_pwm_disable() is always safe: forces duty to 0 and disarms.
 *   - hal_pwm_set_duty_pct() silently fails (returns false) when disabled.
 *
 * duty_pct_x100: 0 = 0.00%, 10000 = 100.00%.  Values above 10000 are clamped. */

#include <stdbool.h>
#include <stdint.h>

bool     hal_pwm_init(void);
bool     hal_pwm_enable(void);
void     hal_pwm_disable(void);
bool     hal_pwm_set_duty_pct(uint16_t duty_pct_x100);
bool     hal_pwm_is_enabled(void);
uint16_t hal_pwm_get_duty_pct(void);

#endif /* HAL_PWM_H */
