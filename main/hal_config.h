#ifndef HAL_CONFIG_H
#define HAL_CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include <limits.h>

/* ── Safety guard ──────────────────────────────────────────────────────────
 * Set to true ONLY after physically installing the blade fuse (in series
 * with the motor supply) and the flyback diode (1N5822 across the motor).
 * Default false: hal_pwm_enable() refuses to arm the PWM until confirmed. */
#ifndef HW_PROTECTION_CONFIRMED
#define HW_PROTECTION_CONFIRMED false
#endif

/* ── Encoder parameters ────────────────────────────────────────────────────
 * TODO(datasheet): measure or look up FIT0522 encoder resolution and the
 * gearbox ratio printed on the motor label.
 * Speed conversion returns 0.0 km/h until both values are non-zero. */
#define HAL_ENC_PULSES_PER_MOTOR_REV  0u   /* TODO(datasheet) FIT0522 */
#define HAL_ENC_GEAR_RATIO            0u   /* TODO(datasheet) FIT0522 */

/* Wheel circumference in metres for km/h conversion.
 * TODO(datasheet): measure wheel + tyre fitted to the test bench. */
#define HAL_ENC_WHEEL_CIRC_M          0.0f /* TODO(datasheet) */

/* Guard: CPR * gear must fit in uint32_t (the formula uses uint64_t cast, but
 * float precision is lost above ~16M). Fires only when both are non-zero. */
#if HAL_ENC_PULSES_PER_MOTOR_REV != 0 && HAL_ENC_GEAR_RATIO != 0
_Static_assert(
    (uint64_t)HAL_ENC_PULSES_PER_MOTOR_REV * HAL_ENC_GEAR_RATIO <= UINT32_MAX,
    "HAL_ENC_PULSES_PER_MOTOR_REV * HAL_ENC_GEAR_RATIO overflows uint32_t"
);
#endif

/* ── Current sensor ────────────────────────────────────────────────────────
 * TODO(datasheet): set to the motor's rated stall current in mA.
 * Stall detection is disabled (always returns false) while this is 0. */
#define HAL_CURRENT_STALL_THRESHOLD_MA  0u /* TODO(datasheet) FIT0522 */

/* INA219 I2C address (A0=A1=GND on breakout → 0x40). */
#define HAL_INA219_I2C_ADDR  0x40u

/* INA219 shunt resistor value in milli-ohms on the breakout board.
 * TODO(datasheet): read the shunt resistor value printed on the INA219 PCB.
 * Common values: 100 mΩ (0.1 Ω) on most breakout boards. */
#define HAL_INA219_SHUNT_MOHM  0u /* TODO(datasheet) */

/* ── PWM ───────────────────────────────────────────────────────────────────
 * TODO(datasheet): confirm GT1168 (likely IRF520, non logic-level) switches
 * correctly at Vgs=3.3 V. Measure Vds under load; if Vds > 1 V at rated
 * current, a gate driver (e.g. IR2104) is required.
 * Max LEDC resolution at 20 kHz on 80 MHz APB: floor(log2(80e6/20e3)) = 12.
 * 10 bits is a conservative, safe choice. */
#define HAL_PWM_FREQ_HZ          20000u
#define HAL_PWM_RESOLUTION_BITS  10u

/* ── CAN / TWAI ────────────────────────────────────────────────────────────
 * Set true to use TWAI_MODE_NO_ACK (single-node bench test without a second
 * node to ACK frames). Must be false on the real two-node CAN bus. */
#ifndef HAL_CAN_NO_ACK_MODE
#define HAL_CAN_NO_ACK_MODE false
#endif

/* ── Control loop source selection ────────────────────────────────────────
 * false (default): speed input comes from UART SPEED frames (existing path).
 * true: speed input comes from the hardware encoder via hal_encoder. */
#ifndef ECU_USE_ENCODER_SPEED
#define ECU_USE_ENCODER_SPEED false
#endif

/* ── Hardware PWM output ───────────────────────────────────────────────────
 * false (default): PID output goes to UART OUTPUT frames only (existing path).
 * true: PID output also drives the LEDC PWM channel on MOTOR_PWM_PIN. */
#ifndef ECU_DRIVE_HW_PWM
#define ECU_DRIVE_HW_PWM false
#endif

#endif /* HAL_CONFIG_H */
