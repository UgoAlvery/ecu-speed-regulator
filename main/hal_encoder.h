#ifndef HAL_ENCODER_H
#define HAL_ENCODER_H

/* Quadrature encoder HAL (FIT0522 + PCNT).
 *
 * Implementations:
 *   hal_encoder_esp32.c  — ESP32 PCNT driver
 *   hal_encoder_mock.c   — host mock for unit tests
 *
 * hal_encoder_get_speed_kmh() returns 0.0 when HAL_ENC_PULSES_PER_MOTOR_REV
 * or HAL_ENC_GEAR_RATIO is 0 (see hal_config.h TODO(datasheet) entries). */

#include <stdbool.h>
#include <stdint.h>

bool    hal_encoder_init(void);
/* Reset the internal pulse counter to 0. */
void    hal_encoder_reset(void);
/* Return the signed pulse count since the last reset (or init). */
int32_t hal_encoder_get_count(void);
/* Convert count since last call to km/h over the given time interval.
 * Internally calls hal_encoder_get_count() then hal_encoder_reset().
 * Returns 0.0 when any TODO(datasheet) constant is not yet filled. */
float   hal_encoder_get_speed_kmh(float dt_s);

#endif /* HAL_ENCODER_H */
