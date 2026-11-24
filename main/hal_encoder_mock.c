#include "hal_encoder.h"
#include "hal_config.h"

#include <stdbool.h>
#include <stdint.h>

static bool    s_initialized = false;
static int32_t s_count       = 0;

/* ── Test hooks ────────────────────────────────────────────────────────────*/

void hal_encoder_mock_set_count(int32_t count)
{
    s_count = count;
}

void hal_encoder_mock_reset_state(void)
{
    s_initialized = false;
    s_count       = 0;
}

/* ── Interface ─────────────────────────────────────────────────────────────*/

bool hal_encoder_init(void)
{
    s_count       = 0;
    s_initialized = true;
    return true;
}

void hal_encoder_reset(void)
{
    s_count = 0;
}

int32_t hal_encoder_get_count(void)
{
    return s_initialized ? s_count : 0;
}

float hal_encoder_get_speed_kmh(float dt_s)
{
    /* shortcut: returns 0 until TODO(datasheet) constants are filled */
    if (HAL_ENC_PULSES_PER_MOTOR_REV == 0u) return 0.0f;
    if (HAL_ENC_GEAR_RATIO == 0u)           return 0.0f;
    if (HAL_ENC_WHEEL_CIRC_M <= 0.0f)       return 0.0f;
    if (dt_s <= 0.0f)                        return 0.0f;

    const int32_t count = hal_encoder_get_count();
    hal_encoder_reset();

    const float output_revs_per_s =
        (float)count
        / (float)(HAL_ENC_PULSES_PER_MOTOR_REV * HAL_ENC_GEAR_RATIO)
        / dt_s;
    return output_revs_per_s * HAL_ENC_WHEEL_CIRC_M * 3600.0f;
}
