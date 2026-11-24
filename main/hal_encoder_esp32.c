#include "hal_encoder.h"
#include "hal_config.h"
#include "board_pins.h"

#include <stdint.h>
#include <math.h>

#include "driver/pulse_cnt.h"
#include "esp_log.h"

static const char *TAG = "ENC";

static pcnt_unit_handle_t s_unit  = NULL;
static pcnt_chan_handle_t  s_chan0 = NULL;
static pcnt_chan_handle_t  s_chan1 = NULL;
static bool s_initialized = false;

bool hal_encoder_init(void)
{
    if (s_initialized) return true;

    pcnt_unit_config_t unit_cfg = {
        .high_limit = INT16_MAX,
        .low_limit  = INT16_MIN,
    };
    if (pcnt_new_unit(&unit_cfg, &s_unit) != ESP_OK) {
        ESP_LOGE(TAG, "pcnt_new_unit failed");
        return false;
    }

    /* Channel 0: edge on A, level on B */
    const pcnt_chan_config_t ch0 = {
        .edge_gpio_num  = ENCODER_A_PIN,
        .level_gpio_num = ENCODER_B_PIN,
    };
    if (pcnt_new_channel(s_unit, &ch0, &s_chan0) != ESP_OK) return false;
    pcnt_channel_set_edge_action(s_chan0,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE);
    pcnt_channel_set_level_action(s_chan0,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    /* Channel 1: edge on B, level on A — gives 4× quadrature resolution */
    const pcnt_chan_config_t ch1 = {
        .edge_gpio_num  = ENCODER_B_PIN,
        .level_gpio_num = ENCODER_A_PIN,
    };
    if (pcnt_new_channel(s_unit, &ch1, &s_chan1) != ESP_OK) return false;
    pcnt_channel_set_edge_action(s_chan1,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE);
    pcnt_channel_set_level_action(s_chan1,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE);

    if (pcnt_unit_enable(s_unit)      != ESP_OK) return false;
    if (pcnt_unit_clear_count(s_unit) != ESP_OK) return false;
    if (pcnt_unit_start(s_unit)       != ESP_OK) return false;

    s_initialized = true;
    ESP_LOGI(TAG, "PCNT encoder init OK (A=GPIO%d B=GPIO%d)",
             ENCODER_A_PIN, ENCODER_B_PIN);
    return true;
}

void hal_encoder_reset(void)
{
    if (!s_initialized) return;
    pcnt_unit_clear_count(s_unit);
}

int32_t hal_encoder_get_count(void)
{
    if (!s_initialized) return 0;
    int count = 0;
    pcnt_unit_get_count(s_unit, &count);
    return (int32_t)count;
}

float hal_encoder_get_speed_kmh(const float dt_s)
{
    if (!s_initialized || dt_s <= 0.0f) return 0.0f;

    /* shortcut: returns 0 until TODO(datasheet) constants are filled */
    if (HAL_ENC_PULSES_PER_MOTOR_REV == 0u || HAL_ENC_GEAR_RATIO == 0u) {
        hal_encoder_reset();
        return 0.0f;
    }

    int raw = 0;
    pcnt_unit_get_count(s_unit, &raw);
    hal_encoder_reset();

    const float output_revs_per_s =
        (float)raw /
        (float)(HAL_ENC_PULSES_PER_MOTOR_REV * HAL_ENC_GEAR_RATIO) /
        dt_s;
    return output_revs_per_s * HAL_ENC_WHEEL_CIRC_M * 3600.0f;
}
