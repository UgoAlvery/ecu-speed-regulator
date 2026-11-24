#include "hal_pwm.h"
#include "hal_config.h"
#include "board_pins.h"

#include "driver/ledc.h"
#include "esp_log.h"

#define LEDC_SPEED_MODE  LEDC_LOW_SPEED_MODE
#define LEDC_TIMER_IDX   LEDC_TIMER_0
#define LEDC_CHANNEL_IDX LEDC_CHANNEL_0
#define DUTY_MAX_TICKS   ((1u << HAL_PWM_RESOLUTION_BITS) - 1u)

static const char *TAG = "PWM";

static bool     s_initialized = false;
static bool     s_enabled     = false;
static uint16_t s_duty_pct    = 0;

static void hw_set_duty(const uint16_t duty_pct_x100)
{
    const uint32_t ticks =
        ((uint32_t)duty_pct_x100 * DUTY_MAX_TICKS) / 10000u;
    ledc_set_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_IDX, ticks);
    ledc_update_duty(LEDC_SPEED_MODE, LEDC_CHANNEL_IDX);
}

bool hal_pwm_init(void)
{
    if (s_initialized) return true;

    s_enabled  = false;
    s_duty_pct = 0;

    const ledc_timer_config_t timer_cfg = {
        .speed_mode      = LEDC_SPEED_MODE,
        .duty_resolution = (ledc_timer_bit_t)HAL_PWM_RESOLUTION_BITS,
        .timer_num       = LEDC_TIMER_IDX,
        .freq_hz         = HAL_PWM_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    if (ledc_timer_config(&timer_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "ledc_timer_config failed");
        return false;
    }

    const ledc_channel_config_t channel_cfg = {
        .speed_mode = LEDC_SPEED_MODE,
        .channel    = LEDC_CHANNEL_IDX,
        .timer_sel  = LEDC_TIMER_IDX,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = MOTOR_PWM_PIN,
        .duty       = 0,
        .hpoint     = 0,
    };
    if (ledc_channel_config(&channel_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "ledc_channel_config failed");
        return false;
    }

    s_initialized = true;
    ESP_LOGI(TAG, "LEDC init OK (GPIO%d, %uHz, %u-bit)",
             MOTOR_PWM_PIN, HAL_PWM_FREQ_HZ, HAL_PWM_RESOLUTION_BITS);
    return true;
}

bool hal_pwm_enable(void)
{
    if (!s_initialized) return false;
    if (!HW_PROTECTION_CONFIRMED) {
        ESP_LOGE(TAG, "hal_pwm_enable refused: HW_PROTECTION_CONFIRMED is false");
        return false;
    }
    s_enabled = true;
    return true;
}

void hal_pwm_disable(void)
{
    s_enabled  = false;
    s_duty_pct = 0;
    if (s_initialized) {
        hw_set_duty(0);
    }
}

bool hal_pwm_set_duty_pct(uint16_t duty_pct_x100)
{
    if (!s_initialized || !s_enabled) return false;
    if (duty_pct_x100 > 10000u) duty_pct_x100 = 10000u;
    s_duty_pct = duty_pct_x100;
    hw_set_duty(duty_pct_x100);
    return true;
}

bool hal_pwm_is_enabled(void)
{
    return s_enabled;
}

uint16_t hal_pwm_get_duty_pct(void)
{
    return s_duty_pct;
}
