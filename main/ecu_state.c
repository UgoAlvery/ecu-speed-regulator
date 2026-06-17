//
// Created by ugoal on 29/05/2026.
//

#include "ecu_state.h"

#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "ecu_state";

static ecu_state_t s_state;
static SemaphoreHandle_t s_mutex = NULL;

#define LOCK()                                                  \
    do {                                                        \
        if (s_mutex == NULL ||                                  \
            xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) { \
            ESP_LOGE(TAG, "mutex take failed");                 \
            return;                                             \
        }                                                       \
    } while (0)

#define LOCK_RET(ret_val)                                       \
    do {                                                        \
        if (s_mutex == NULL ||                                  \
            xSemaphoreTake(s_mutex, portMAX_DELAY) != pdTRUE) { \
            ESP_LOGE(TAG, "mutex take failed");                 \
            return (ret_val);                                   \
        }                                                       \
    } while (0)

#define UNLOCK() xSemaphoreGive(s_mutex)

void ecu_state_init(void) {
    s_mutex = xSemaphoreCreateMutex();
    if (s_mutex == NULL) {
        ESP_LOGE(TAG, "xSemaphoreCreateMutex failed - halting");
        for (;;) { vTaskDelay(portMAX_DELAY); }
    }
    s_state.mode         = ECU_MODE_OFF;
    s_state.setpoint     = 0.0f;
    s_state.speed        = 0.0f;
    s_state.output       = 0.0f;
    s_state.last_rx_tick = 0;

    ESP_LOGI(TAG, "initialized (mode=OFF)");
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Setters                                                                    */
/* ─────────────────────────────────────────────────────────────────────────── */

void ecu_state_set_mode(const ecu_mode_t mode)
{
    LOCK();
    s_state.mode = mode;
    UNLOCK();
}

void ecu_state_set_setpoint(const float setpoint)
{
    LOCK();
    s_state.setpoint = setpoint;
    UNLOCK();
}

void ecu_state_set_speed(const float speed)
{
    LOCK();
    s_state.speed = speed;
    UNLOCK();
}

void ecu_state_set_output(const float output)
{
    LOCK();
    s_state.output = output;
    UNLOCK();
}

void ecu_state_set_last_rx_tick(const TickType_t tick)
{
    LOCK();
    s_state.last_rx_tick = tick;
    UNLOCK();
}

/* ─────────────────────────────────────────────────────────────────────────── */
/*  Getters                                                                    */
/* ─────────────────────────────────────────────────────────────────────────── */

ecu_mode_t ecu_state_get_mode(void)
{
    LOCK_RET(ECU_MODE_OFF);
    const ecu_mode_t v = s_state.mode;
    UNLOCK();
    return v;
}

float ecu_state_get_setpoint(void)
{
    LOCK_RET(0.0f);
    const float v = s_state.setpoint;
    UNLOCK();
    return v;
}

float ecu_state_get_speed(void)
{
    LOCK_RET(0.0f);
    const float v = s_state.speed;
    UNLOCK();
    return v;
}

float ecu_state_get_output(void)
{
    LOCK_RET(0.0f);
    const float v = s_state.output;
    UNLOCK();
    return v;
}

TickType_t ecu_state_get_last_rx_tick(void)
{
    LOCK_RET(0);
    const TickType_t v = s_state.last_rx_tick;
    UNLOCK();
    return v;
}