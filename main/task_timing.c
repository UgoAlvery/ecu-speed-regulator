#include "task_timing.h"

#include <stdlib.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

#define TAG        "TIMING"
#define RING_SIZE  128u     /* ~12.8 s of 100 ms ticks; enough for a p99 estimate */

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

/* --- Loop jitter (deviation from 100 ms nominal, in µs, signed) --- */
static int64_t  s_prev_tick_us   = 0;
static int64_t  s_jitter_ring[RING_SIZE];
static uint32_t s_ring_head      = 0;   /* next write position */
static uint32_t s_ring_count     = 0;   /* samples in ring (saturates at RING_SIZE) */
static uint32_t s_total_ticks    = 0;   /* total ticks ever recorded */
static int64_t  s_jitter_min     = INT64_MAX;
static int64_t  s_jitter_max     = INT64_MIN;
static int64_t  s_jitter_sum     = 0;   /* sum over all ticks for running avg */

/* --- Failsafe SW latency (µs from trigger call to done call) --- */
static int64_t  s_fs_trigger_us  = 0;
static int64_t  s_fs_latency_us  = -1;  /* -1 = not yet measured */

/* --- Heartbeat: observed silence when timeout fires (µs) --- */
static long long s_hb_silence_us = -1;  /* -1 = not yet triggered */

static const int64_t NOMINAL_PERIOD_US = 100000LL;

/* ------------------------------------------------------------------ */

void timing_record_control_tick(void)
{
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_mux);
    if (s_prev_tick_us != 0) {
        const int64_t jitter = (now - s_prev_tick_us) - NOMINAL_PERIOD_US;
        s_jitter_ring[s_ring_head % RING_SIZE] = jitter;
        s_ring_head++;
        if (s_ring_count < RING_SIZE) s_ring_count++;
        s_total_ticks++;
        s_jitter_sum += jitter;
        if (jitter < s_jitter_min) s_jitter_min = jitter;
        if (jitter > s_jitter_max) s_jitter_max = jitter;
    }
    s_prev_tick_us = now;
    taskEXIT_CRITICAL(&s_mux);
}

void timing_record_failsafe_trigger(void)
{
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_mux);
    s_fs_trigger_us = now;
    taskEXIT_CRITICAL(&s_mux);
}

void timing_record_failsafe_done(void)
{
    const int64_t now = esp_timer_get_time();
    taskENTER_CRITICAL(&s_mux);
    if (s_fs_trigger_us > 0) {
        s_fs_latency_us = now - s_fs_trigger_us;
    }
    taskEXIT_CRITICAL(&s_mux);
}

void timing_record_heartbeat_silence_us(const long long silence_us)
{
    taskENTER_CRITICAL(&s_mux);
    s_hb_silence_us = silence_us;
    taskEXIT_CRITICAL(&s_mux);
}

static int cmp_i64(const void *a, const void *b)
{
    const int64_t x = *(const int64_t *)a;
    const int64_t y = *(const int64_t *)b;
    return (x > y) - (x < y);
}

void timing_print_csv(void)
{
    int64_t  ring_copy[RING_SIZE];
    uint32_t count, total;
    int64_t  jmin, jmax, jsum, fs_lat;
    long long hb_sil;

    taskENTER_CRITICAL(&s_mux);
    count  = s_ring_count;
    total  = s_total_ticks;
    memcpy(ring_copy, s_jitter_ring, count * sizeof(int64_t));
    jmin   = s_jitter_min;
    jmax   = s_jitter_max;
    jsum   = s_jitter_sum;
    fs_lat = s_fs_latency_us;
    hb_sil = s_hb_silence_us;
    taskEXIT_CRITICAL(&s_mux);

    if (count < 2u) {
        ESP_LOGI(TAG, "CSV,no_data_yet");
        return;
    }

    qsort(ring_copy, count, sizeof(int64_t), cmp_i64);
    const int64_t  javg = jsum / (int64_t)total;
    const int64_t  jp99 = ring_copy[(uint32_t)(count * 99u / 100u)];

    /* One line per metric: "CSV,<name>,<value_us>" — grep-friendly on monitor */
    ESP_LOGI(TAG, "CSV,loop_jitter_min_us,%lld",  (long long)jmin);
    ESP_LOGI(TAG, "CSV,loop_jitter_avg_us,%lld",  (long long)javg);
    ESP_LOGI(TAG, "CSV,loop_jitter_p99_us,%lld",  (long long)jp99);
    ESP_LOGI(TAG, "CSV,loop_jitter_max_us,%lld",  (long long)jmax);
    ESP_LOGI(TAG, "CSV,loop_samples,%lu",         (unsigned long)total);

    if (fs_lat >= 0) {
        ESP_LOGI(TAG, "CSV,failsafe_sw_latency_us,%lld", (long long)fs_lat);
    } else {
        ESP_LOGI(TAG, "CSV,failsafe_sw_latency_us,NOT_TRIGGERED");
    }

    if (hb_sil >= 0) {
        ESP_LOGI(TAG, "CSV,heartbeat_silence_at_timeout_us,%lld", (long long)hb_sil);
    } else {
        ESP_LOGI(TAG, "CSV,heartbeat_silence_at_timeout_us,NOT_TRIGGERED");
    }
}
