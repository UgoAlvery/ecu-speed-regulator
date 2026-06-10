#ifndef PID_H
#define PID_H

#pragma once

#include <stdint.h>

#define PID_DEFAULT_KP      2.0f
#define PID_DEFAULT_KI      0.8f
#define PID_DEFAULT_KD      0.02f
#define PID_DEFAULT_DT      0.1f
#define PID_DEFAULT_OUT_MIN 0.0f
#define PID_DEFAULT_OUT_MAX 255.0f

typedef struct {
    float kp;
    float ki;
    float kd;

    float dt;

    float out_min;
    float out_max;

    float integral;
    float prev_err;
} pid_t;

void pid_init(pid_t *pid,
              float kp, float ki, float kd,
              float dt,
              float out_min, float out_max);

void pid_init_default(pid_t *pid);
void pid_reset(pid_t *pid);
float pid_compute(pid_t *pid, float setpoint, float measure);

#endif