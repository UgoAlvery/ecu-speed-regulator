
#include "pid.h"

#include <stddef.h>
#include <assert.h>

/* -------------------------------------------------------------------------- */
/* API publique                                                                */
/* -------------------------------------------------------------------------- */

void pid_init(pid_t *pid,
              const float kp, const float ki, const float kd,
              const float dt,
              const float out_min, const float out_max)
{
    if (pid == NULL) return;

    pid->kp      = kp;
    pid->ki      = ki;
    pid->kd      = kd;
    pid->dt      = (dt > 0.0f) ? dt : PID_DEFAULT_DT;
    pid->out_min = out_min;
    pid->out_max = out_max;

    pid->integral = 0.0f;
    pid->prev_err = 0.0f;
}

void pid_init_default(pid_t *pid)
{
    pid_init(pid,
             PID_DEFAULT_KP,
             PID_DEFAULT_KI,
             PID_DEFAULT_KD,
             PID_DEFAULT_DT,
             PID_DEFAULT_OUT_MIN,
             PID_DEFAULT_OUT_MAX);
}

void pid_reset(pid_t *pid)
{
    if (pid == NULL) return;

    pid->integral = 0.0f;
    pid->prev_err = 0.0f;
}

float pid_compute(pid_t *pid, const float setpoint, const float measure)
{
    if (pid == NULL) return 0.0f;

    const float err = setpoint - measure;

    float integral_candidate = pid->integral + err * pid->dt;

    const float derivative = (err - pid->prev_err) / pid->dt;

    float output = (pid->kp * err)
                 + (pid->ki * integral_candidate)
                 + (pid->kd * derivative);


    if (output > pid->out_max) {
        output = pid->out_max;
        if (err > 0.0f) {
            integral_candidate = pid->integral;
        }
    } else if (output < pid->out_min) {
        output = pid->out_min;
        if (err < 0.0f) {
            integral_candidate = pid->integral;
        }
    }

    pid->integral = integral_candidate;
    pid->prev_err = err;

    return output;
}