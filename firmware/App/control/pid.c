#include "pid.h"
#include <math.h>

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = (dt > 0.0f) ? dt : 0.001f; // V-04 fix: enforce positive dt
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output_min = 0.0f;
    pid->output_max = 1.0f;
    pid->integral_max = 100.0f;
}

void pid_set_limits(pid_controller_t *pid, float out_min, float out_max)
{
    // V-18 fix: validate ordering
    if (out_min >= out_max)
        return;
    pid->output_min = out_min;
    pid->output_max = out_max;
}

void pid_set_integral_limit(pid_controller_t *pid, float limit)
{
    if (limit <= 0.0f)
        return;
    pid->integral_max = limit;
}

float pid_update(pid_controller_t *pid, float setpoint, float measurement)
{
    // V-05 fix: reject NaN/Inf inputs to prevent propagation
    if (!isfinite(setpoint) || !isfinite(measurement))
        return 0.0f;

    float error = setpoint - measurement;

    pid->integral += error * pid->dt;
    if (pid->integral > pid->integral_max)
        pid->integral = pid->integral_max;
    else if (pid->integral < -pid->integral_max)
        pid->integral = -pid->integral_max;

    float derivative = (error - pid->prev_error) / pid->dt;
    pid->prev_error = error;

    float output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;

    // V-05 fix: NaN guard on output
    if (!isfinite(output))
        output = 0.0f;

    if (output > pid->output_max)
        output = pid->output_max;
    else if (output < pid->output_min)
        output = pid->output_min;

    return output;
}

void pid_reset(pid_controller_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}

void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd)
{
    // V-18 fix: reject non-finite gains
    if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd))
        return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}
