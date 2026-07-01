#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output_min;
    float output_max;
    float dt;
    float integral_max;
} pid_controller_t;

void pid_init(pid_controller_t *pid, float kp, float ki, float kd, float dt);
void pid_set_limits(pid_controller_t *pid, float out_min, float out_max);
void pid_set_integral_limit(pid_controller_t *pid, float limit);
float pid_update(pid_controller_t *pid, float setpoint, float measurement);
void pid_reset(pid_controller_t *pid);
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

#endif
