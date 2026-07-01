#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float duty;             // 0.0–1.0
    uint16_t pwm_period;    // timer ARR
    bool enabled;
} pump_state_t;

typedef struct {
    float duty;             // 0.0–1.0
    float max_duty;         // safety limit
    bool enabled;
} heater_state_t;

typedef struct {
    bool open;
} solenoid_state_t;

typedef struct {
    pump_state_t pump;
    heater_state_t heater;
    solenoid_state_t solenoid;
} actuator_state_t;

void actuators_init(actuator_state_t *act);
void pump_set_duty(actuator_state_t *act, float duty);
void pump_enable(actuator_state_t *act, bool enable);
void heater_set_duty(actuator_state_t *act, float duty);
void heater_enable(actuator_state_t *act, bool enable);
void solenoid_set(actuator_state_t *act, bool open);
void actuators_emergency_stop(actuator_state_t *act);
void actuators_apply(const actuator_state_t *act);

#endif
