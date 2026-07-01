#ifndef CONTROLLER_INTERFACE_H
#define CONTROLLER_INTERFACE_H

#include <stdint.h>

/*
 * Uniform interface for all controller variants.
 * Allows build-time or runtime switching between:
 *   - Baseline PID
 *   - Adaptive (Kalman + gain-scheduled PID + Smith predictor)
 *   - Adaptive + ML
 *   - Simulink Coder-generated controller
 */

typedef struct {
    float pressure_bar;
    float flow_ml_s;
    float temperature_c;
    float setpoint_pressure;
    float pump_duty_prev;
    uint32_t timestamp_ms;
} controller_input_t;

typedef struct {
    float pump_duty;        // 0.0–1.0
    float heater_duty;      // 0.0–1.0
    uint8_t phase;          // shot phase enum
    float est_resistance;   // Kalman-estimated puck resistance
} controller_output_t;

typedef void (*controller_init_fn)(void);
typedef void (*controller_step_fn)(const controller_input_t *in, controller_output_t *out);
typedef void (*controller_reset_fn)(void);

typedef struct {
    const char *name;
    controller_init_fn init;
    controller_step_fn step;
    controller_reset_fn reset;
} controller_vtable_t;

#endif
