#ifndef ADAPTIVE_H
#define ADAPTIVE_H

#include "pid.h"
#include "kalman.h"

/*
 * Gain-scheduled PID that adjusts gains based on:
 *   1. Shot phase (pre-infusion, ramp-up, extraction, tail-off)
 *   2. Estimated puck resistance from the Kalman filter
 *
 * Also implements a Smith predictor wrapper to compensate the
 * transport delay between pump actuation and pressure response.
 */

typedef enum {
    SHOT_PHASE_PREINFUSE,
    SHOT_PHASE_RAMP,
    SHOT_PHASE_EXTRACT,
    SHOT_PHASE_DECLINE,
    SHOT_PHASE_COUNT
} shot_phase_t;

typedef struct {
    float kp;
    float ki;
    float kd;
} pid_gains_t;

typedef struct {
    pid_controller_t pid;
    kalman_filter_t *kf;
    shot_phase_t current_phase;
    float shot_time;
    pid_gains_t gain_table[SHOT_PHASE_COUNT];

    // Smith predictor state
    float delay_buffer[64];  // circular buffer for delayed model output
    uint16_t delay_head;
    uint16_t delay_samples;  // transport delay in samples
    float model_pressure;    // internal model predicted pressure

    // Resistance-based gain scaling
    float resistance_nominal;
    float gain_scale_factor;
} adaptive_controller_t;

void adaptive_init(adaptive_controller_t *ac, kalman_filter_t *kf, float dt);
void adaptive_set_delay(adaptive_controller_t *ac, float delay_seconds, float dt);
void adaptive_set_gain_table(adaptive_controller_t *ac, const pid_gains_t gains[SHOT_PHASE_COUNT]);
float adaptive_update(adaptive_controller_t *ac, float setpoint, float pressure, float flow, float pump_duty);
void adaptive_reset(adaptive_controller_t *ac);
shot_phase_t adaptive_get_phase(const adaptive_controller_t *ac);
void adaptive_set_ml_resistance(adaptive_controller_t *ac, float predicted_resistance);

#endif
