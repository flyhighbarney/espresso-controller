#include "adaptive.h"
#include <string.h>
#include <math.h>

static const pid_gains_t DEFAULT_GAINS[SHOT_PHASE_COUNT] = {
    { .kp = 0.8f, .ki = 0.3f, .kd = 0.05f },  // PREINFUSE: gentle
    { .kp = 1.5f, .ki = 0.8f, .kd = 0.10f },  // RAMP: aggressive
    { .kp = 1.2f, .ki = 0.6f, .kd = 0.08f },  // EXTRACT: steady
    { .kp = 0.6f, .ki = 0.2f, .kd = 0.03f },  // DECLINE: backing off
};

static shot_phase_t detect_phase(float shot_time, float pressure)
{
    if (shot_time < 3.0f)
        return SHOT_PHASE_PREINFUSE;
    if (pressure < 7.0f && shot_time < 10.0f)
        return SHOT_PHASE_RAMP;
    if (shot_time < 25.0f)
        return SHOT_PHASE_EXTRACT;
    return SHOT_PHASE_DECLINE;
}

void adaptive_init(adaptive_controller_t *ac, kalman_filter_t *kf, float dt)
{
    memset(ac, 0, sizeof(adaptive_controller_t));
    ac->kf = kf;
    pid_init(&ac->pid, DEFAULT_GAINS[0].kp, DEFAULT_GAINS[0].ki, DEFAULT_GAINS[0].kd, dt);
    pid_set_limits(&ac->pid, 0.0f, 1.0f);
    pid_set_integral_limit(&ac->pid, 50.0f);

    memcpy(ac->gain_table, DEFAULT_GAINS, sizeof(DEFAULT_GAINS));

    ac->resistance_nominal = 10.0f;
    ac->gain_scale_factor = 1.0f;
    ac->delay_samples = 0;
    ac->delay_head = 0;
    ac->model_pressure = 0.0f;
}

void adaptive_set_delay(adaptive_controller_t *ac, float delay_seconds, float dt)
{
    // V-09 fix: clamp float before casting to prevent UB
    if (dt <= 0.0f) dt = 0.001f;
    float s = delay_seconds / dt;
    if (s > 63.0f) s = 63.0f;
    if (s < 0.0f) s = 0.0f;
    ac->delay_samples = (uint16_t)s;
    memset(ac->delay_buffer, 0, sizeof(ac->delay_buffer));
}

void adaptive_set_gain_table(adaptive_controller_t *ac, const pid_gains_t gains[SHOT_PHASE_COUNT])
{
    memcpy(ac->gain_table, gains, sizeof(ac->gain_table));
}

float adaptive_update(adaptive_controller_t *ac, float setpoint, float pressure, float flow, float pump_duty)
{
    ac->shot_time += ac->pid.dt;

    // Phase detection and gain scheduling
    shot_phase_t phase = detect_phase(ac->shot_time, pressure);
    if (phase != ac->current_phase) {
        ac->current_phase = phase;
        pid_gains_t *g = &ac->gain_table[phase];

        // Scale gains by resistance ratio
        float r_est = kalman_get_resistance(ac->kf);
        if (r_est > 0.1f)
            ac->gain_scale_factor = ac->resistance_nominal / r_est;
        else
            ac->gain_scale_factor = 1.0f;

        if (ac->gain_scale_factor < 0.3f) ac->gain_scale_factor = 0.3f;
        if (ac->gain_scale_factor > 3.0f) ac->gain_scale_factor = 3.0f;

        pid_set_gains(&ac->pid,
                      g->kp * ac->gain_scale_factor,
                      g->ki * ac->gain_scale_factor,
                      g->kd);
    }

    // Smith predictor: compensate transport delay
    float control_input;
    if (ac->delay_samples > 0) {
        // Internal model: simple first-order response to pump duty
        static const float MODEL_GAIN = 10.0f;
        static const float MODEL_TAU = 0.3f;
        float dt = ac->pid.dt;

        float model_target = MODEL_GAIN * pump_duty;
        ac->model_pressure += dt / MODEL_TAU * (model_target - ac->model_pressure);

        // Delayed model output from circular buffer
        uint16_t delay_idx = (ac->delay_head + 64 - ac->delay_samples) % 64;
        float delayed_model = ac->delay_buffer[delay_idx];

        // Store current model output
        ac->delay_buffer[ac->delay_head] = ac->model_pressure;
        ac->delay_head = (ac->delay_head + 1) % 64;

        // Smith predictor error: use (plant - delayed_model + current_model) as feedback
        float compensated = pressure - delayed_model + ac->model_pressure;
        control_input = pid_update(&ac->pid, setpoint, compensated);
    } else {
        control_input = pid_update(&ac->pid, setpoint, pressure);
    }

    (void)flow;
    return control_input;
}

void adaptive_reset(adaptive_controller_t *ac)
{
    pid_reset(&ac->pid);
    ac->shot_time = 0.0f;
    ac->current_phase = SHOT_PHASE_PREINFUSE;
    ac->model_pressure = 0.0f;
    ac->delay_head = 0;
    memset(ac->delay_buffer, 0, sizeof(ac->delay_buffer));
    ac->gain_scale_factor = 1.0f;
}

shot_phase_t adaptive_get_phase(const adaptive_controller_t *ac)
{
    return ac->current_phase;
}

void adaptive_set_ml_resistance(adaptive_controller_t *ac, float predicted_resistance)
{
    // V-19 fix: reject non-finite or out-of-range values
    if (!isfinite(predicted_resistance) || predicted_resistance < 1.0f)
        return;
    if (predicted_resistance > 100.0f)
        predicted_resistance = 100.0f;
    ac->resistance_nominal = predicted_resistance;
}
