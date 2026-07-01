#include "supervisor.h"

void supervisor_init(supervisor_t *sv)
{
    sv->state = STATE_IDLE;
    sv->fault_flags = FAULT_NONE;
    sv->state_time_s = 0.0f;
    sv->last_flow_time_s = 0.0f;
    sv->shot_requested = false;
    sv->flush_requested = false;

    sv->config.max_pressure_bar = 12.0f;
    sv->config.max_temperature_c = 110.0f;
    sv->config.preheat_target_c = 93.0f;
    sv->config.preinfuse_pressure_bar = 3.0f;
    sv->config.preinfuse_duration_s = 5.0f;
    sv->config.extract_pressure_bar = 9.0f;
    sv->config.max_shot_duration_s = 40.0f;
    sv->config.flush_duration_s = 3.0f;
    sv->config.dry_fire_timeout_s = 10.0f;
}

static void enter_state(supervisor_t *sv, machine_state_t new_state)
{
    sv->state = new_state;
    sv->state_time_s = 0.0f;
}

static void check_safety(supervisor_t *sv, const sensor_readings_t *sensors,
                          actuator_state_t *actuators)
{
    if (sensors->pressure_bar > sv->config.max_pressure_bar) {
        sv->fault_flags |= FAULT_OVERPRESSURE;
        actuators_emergency_stop(actuators);
        enter_state(sv, STATE_FAULT);
    }

    if (sensors->temperature_c > sv->config.max_temperature_c) {
        sv->fault_flags |= FAULT_OVERTEMP;
        actuators_emergency_stop(actuators);
        enter_state(sv, STATE_FAULT);
    }

    if (sensors->flow_ml_per_s > 0.1f)
        sv->last_flow_time_s = 0.0f;

    if (actuators->heater.enabled && actuators->heater.duty > 0.0f) {
        if (sv->last_flow_time_s > sv->config.dry_fire_timeout_s) {
            sv->fault_flags |= FAULT_DRY_FIRE;
            actuators_emergency_stop(actuators);
            enter_state(sv, STATE_FAULT);
        }
    }

    // V-05: sensor validation — NaN/Inf or out-of-range readings indicate sensor failure
    if (sensors->temperature_c < -900.0f || sensors->pressure_bar < -0.5f) {
        sv->fault_flags |= FAULT_SENSOR_FAIL;
        actuators_emergency_stop(actuators);
        enter_state(sv, STATE_FAULT);
    }

    if (!sensors_validate(sensors)) {
        sv->fault_flags |= FAULT_SENSOR_FAIL;
        actuators_emergency_stop(actuators);
        enter_state(sv, STATE_FAULT);
    }
}

void supervisor_update(supervisor_t *sv, const sensor_readings_t *sensors,
                       actuator_state_t *actuators, float dt)
{
    sv->state_time_s += dt;
    sv->last_flow_time_s += dt;

    if (sv->state != STATE_FAULT)
        check_safety(sv, sensors, actuators);

    switch (sv->state) {
    case STATE_IDLE:
        pump_enable(actuators, false);
        heater_enable(actuators, false);
        solenoid_set(actuators, false);

        if (sv->shot_requested) {
            sv->shot_requested = false;
            enter_state(sv, STATE_PREHEAT);
            heater_enable(actuators, true);
        }
        break;

    case STATE_PREHEAT:
        if (sensors->temperature_c >= sv->config.preheat_target_c) {
            enter_state(sv, STATE_PREINFUSE);
            pump_enable(actuators, true);
        }
        if (sensors->temperature_c < sv->config.preheat_target_c - 2.0f)
            heater_set_duty(actuators, 1.0f);
        else
            heater_set_duty(actuators, 0.0f);
        break;

    case STATE_PREINFUSE:
        if (sv->state_time_s >= sv->config.preinfuse_duration_s)
            enter_state(sv, STATE_EXTRACT);
        break;

    case STATE_EXTRACT:
        if (sv->state_time_s >= sv->config.max_shot_duration_s) {
            enter_state(sv, STATE_FLUSH);
        }
        break;

    case STATE_FLUSH:
        pump_enable(actuators, false);
        solenoid_set(actuators, true);
        if (sv->state_time_s >= sv->config.flush_duration_s) {
            solenoid_set(actuators, false);
            enter_state(sv, STATE_IDLE);
        }
        break;

    case STATE_FAULT:
        actuators_emergency_stop(actuators);
        break;

    default:
        enter_state(sv, STATE_FAULT);
        break;
    }

    if (sv->flush_requested) {
        sv->flush_requested = false;
        if (sv->state == STATE_EXTRACT || sv->state == STATE_PREINFUSE)
            enter_state(sv, STATE_FLUSH);
    }
}

void supervisor_request_shot(supervisor_t *sv)
{
    if (sv->state == STATE_IDLE)
        sv->shot_requested = true;
}

void supervisor_request_flush(supervisor_t *sv)
{
    sv->flush_requested = true;
}

void supervisor_request_stop(supervisor_t *sv)
{
    // V-07 fix: transition through FLUSH to depressurize instead of jumping to IDLE
    if (sv->state == STATE_EXTRACT || sv->state == STATE_PREINFUSE)
        enter_state(sv, STATE_FLUSH);
    else if (sv->state == STATE_PREHEAT)
        enter_state(sv, STATE_IDLE);
}

machine_state_t supervisor_get_state(const supervisor_t *sv) { return sv->state; }
uint32_t supervisor_get_faults(const supervisor_t *sv)       { return sv->fault_flags; }

void supervisor_clear_faults(supervisor_t *sv)
{
    // V-12 fix: only clear faults — do not transition out of FAULT state here.
    // The caller must verify sensor readings are safe before requesting a shot.
    sv->fault_flags = FAULT_NONE;
}

void supervisor_clear_faults_if_safe(supervisor_t *sv, const sensor_readings_t *sensors)
{
    if (sv->state != STATE_FAULT)
        return;

    if (sensors->pressure_bar > sv->config.max_pressure_bar * 0.9f)
        return;
    if (sensors->temperature_c > sv->config.max_temperature_c * 0.9f)
        return;
    if (!sensors_validate(sensors))
        return;

    sv->fault_flags = FAULT_NONE;
    enter_state(sv, STATE_IDLE);
}

float supervisor_get_pressure_setpoint(const supervisor_t *sv)
{
    switch (sv->state) {
    case STATE_PREINFUSE: return sv->config.preinfuse_pressure_bar;
    case STATE_EXTRACT:   return sv->config.extract_pressure_bar;
    default:              return 0.0f;
    }
}
