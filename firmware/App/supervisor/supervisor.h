#ifndef SUPERVISOR_H
#define SUPERVISOR_H

#include <stdint.h>
#include <stdbool.h>
#include "../sensors/sensors.h"
#include "../actuators/actuators.h"

typedef enum {
    STATE_IDLE,
    STATE_PREHEAT,
    STATE_PREINFUSE,
    STATE_EXTRACT,
    STATE_FLUSH,
    STATE_FAULT,
    STATE_COUNT
} machine_state_t;

typedef enum {
    FAULT_NONE           = 0,
    FAULT_OVERPRESSURE   = (1 << 0),
    FAULT_OVERTEMP       = (1 << 1),
    FAULT_DRY_FIRE       = (1 << 2),
    FAULT_SENSOR_FAIL    = (1 << 3),
    FAULT_WATCHDOG       = (1 << 4),
} fault_flags_t;

typedef struct {
    float max_pressure_bar;
    float max_temperature_c;
    float preheat_target_c;
    float preinfuse_pressure_bar;
    float preinfuse_duration_s;
    float extract_pressure_bar;
    float max_shot_duration_s;
    float flush_duration_s;
    float dry_fire_timeout_s;
} supervisor_config_t;

typedef struct {
    machine_state_t state;
    uint32_t fault_flags;
    float state_time_s;
    float last_flow_time_s;
    bool shot_requested;
    bool flush_requested;
    supervisor_config_t config;
} supervisor_t;

void supervisor_init(supervisor_t *sv);
void supervisor_update(supervisor_t *sv, const sensor_readings_t *sensors,
                       actuator_state_t *actuators, float dt);
void supervisor_request_shot(supervisor_t *sv);
void supervisor_request_flush(supervisor_t *sv);
void supervisor_request_stop(supervisor_t *sv);
machine_state_t supervisor_get_state(const supervisor_t *sv);
uint32_t supervisor_get_faults(const supervisor_t *sv);
void supervisor_clear_faults(supervisor_t *sv);
void supervisor_clear_faults_if_safe(supervisor_t *sv, const sensor_readings_t *sensors);
float supervisor_get_pressure_setpoint(const supervisor_t *sv);

#endif
