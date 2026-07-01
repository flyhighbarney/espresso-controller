#ifndef SENSORS_H
#define SENSORS_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float pressure_bar;
    float flow_ml_per_s;
    float temperature_c;
    uint32_t flow_pulse_count;
    uint32_t timestamp_ms;
} sensor_readings_t;

typedef struct {
    // Pressure transducer calibration (0.5–4.5V → 0–13.8 bar / 200 psi)
    float pressure_v_min;
    float pressure_v_max;
    float pressure_bar_min;
    float pressure_bar_max;

    // NTC thermistor parameters (Steinhart-Hart)
    float ntc_r_series;     // series resistor (ohms)
    float ntc_beta;         // B-coefficient
    float ntc_r_nominal;    // resistance at T_nominal
    float ntc_t_nominal;    // nominal temperature (K)

    // Flow meter calibration
    float flow_pulses_per_ml;

    // Internal state — flow_pulse_count is written from ISR, must be volatile
    volatile uint32_t flow_pulse_count;
    uint32_t last_flow_count;
    uint32_t last_flow_time_ms;
} sensor_config_t;

// Plausibility bounds for sensor validation
#define SENSOR_PRESSURE_MIN  -0.5f
#define SENSOR_PRESSURE_MAX   16.0f
#define SENSOR_FLOW_MIN       0.0f
#define SENSOR_FLOW_MAX       20.0f
#define SENSOR_TEMP_MIN      -40.0f
#define SENSOR_TEMP_MAX       150.0f
#define SENSOR_ADC_GUARD       10    // reject ADC values within this margin of rails

void sensors_init(sensor_config_t *cfg);
void sensors_read(sensor_config_t *cfg, sensor_readings_t *out);
float sensors_adc_to_pressure(const sensor_config_t *cfg, uint16_t adc_raw);
float sensors_adc_to_temperature(const sensor_config_t *cfg, uint16_t adc_raw);
float sensors_compute_flow(sensor_config_t *cfg, uint32_t current_time_ms);
bool sensors_validate(const sensor_readings_t *r);

// Called from timer capture ISR
void sensors_flow_pulse_isr(sensor_config_t *cfg);

#endif
