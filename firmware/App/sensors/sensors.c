#include "sensors.h"
#include <math.h>

#define ADC_MAX 4095
#define ADC_VREF 3.3f

void sensors_init(sensor_config_t *cfg)
{
    cfg->pressure_v_min = 0.5f;
    cfg->pressure_v_max = 4.5f;
    cfg->pressure_bar_min = 0.0f;
    cfg->pressure_bar_max = 13.8f;  // 200 psi

    cfg->ntc_r_series = 10000.0f;
    cfg->ntc_beta = 3950.0f;
    cfg->ntc_r_nominal = 10000.0f;
    cfg->ntc_t_nominal = 298.15f;  // 25C in Kelvin

    cfg->flow_pulses_per_ml = 5.5f; // YF-S401 typical

    cfg->flow_pulse_count = 0;
    cfg->last_flow_count = 0;
    cfg->last_flow_time_ms = 0;
}

float sensors_adc_to_pressure(const sensor_config_t *cfg, uint16_t adc_raw)
{
    float voltage = (float)adc_raw / ADC_MAX * ADC_VREF;

    if (voltage < cfg->pressure_v_min)
        voltage = cfg->pressure_v_min;
    if (voltage > cfg->pressure_v_max)
        voltage = cfg->pressure_v_max;

    float fraction = (voltage - cfg->pressure_v_min) / (cfg->pressure_v_max - cfg->pressure_v_min);
    return cfg->pressure_bar_min + fraction * (cfg->pressure_bar_max - cfg->pressure_bar_min);
}

float sensors_adc_to_temperature(const sensor_config_t *cfg, uint16_t adc_raw)
{
    // V-10 fix: reject ADC values near rails to prevent division-by-near-zero
    if (adc_raw < SENSOR_ADC_GUARD || adc_raw > (ADC_MAX - SENSOR_ADC_GUARD))
        return -999.0f; // fault

    float voltage = (float)adc_raw / ADC_MAX * ADC_VREF;
    float denominator = ADC_VREF - voltage;

    if (denominator < 0.01f)
        return -999.0f; // fault — avoid division by near-zero

    float r_ntc = cfg->ntc_r_series * voltage / denominator;

    if (r_ntc <= 0.0f)
        return -999.0f;

    // Simplified Steinhart-Hart (B-parameter equation)
    float inv_t = (1.0f / cfg->ntc_t_nominal) +
                  (1.0f / cfg->ntc_beta) * logf(r_ntc / cfg->ntc_r_nominal);

    if (inv_t <= 0.0f)
        return -999.0f;

    float temp_c = (1.0f / inv_t) - 273.15f;

    // V-10 fix: plausibility check — reject physically impossible readings
    if (temp_c < SENSOR_TEMP_MIN || temp_c > SENSOR_TEMP_MAX)
        return -999.0f;

    return temp_c;
}

float sensors_compute_flow(sensor_config_t *cfg, uint32_t current_time_ms)
{
    // V-03 fix: atomic snapshot of volatile counter to prevent ISR tearing
    // On Cortex-M4, 32-bit aligned reads are atomic, but we use __disable_irq
    // to ensure the read-and-update of last_flow_count is consistent
#ifndef UNIT_TEST
    __disable_irq();
#endif
    uint32_t count = cfg->flow_pulse_count;
#ifndef UNIT_TEST
    __enable_irq();
#endif

    uint32_t dt_ms = current_time_ms - cfg->last_flow_time_ms;

    if (dt_ms == 0)
        return 0.0f;

    uint32_t delta_pulses = count - cfg->last_flow_count;
    cfg->last_flow_count = count;
    cfg->last_flow_time_ms = current_time_ms;

    float ml = (float)delta_pulses / cfg->flow_pulses_per_ml;
    float seconds = (float)dt_ms / 1000.0f;

    return ml / seconds;
}

void sensors_read(sensor_config_t *cfg, sensor_readings_t *out)
{
    (void)cfg;
    (void)out;
}

bool sensors_validate(const sensor_readings_t *r)
{
    if (!isfinite(r->pressure_bar) || r->pressure_bar < SENSOR_PRESSURE_MIN || r->pressure_bar > SENSOR_PRESSURE_MAX)
        return false;
    if (!isfinite(r->flow_ml_per_s) || r->flow_ml_per_s < SENSOR_FLOW_MIN || r->flow_ml_per_s > SENSOR_FLOW_MAX)
        return false;
    if (r->temperature_c < -900.0f)
        return false;
    if (!isfinite(r->temperature_c))
        return false;
    return true;
}

void sensors_flow_pulse_isr(sensor_config_t *cfg)
{
    cfg->flow_pulse_count++;
}
