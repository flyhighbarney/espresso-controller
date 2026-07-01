/*
 * Espresso Controller — Main entry point
 *
 * FreeRTOS tasks:
 *   Control task  (1 kHz) — sensor read, Kalman, controller, actuator write
 *   Telemetry task (100 Hz) — stream state over USB CDC
 *   Supervisor task (10 Hz) — state machine, safety, command handling
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// Uncomment when building with real HAL:
// #include "stm32g4xx_hal.h"
// #include "FreeRTOS.h"
// #include "task.h"

#include "../App/control/pid.h"
#include "../App/control/kalman.h"
#include "../App/control/adaptive.h"
#include "../App/sensors/sensors.h"
#include "../App/actuators/actuators.h"
#include "../App/telemetry/telemetry.h"
#include "../App/supervisor/supervisor.h"
#include "../App/ml/ml_inference.h"

// --- Global state ---
static sensor_config_t   g_sensor_cfg;
static sensor_readings_t g_readings;
static actuator_state_t  g_actuators;
static kalman_filter_t   g_kalman;
static adaptive_controller_t g_controller;
static telemetry_ctx_t   g_telemetry;
static supervisor_t      g_supervisor;
static ml_context_t      g_ml;

// Controller mode selection
typedef enum {
    CTRL_MODE_PID_BASELINE,
    CTRL_MODE_ADAPTIVE,
    CTRL_MODE_ADAPTIVE_ML,
#ifdef USE_SIMULINK_CONTROLLER
    CTRL_MODE_SIMULINK,
#endif
} controller_mode_t;

static controller_mode_t g_ctrl_mode = CTRL_MODE_ADAPTIVE_ML;
static pid_controller_t  g_baseline_pid;

// ADC DMA buffer (pressure ch0, temperature ch1)
static volatile uint16_t g_adc_dma_buf[2];

static float g_pump_duty = 0.0f;

// Telemetry output callback (USB CDC)
static void telem_send(const uint8_t *data, uint16_t len)
{
    // CDC_Transmit_FS((uint8_t *)data, len);
    (void)data;
    (void)len;
}

// --- Control task (1 kHz) ---
static void control_task(void *arg)
{
    (void)arg;
    const float dt = 0.001f; // 1 kHz

    while (1) {
        // Read sensors
        g_readings.pressure_bar = sensors_adc_to_pressure(&g_sensor_cfg, g_adc_dma_buf[0]);
        g_readings.temperature_c = sensors_adc_to_temperature(&g_sensor_cfg, g_adc_dma_buf[1]);
        g_readings.flow_ml_per_s = sensors_compute_flow(&g_sensor_cfg, 0 /* HAL_GetTick() */);

        // Kalman predict + update
        kalman_predict(&g_kalman, g_pump_duty);
        kalman_update(&g_kalman, g_readings.pressure_bar, g_readings.flow_ml_per_s);

        // Feed ML predictor during first 3 seconds
        // (downsampled: feed every 10th sample for 100 Hz effective rate)
        static uint16_t ml_decimator = 0;
        if (++ml_decimator >= 10) {
            ml_decimator = 0;
            ml_feed_sample(&g_ml, g_readings.pressure_bar, g_readings.flow_ml_per_s);

            if (ml_is_ready(&g_ml) && !g_ml.prediction_valid) {
                ml_run_inference(&g_ml);
                const ml_prediction_t *pred = ml_get_prediction(&g_ml);
                if (pred)
                    adaptive_set_ml_resistance(&g_controller, pred->r_initial);
            }
        }

        // Get setpoint from supervisor
        float setpoint = supervisor_get_pressure_setpoint(&g_supervisor);

        // Run controller
        float output = 0.0f;
        machine_state_t state = supervisor_get_state(&g_supervisor);

        if (state == STATE_PREINFUSE || state == STATE_EXTRACT) {
            switch (g_ctrl_mode) {
            case CTRL_MODE_PID_BASELINE:
                output = pid_update(&g_baseline_pid, setpoint, g_readings.pressure_bar);
                break;
            case CTRL_MODE_ADAPTIVE:
            case CTRL_MODE_ADAPTIVE_ML:
                output = adaptive_update(&g_controller, setpoint,
                                         g_readings.pressure_bar,
                                         g_readings.flow_ml_per_s,
                                         g_pump_duty);
                break;
#ifdef USE_SIMULINK_CONTROLLER
            case CTRL_MODE_SIMULINK:
                // output = simulink_controller_step(setpoint, g_readings.pressure_bar);
                break;
#endif
            }
        }

        g_pump_duty = output;
        pump_set_duty(&g_actuators, output);
        actuators_apply(&g_actuators);

        // vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// --- Telemetry task (100 Hz) ---
static void telemetry_task(void *arg)
{
    (void)arg;

    while (1) {
        telem_state_packet_t pkt;
        pkt.packet_type = TELEM_PKT_STATE;
        pkt.timestamp_ms = 0; // HAL_GetTick();
        pkt.pressure_bar = g_readings.pressure_bar;
        pkt.flow_ml_s = g_readings.flow_ml_per_s;
        pkt.temperature_c = g_readings.temperature_c;
        pkt.pump_duty = g_pump_duty;
        pkt.heater_duty = g_actuators.heater.duty;
        pkt.setpoint_pressure = supervisor_get_pressure_setpoint(&g_supervisor);
        pkt.est_resistance = kalman_get_resistance(&g_kalman);
        pkt.phase = (uint8_t)adaptive_get_phase(&g_controller);
        pkt.machine_state = (uint8_t)supervisor_get_state(&g_supervisor);

        telemetry_send_state(&g_telemetry, &pkt);

        // vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// --- Supervisor task (10 Hz) ---
static void supervisor_task(void *arg)
{
    (void)arg;
    const float dt = 0.1f; // 10 Hz

    while (1) {
        supervisor_update(&g_supervisor, &g_readings, &g_actuators, dt);

        machine_state_t state = supervisor_get_state(&g_supervisor);
        if (state == STATE_IDLE || state == STATE_FLUSH) {
            pid_reset(&g_baseline_pid);
            adaptive_reset(&g_controller);
            ml_reset(&g_ml);
        }

        // vTaskDelay(pdMS_TO_TICKS(100));
    }
}

int main(void)
{
    // HAL_Init();
    // SystemClock_Config();
    // MX_GPIO_Init();
    // MX_DMA_Init();
    // MX_ADC1_Init();
    // MX_TIM2_Init();   // pump PWM
    // MX_TIM3_Init();   // heater PWM
    // MX_TIM4_Init();   // flow meter input capture
    // MX_USB_DEVICE_Init();

    sensors_init(&g_sensor_cfg);
    actuators_init(&g_actuators);
    kalman_init(&g_kalman, 0.001f);
    adaptive_init(&g_controller, &g_kalman, 0.001f);
    adaptive_set_delay(&g_controller, 1.0f, 0.001f);
    telemetry_init(&g_telemetry, telem_send);
    supervisor_init(&g_supervisor);
    ml_init(&g_ml);

    pid_init(&g_baseline_pid, 1.0f, 0.5f, 0.05f, 0.001f);
    pid_set_limits(&g_baseline_pid, 0.0f, 1.0f);

    // xTaskCreate(control_task, "ctrl", 512, NULL, 3, NULL);
    // xTaskCreate(telemetry_task, "telem", 256, NULL, 1, NULL);
    // xTaskCreate(supervisor_task, "super", 256, NULL, 2, NULL);
    // vTaskStartScheduler();

    // Fallback: bare-metal super-loop for testing without FreeRTOS
    while (1) {
        control_task(NULL);
    }
}
