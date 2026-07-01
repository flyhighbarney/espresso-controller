#include "actuators.h"
#include <math.h>

void actuators_init(actuator_state_t *act)
{
    act->pump.duty = 0.0f;
    act->pump.pwm_period = 1000;
    act->pump.enabled = false;

    act->heater.duty = 0.0f;
    act->heater.max_duty = 0.8f;
    act->heater.enabled = false;

    act->solenoid.open = false;
}

void pump_set_duty(actuator_state_t *act, float duty)
{
    // V-05 fix: NaN/Inf produces 0 duty (safe shutdown)
    if (!isfinite(duty))
        duty = 0.0f;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > 1.0f) duty = 1.0f;
    act->pump.duty = duty;
}

void pump_enable(actuator_state_t *act, bool enable)
{
    act->pump.enabled = enable;
    if (!enable)
        act->pump.duty = 0.0f;
}

void heater_set_duty(actuator_state_t *act, float duty)
{
    if (!isfinite(duty))
        duty = 0.0f;
    if (duty < 0.0f) duty = 0.0f;
    if (duty > act->heater.max_duty) duty = act->heater.max_duty;
    act->heater.duty = duty;
}

void heater_enable(actuator_state_t *act, bool enable)
{
    act->heater.enabled = enable;
    if (!enable)
        act->heater.duty = 0.0f;
}

void solenoid_set(actuator_state_t *act, bool open)
{
    act->solenoid.open = open;
}

void actuators_emergency_stop(actuator_state_t *act)
{
    act->pump.duty = 0.0f;
    act->pump.enabled = false;
    act->heater.duty = 0.0f;
    act->heater.enabled = false;
    act->solenoid.open = true;
}

void actuators_apply(const actuator_state_t *act)
{
#ifndef UNIT_TEST
    // V-16 fix: re-check heater max_duty before applying to hardware
    uint16_t pump_ccr = 0;
    if (act->pump.enabled) {
        float duty = act->pump.duty;
        if (!isfinite(duty) || duty < 0.0f) duty = 0.0f;
        if (duty > 1.0f) duty = 1.0f;
        pump_ccr = (uint16_t)(duty * act->pump.pwm_period);
    }
    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pump_ccr);
    (void)pump_ccr;

    uint16_t heater_ccr = 0;
    if (act->heater.enabled) {
        float duty = act->heater.duty;
        if (!isfinite(duty) || duty < 0.0f) duty = 0.0f;
        if (duty > act->heater.max_duty) duty = act->heater.max_duty;
        heater_ccr = (uint16_t)(duty * 1000);
    }
    // __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, heater_ccr);
    (void)heater_ccr;

    // HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, act->solenoid.open ? GPIO_PIN_SET : GPIO_PIN_RESET);
#endif
}
