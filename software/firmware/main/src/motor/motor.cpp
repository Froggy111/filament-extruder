#include "motor.hpp"

#include <stm32h7xx_hal.h>

#include <cmath>

#include "config.hpp"

static float max_duty_cycle = 0.0f;
static uint16_t period = 0;

static TIM_HandleTypeDef timer = {};

bool motor::init(float max_duty_cycle_) {
    max_duty_cycle = std::fabs(max_duty_cycle);
    if (max_duty_cycle > 0.5f) {
        return false;
    }
    max_duty_cycle = max_duty_cycle_;
    period = (TIMER_INPUT_FREQ / 20000) - 1;
    if (MOTOR_TIMER == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
    }
    timer.Instance = MOTOR_TIMER;
    timer.Init.Prescaler = 0;
    timer.Init.Period = period;
    timer.Init.CounterMode = TIM_COUNTERMODE_UP;
    timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer.Init.RepetitionCounter = 1;
    timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&timer) != HAL_OK) {
        return false;
    }

    TIM_MasterConfigTypeDef master_config = {};
    master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    master_config.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&timer, &master_config) !=
        HAL_OK) {
        return false;
    }

    TIM_OC_InitTypeDef oc_config = {};
    oc_config.OCMode = TIM_OCMODE_PWM1;          // duty cycle = pulse / period
    oc_config.Pulse = 0;                         // 0 duty cycle initially
    oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;  // high == on
    oc_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;   // high == on
    oc_config.OCFastMode = TIM_OCFAST_DISABLE;      // not used for PWM
    oc_config.OCIdleState = TIM_OCIDLESTATE_RESET;  // high side off by default
    oc_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;  // low side off by default
    if (HAL_TIM_OC_ConfigChannel(&timer, &oc_config, MOTOR_CHANNEL) != HAL_OK) {
        return false;
    }
    __HAL_TIM_ENABLE_OCxPRELOAD(&timer, MOTOR_CHANNEL);
    __HAL_TIM_SET_COMPARE(&timer, MOTOR_CHANNEL, 0);
    if (HAL_TIM_PWM_Start(&timer, MOTOR_CHANNEL) != HAL_OK) {
        return false;
    }
    __HAL_TIM_MOE_ENABLE(&timer);
    gpio::init(MOTOR_IN1, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::HIGH);
    gpio::init(MOTOR_IN2, gpio::Mode::OUTPUT_PP_, gpio::Pull::NOPULL,
               gpio::Speed::HIGH);
    gpio::write(MOTOR_IN2, gpio::LOW);
    return true;
}

void motor::set_duty_cycle(float duty_cycle) {
    if (duty_cycle > max_duty_cycle) {
        return;
    }
    uint16_t compare_val = duty_cycle * period;
    __HAL_TIM_SET_COMPARE(&timer, MOTOR_CHANNEL, compare_val);
}
