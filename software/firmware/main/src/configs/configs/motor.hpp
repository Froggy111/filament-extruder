#pragma once

#include <stm32h7xx_hal.h>

#include "gpio.hpp"

#define MOTOR_TIMER TIM1
#define MOTOR_CHANNEL TIM_CHANNEL_1
const gpio::PinConfig MOTOR_IN1 = {GPIOE, gpio::Pin::PIN9, gpio::AF::AF1_TIM1};
const gpio::PinConfig MOTOR_IN2 = {GPIOE, gpio::Pin::PIN8, gpio::AF::NONE};
