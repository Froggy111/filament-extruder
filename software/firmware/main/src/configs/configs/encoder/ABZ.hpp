#pragma once

#include <stm32h7xx_hal.h>

#include "gpio.hpp"

const int64_t ENCODER_POLARITY = 1;

#define ENCODER_TIMER TIM3
const gpio::PinConfig ENCODER_A = {GPIOB, gpio::Pin::PIN0, gpio::AF::AF2_TIM3};
#define ENCODER_A_CHANNEL TIM_CHANNEL_3

const gpio::PinConfig ENCODER_B = {GPIOB, gpio::Pin::PIN1, gpio::AF::AF2_TIM3};
#define ENCODER_B_CHANNEL TIM_CHANNEL_4

const uint8_t ENCODER_FILTER = 4;
const uint8_t ENCODER_PIN_POLARITY = TIM_ICPOLARITY_RISING;
const uint32_t ENCODER_RESOLUTION = 11 * 4 * 270;

const uint16_t ENCODER_TIMER_PERIOD = 65535;  // using full 16 bits
