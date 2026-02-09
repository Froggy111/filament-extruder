#pragma once

#include <stm32h7xx_hal.h>

#include "gpio.hpp"

const int64_t ENCODER_POLARITY = -1;

const gpio::PinConfig ENCODER_A = {GPIOB, gpio::Pin::PIN0, gpio::AF::AF2_TIM3};
const gpio::PinConfig ENCODER_B = {GPIOB, gpio::Pin::PIN1, gpio::AF::AF2_TIM3};
const uint32_t ENCODER_RESOLUTION = 11 * 4 * 270;
