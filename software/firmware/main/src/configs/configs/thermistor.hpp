#pragma once

#include <stm32h7xx_hal.h>

#include "gpio.hpp"

const float THERMISTOR_R = 1000.0f;
const float ADC_VREF = 3.3f;
#define THERM_SAMPLETIME ADC_SAMPLETIME_16CYCLES_5

const gpio::PinConfig THERM1_PIN = {GPIOF, gpio::Pin::PIN14, gpio::AF::NONE};
#define THERM1_ADC ADC3
#define THERM1_CHANNEL ADC_CHANNEL_9
const gpio::PinConfig THERM2_PIN = {GPIOC, gpio::Pin::PIN0, gpio::AF::NONE};
#define THERM2_ADC ADC1  // can be 2 or 3
#define THERM2_CHANNEL ADC_CHANNEL_10
const gpio::PinConfig THERM3_PIN = {GPIOC, gpio::Pin::PIN2, gpio::AF::NONE};
#define THERM3_ADC ADC3
#define THERM3_CHANNEL ADC_CHANNEL_0
const gpio::PinConfig THERM4_PIN = {GPIOC, gpio::Pin::PIN3, gpio::AF::NONE};
#define THERM4_ADC ADC3
#define THERM4_CHANNEL ADC_CHANNEL_1
const gpio::PinConfig THERM5_PIN = {GPIOA, gpio::Pin::PIN3, gpio::AF::NONE};
#define THERM5_ADC ADC1
#define THERM5_CHANNEL ADC_CHANNEL_15
const gpio::PinConfig THERM6_PIN = {GPIOA, gpio::Pin::PIN4, gpio::AF::NONE};
#define THERM6_ADC ADC1
#define THERM6_CHANNEL ADC_CHANNEL_18
const gpio::PinConfig THERM7_PIN = {GPIOA, gpio::Pin::PIN5, gpio::AF::NONE};
#define THERM7_ADC ADC2
#define THERM7_CHANNEL ADC_CHANNEL_19
const gpio::PinConfig THERM8_PIN = {GPIOA, gpio::Pin::PIN6, gpio::AF::NONE};
#define THERM8_ADC ADC2
#define THERM8_CHANNEL ADC_CHANNEL_3
