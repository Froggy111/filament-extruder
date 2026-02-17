#pragma once
#include <stm32h7xx_hal.h>

#include "gpio.hpp"

// NOTE : software defaults and limits
const float FAN_PWM_FREQ_MIN = 10.0f;
const float FAN_PWM_FREQ_DEFAULT = 100.0f;
const float FAN_PWM_FREQ_MAX = 30000.0f;
const float SSR_PWM_FREQ_MIN = 0.1f;
const float SSR_PWM_FREQ_DEFAULT = 1.0f;
const float SSR_PWM_FREQ_MAX = 10.0f;

const uint16_t PWM_MIN_RESOLUTION = 10;  // minimum bits of resolution
const float PWM_MAX_FREQ_DEVIATION =
    0.01f;  // maximum deviation in actual frequency

// NOTE : hardware map
#define BLOCK1_TIMER TIM23
const gpio::PinConfig FAN_SSR1_PIN = {GPIOF, gpio::Pin::PIN0,
                                      gpio::AF::AF13_TIM23};
#define FAN_SSR1_CHANNEL TIM_CHANNEL_1
const gpio::PinConfig FAN_SSR2_PIN = {GPIOF, gpio::Pin::PIN1,
                                      gpio::AF::AF13_TIM23};
#define FAN_SSR2_CHANNEL TIM_CHANNEL_2
const gpio::PinConfig FAN_SSR3_PIN = {GPIOF, gpio::Pin::PIN2,
                                      gpio::AF::AF13_TIM23};
#define FAN_SSR3_CHANNEL TIM_CHANNEL_3
const gpio::PinConfig FAN_SSR4_PIN = {GPIOF, gpio::Pin::PIN3,
                                      gpio::AF::AF13_TIM23};
#define FAN_SSR4_CHANNEL TIM_CHANNEL_4
const gpio::PinConfig FAN_SSR5_PIN = {GPIOD, gpio::Pin::PIN12,
                                      gpio::AF::AF2_TIM4};

#define BLOCK2_TIMER TIM4
#define FAN_SSR5_CHANNEL TIM_CHANNEL_1
const gpio::PinConfig FAN_SSR6_PIN = {GPIOD, gpio::Pin::PIN13,
                                      gpio::AF::AF2_TIM4};
#define FAN_SSR6_CHANNEL TIM_CHANNEL_2
const gpio::PinConfig FAN_SSR7_PIN = {GPIOD, gpio::Pin::PIN14,
                                      gpio::AF::AF2_TIM4};
#define FAN_SSR7_CHANNEL TIM_CHANNEL_3
const gpio::PinConfig FAN_SSR8_PIN = {GPIOD, gpio::Pin::PIN15,
                                      gpio::AF::AF2_TIM4};
#define FAN_SSR8_CHANNEL TIM_CHANNEL_4
const gpio::PinConfig FAN_SSR9_PIN = {GPIOC, gpio::Pin::PIN6,
                                      gpio::AF::AF3_TIM8};

#define BLOCK3_TIMER TIM8
#define FAN_SSR9_CHANNEL TIM_CHANNEL_1
const gpio::PinConfig FAN_SSR10_PIN = {GPIOC, gpio::Pin::PIN7,
                                       gpio::AF::AF3_TIM8};
#define FAN_SSR10_CHANNEL TIM_CHANNEL_2
const gpio::PinConfig FAN_SSR11_PIN = {GPIOC, gpio::Pin::PIN8,
                                       gpio::AF::AF3_TIM8};
#define FAN_SSR11_CHANNEL TIM_CHANNEL_3
const gpio::PinConfig FAN_SSR12_PIN = {GPIOC, gpio::Pin::PIN9,
                                       gpio::AF::AF3_TIM8};
#define FAN_SSR12_CHANNEL TIM_CHANNEL_4
