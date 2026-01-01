#pragma once

#include <stdint.h>
#include <stm32g4xx_hal.h>

#include "gpio.hpp"

/**
 * @brief PWM peripheral settings.
 */
// PWM resolution must be above 1 << PWM_MIN_RESOLUTION. This is prioritised
// over frequency deviation
const uint32_t PWM_MIN_RESOLUTION = 6;  // 6 bits of resolution at minimum
// PWM frequency is as far as possible maintained to deviate less than
// PWM_MAX_FREQ_DEVIATION
const float PWM_MAX_FREQ_DEVIATION = 0.05;  // 5% max deviation

// Configuration for drive phase
#define INVERTER_TIMER TIM1  // U, V, W phases on TIM1
#define INVERTER_CLOCK PCLK2
#define PCLK1 1
#define PCLK2 2

#define U_PHASE_CHANNEL TIM_CHANNEL_1
#define V_PHASE_CHANNEL TIM_CHANNEL_3
#define W_PHASE_CHANNEL TIM_CHANNEL_2

/**
 * PC13     ------> TIM1_CH1N (AF4)
 * PB1     ------> TIM1_CH3N (AF6)
 * PB14     ------> TIM1_CH2N (AF6)
 * PA8     ------> TIM1_CH1 (AF6)
 * PA9     ------> TIM1_CH2 (AF6)
 * PA10     ------> TIM1_CH3 (AF6)
 */

// INFO : comment out for drivers that do not use complementary PWM
#define INVERTER_COMPLEMENTARY_PWM

// --- Inverter Leg U (High and Low Side) ---
const gpio::PinConfig INVERTER_U = {GPIOE, gpio::Pin::PIN9, gpio::AF::AF2_TIM1};
const gpio::PinConfig INVERTER_U_N = {GPIOE, gpio::Pin::PIN8,
                                      gpio::AF::AF2_TIM1};

// --- Inverter Leg W (High and Low Side) ---
const gpio::PinConfig INVERTER_V = {GPIOE, gpio::Pin::PIN13,
                                    gpio::AF::AF2_TIM1};
const gpio::PinConfig INVERTER_V_N = {GPIOE, gpio::Pin::PIN12,
                                      gpio::AF::AF2_TIM1};

// --- Inverter Leg V (High and Low Side) ---
const gpio::PinConfig INVERTER_W = {GPIOE, gpio::Pin::PIN11,
                                    gpio::AF::AF2_TIM1};
const gpio::PinConfig INVERTER_W_N = {GPIOE, gpio::Pin::PIN10,
                                      gpio::AF::AF2_TIM1};

const uint32_t INVERTER_BREAKTIME = 500;  // in ns

const uint32_t ADC_CONVERSION_TIME = 1350;  // in ns
const uint32_t ADC_TRIGGER_PRE_OFFSET_TIME = ADC_CONVERSION_TIME / 2;
const uint32_t MIN_TRIGGER_CYCLES = 10;  // clamp to ensure no underflow

// INFO : comment out for drivers without such features

// #define INVERTER_USE_DRIVER_MODE

// maximum high-side on-time, due to bootstrap circuit used
const float MAX_PWM_ONTIME = 0.95;  // 95% should be safe
