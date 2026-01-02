#pragma once

#include <stm32h7xx_hal.h>

#include "gpio.hpp"

/**
 * NOTE : SYSCLK is 160MHz
 * NOTE : Maximum ADC clock is 60MHz
 * NOTE : ADC clock here is 40MHz
 * NOTE : All measurement times will be 1.5x longer
 */
#define ADC_CLK_PRESCALER ADC_CLOCK_ASYNC_DIV4

// INFO : DMA configuration
#define ADC1_DMA_INSTANCE DMA1_Channel1
#define ADC2_DMA_INSTANCE DMA1_Channel2

/**
 * NOTE : For STM32G4 ADCs at 12bit:
 * NOTE : Fast ADCs: ADCx_CHANNEL_1 - CHANNEL_5, rest are slow (minimum 6.5)
 * NOTE : Format: <sampling cycle>, <sampling time (@60MHz)>, <time @40MHz>:
 * NOTE : <fast maximum source impedance>, <slow maximum source impedance>
 * 2.5, 41.67ns, 62.51ns: 100, N/A
 * 6.5, 108.33ns, 162.50ns: 330, 100
 * 12.5, 208.33ns, 312.50ns: 680, 470
 * 24.5, 408.33ns, 612.50ns: 1.5k, 1.2k
 * 47.5, 791.67ns, 1187.51ns: 2.2k, 1.8k
 * 92.5, 1541.67ns, 2312.51ns: 4.7k, 3.9k
 * 247.5, 4.125us, 6.188us: 12k, 10k
 * 640.5, 10.675us, 16.013us: 39k, 33k
 */
constexpr uint8_t ADC_POLL_TIMEOUT = 1;        // should never ever exceed 1ms
constexpr uint16_t ADC_RANGE = (1 << 12) - 1;  // 12 bit ADC
constexpr float ADC_VOLTAGE = 3.3f;

// INFO : VOLTAGE SENSING CONFIGS
// PA0, 27K-1K divider
// source impedance = 27K // 1K = 964Ohm
// minimum sampling cycle: 24.5, 612.50ns @40MHz
const gpio::PinConfig VSENSE_VMOT_PIN = {GPIOA, gpio::Pin::PIN3,
                                         gpio::AF::NONE};
#define VSENSE_VMOT_ADC ADC1
#define VSENSE_VMOT_CHANNEL ADC_CHANNEL_4
#define VSENSE_VMOT_SAMPLETIME ADC_SAMPLETIME_24CYCLES_5
constexpr float VSENSE_VMOT_MULTIPLIER = (27.0f + 1.0f) / 1.0f;

// INFO : CURRENT SENSING CONFIGS

constexpr float ISENSE_SHUNT = 2e-3f;  // 2mOhm shunt
constexpr float ISENSE_GAIN = 83.3f;   // 100x gain pre-amplification
constexpr float ISENSE_CURRENT_PER_VOLT = 1.0f / (ISENSE_SHUNT * ISENSE_GAIN);
constexpr float CURRENT_OFFSET_CALIBRATION_CYCLES = 1000;

const gpio::PinConfig ISENSE_U_PHASE_PIN = {GPIOA, gpio::Pin::PIN0,
                                            gpio::AF::NONE};
#define ISENSE_U_PHASE_ADC ADC2
#define ISENSE_U_PHASE_CHANNEL ADC_CHANNEL_1
#define ISENSE_U_PHASE_SAMPLETIME ADC_SAMPLETIME_2CYCLES_5

const gpio::PinConfig ISENSE_V_PHASE_PIN = {GPIOA, gpio::Pin::PIN1,
                                            gpio::AF::NONE};
#define ISENSE_V_PHASE_ADC ADC2
#define ISENSE_V_PHASE_CHANNEL ADC_CHANNEL_2
#define ISENSE_V_PHASE_SAMPLETIME ADC_SAMPLETIME_2CYCLES_5

const gpio::PinConfig ISENSE_W_PHASE_PIN = {GPIOA, gpio::Pin::PIN2,
                                            gpio::AF::NONE};
#define ISENSE_W_PHASE_ADC ADC1
#define ISENSE_W_PHASE_CHANNEL ADC_CHANNEL_3
#define ISENSE_W_PHASE_SAMPLETIME ADC_SAMPLETIME_2CYCLES_5

const gpio::PinConfig OCP_U_PHASE_PIN = {GPIOB, gpio::Pin::PIN1,
                                         gpio::AF::NONE};
#define OCP_U_PHASE_COMP COMP1
#define OCP_U_PHASE_COMP_INPUT COMP_INPUT_PLUS_IO2
#define OCP_U_PHASE_COMP_INVERTING_INPUT COMP_INPUT_MINUS_DAC3_CH1
#define OCP_U_PHASE_BREAKINPUTSOURCE TIM_BREAKINPUTSOURCE_COMP1

const gpio::PinConfig OCP_V_PHASE_PIN = {GPIOB, gpio::Pin::PIN0,
                                         gpio::AF::NONE};
#define OCP_V_PHASE_COMP COMP4
#define OCP_V_PHASE_COMP_INPUT COMP_INPUT_PLUS_IO1
#define OCP_V_PHASE_COMP_INVERTING_INPUT COMP_INPUT_MINUS_DAC3_CH2
#define OCP_V_PHASE_BREAKINPUTSOURCE TIM_BREAKINPUTSOURCE_COMP4

const gpio::PinConfig OCP_W_PHASE_PIN = {GPIOA, gpio::Pin::PIN7,
                                         gpio::AF::NONE};
#define OCP_W_PHASE_COMP COMP2
#define OCP_W_PHASE_COMP_INPUT COMP_INPUT_PLUS_IO1
#define OCP_W_PHASE_COMP_INVERTING_INPUT COMP_INPUT_MINUS_DAC3_CH2
#define OCP_W_PHASE_BREAKINPUTSOURCE TIM_BREAKINPUTSOURCE_COMP2
