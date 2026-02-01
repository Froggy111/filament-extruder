#pragma once

#include <stm32h7xx_hal.h>

#include "gpio.hpp"

#define STEPPER1_UART USART1
const gpio::PinConfig STEPPER1_TX = {GPIOB, gpio::Pin::PIN6,
                                     gpio::AF::AF7_USART1};
const gpio::PinConfig STEPPER1_RX = {GPIOB, gpio::Pin::PIN7,
                                     gpio::AF::AF7_USART1};
const gpio::PinConfig STEPPER1_DIR = {GPIOB, gpio::Pin::PIN3, gpio::AF::NONE};
const gpio::PinConfig STEPPER1_STEP = {GPIOB, gpio::Pin::PIN4, gpio::AF::NONE};
const gpio::PinConfig STEPPER1_ENN = {GPIOB, gpio::Pin::PIN5, gpio::AF::NONE};
const gpio::PinConfig STEPPER1_INDEX = {GPIOB, gpio::Pin::PIN8, gpio::AF::NONE};
const gpio::PinConfig STEPPER1_DIAG = {GPIOB, gpio::Pin::PIN9, gpio::AF::NONE};
const uint8_t STEPPER1_ADDR = 0x00;
const float STEPPER1_SHUNT_RESISTANCE = 0.100f;  // 100mOhm
#define STEPPER1_TIMER TIM12
const uint32_t STEPPER1_TIMER_FREQ = 10e6;  // Hz

#define STEPPER2_UART USART10
const gpio::PinConfig STEPPER2_TX = {GPIOE, gpio::Pin::PIN3,
                                     gpio::AF::AF11_USART10};
const gpio::PinConfig STEPPER2_RX = {GPIOE, gpio::Pin::PIN2,
                                     gpio::AF::AF11_USART10};
const gpio::PinConfig STEPPER2_DIR = {GPIOE, gpio::Pin::PIN0, gpio::AF::NONE};
const gpio::PinConfig STEPPER2_STEP = {GPIOE, gpio::Pin::PIN1, gpio::AF::NONE};
const gpio::PinConfig STEPPER2_ENN = {GPIOE, gpio::Pin::PIN6, gpio::AF::NONE};
const gpio::PinConfig STEPPER2_INDEX = {GPIOE, gpio::Pin::PIN4, gpio::AF::NONE};
const gpio::PinConfig STEPPER2_DIAG = {GPIOE, gpio::Pin::PIN5, gpio::AF::NONE};
const uint8_t STEPPER2_ADDR = 0x00;
const float STEPPER2_SHUNT_RESISTANCE = 0.100f;  // 100mOhm
#define STEPPER2_TIMER TIM13
const uint32_t STEPPER2_TIMER_FREQ = 10e6;  // Hz
