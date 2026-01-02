#pragma once

#include <stm32h7xx_hal.h>

#include "gpio.hpp"

#define UART_BUFFER_SIZE 256

const gpio::PinConfig UART_TX = {GPIOA, gpio::Pin::PIN9, gpio::AF::AF7_USART1};
const gpio::PinConfig UART_RX = {GPIOA, gpio::Pin::PIN10, gpio::AF::AF7_USART1};

#define UART_INSTANCE USART1
#define UART_TX_DMA_INSTANCE DMA1_Channel3
#define UART_RX_DMA_INSTANCE DMA1_Channel4

// uncomment for RS485
// NOTE : also define UART_DE PinConfig
#define USE_RS485
const gpio::PinConfig UART_DE = {GPIOA, gpio::Pin::PIN12, gpio::AF::AF7_USART1};
// const gpio::PinConfig UART_DE = {GPIOA, gpio::Pin::PIN12, gpio::AF::NONE};
