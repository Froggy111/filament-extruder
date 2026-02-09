#pragma once

#include "gpio.hpp"

const gpio::PinConfig INDICATOR_DATA = {GPIOE, gpio::Pin::PIN13,
                                        gpio::AF::NONE};
const gpio::PinConfig INDICATOR_CLK = {GPIOE, gpio::Pin::PIN14, gpio::AF::NONE};
const uint32_t INDICATOR_CLK_TIMEOUT = 10;  // in ms
