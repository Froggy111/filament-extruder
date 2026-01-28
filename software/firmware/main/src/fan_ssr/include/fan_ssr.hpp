#pragma once

#include <stm32h7xx_hal.h>

#include "config.hpp"
#include "gpio.hpp"

namespace fan_ssr {

enum class Mode : uint8_t {
    FAN = 0,
    SSR = 1,
};

enum class Port : uint8_t {
    P1 = 0,
    P2 = 1,
    P3 = 2,
    P4 = 3,
    P5 = 4,
    P6 = 5,
    P7 = 6,
    P8 = 7,
    P9 = 8,
    P10 = 9,
    P11 = 10,
    P12 = 11,
};

enum class InitStatus : uint8_t {
    OK = 0,
    BLOCK1_FREQ_OUT_OF_RANGE = 1,
    BLOCK2_FREQ_OUT_OF_RANGE = 2,
    BLOCK3_FREQ_OUT_OF_RANGE = 3,
    BLOCK1_FREQ_DEVIATION_TOO_LARGE = 4,
    BLOCK2_FREQ_DEVIATION_TOO_LARGE = 5,
    BLOCK3_FREQ_DEVIATION_TOO_LARGE = 6,
    BLOCK1_RESOLUTION_TOO_LOW = 7,
    BLOCK2_RESOLUTION_TOO_LOW = 8,
    BLOCK3_RESOLUTION_TOO_LOW = 9,
    BLOCK1_FREQ_TARGET_INVALID = 10,
    BLOCK2_FREQ_TARGET_INVALID = 11,
    BLOCK3_FREQ_TARGET_INVALID = 12,
    HAL_ERR = 13,
};

InitStatus init(Mode block_1_mode, float block_1_freq, Mode block_2_mode,
                float block_2_freq, Mode block_3_mode, float block_3_freq);
bool enable_port(Port port);
bool disable_port(Port port);
bool set_duty_cycle(Port port, float duty_cycle);

}  // namespace fan_ssr
