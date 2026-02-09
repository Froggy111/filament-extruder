#pragma once

#include <stm32h7xx_hal.h>

namespace thermistor {

enum class Port : uint8_t {
    P1 = 0,
    P2 = 1,
    P3 = 2,
    P4 = 3,
    P5 = 4,
    P6 = 5,
    P7 = 6,
    P8 = 7,
};

enum class InitStatus : uint8_t {
    OK,
    ADC1_INIT_FAILED,
    ADC1_CALIBRATION_FAILED,
    ADC2_INIT_FAILED,
    ADC2_CALIBRATION_FAILED,
    ADC3_INIT_FAILED,
    ADC3_CALIBRATION_FAILED,
};

InitStatus init(void);
float read(Port port);

}  // namespace thermistor
