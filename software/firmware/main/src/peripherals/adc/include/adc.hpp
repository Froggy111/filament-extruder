#pragma once

#include "config.hpp"

namespace adc {

struct Values {
    float voltage_VMOT = 0;
    float current_U = 0;
    float current_V = 0;
    float current_W = 0;
};

void init(void);
void calibrate_offset(void);

// start conversions
void start_conversions(void);
// read values
Values read(void);

// set OCP threshold
void set_OCP_current(float current);

// used in core_interrupts.cpp
void DMA_ADC1_handler(void);
void DMA_ADC2_handler(void);
void ADC1_conversion_complete_callback(void);
void ADC2_conversion_complete_callback(void);

}  // namespace adc
