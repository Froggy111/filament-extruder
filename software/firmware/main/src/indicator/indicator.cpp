#include "indicator.hpp"

#include <stm32h7xx_hal.h>

#include "config.hpp"
#include "gpio.hpp"

volatile static uint32_t bit_idx = 0;
volatile static uint32_t data = 0;
volatile static uint32_t last_tick = 0;
void clk_irq(void* args);

void indicator::init(void) {
    gpio::init(INDICATOR_DATA, gpio::Mode::INPUT, gpio::Pull::UP,
               gpio::Speed::LOW);
    gpio::attach_interrupt(INDICATOR_CLK, gpio::Mode::INT_RISING,
                           gpio::Pull::UP, gpio::Speed::LOW, clk_irq, NULL);
    last_tick = HAL_GetTick();
    return;
}

float indicator::read(void) {
    uint32_t value = data & 0xFFFFF;
    uint32_t sign_bit = (data >> 20) & 0b1;
    uint32_t mode_bit = (data >> 22) & 0b1;
    if (mode_bit != 0) {
        // not in metric mode
        return 0.0f;
    }
    float val = value / 1000.0f;
    if (sign_bit) {
        // negative
        val = -val;
    }
    return val;
}

void clk_irq([[maybe_unused]] void* args) {
    uint32_t current_tick = HAL_GetTick();
    uint32_t time_elapsed = current_tick - last_tick;
    if (time_elapsed > INDICATOR_CLK_TIMEOUT) {
        data = 0;
        bit_idx = 0;
    }

    if (bit_idx < 24) {
        bool state = gpio::read(INDICATOR_DATA);
        if (state) {
            data |= (1UL << bit_idx);
        }
        bit_idx++;
    }

    last_tick = current_tick;
    return;
}
