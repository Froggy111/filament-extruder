#include "config.hpp"
#if ENCODER_TYPE == ABZ

#include <stm32h7xx_hal.h>

#include "ABZ.hpp"
#include "configs/encoder.hpp"
#include "debug.hpp"
#include "error.hpp"

// Volatile variables for ISR access
static volatile int64_t total_count = 0;
static volatile uint8_t old_state = 0;
void handle_encoder_edge(void* args);

// Quadrature Lookup Table to determine direction
// [OldState][NewState] -> +1, -1, or 0 (invalid/no change)
// States are formed by (A << 1) | B
static const int8_t ENCODER_QrT[4][4] = {
    {0, 1, -1, 0},  // 00 -> 00, 01, 10, 11
    {-1, 0, 0, 1},  // 01 -> ...
    {1, 0, 0, -1},  // 10 -> ...
    {0, -1, 1, 0}   // 11 -> ...
};

void encoder::init(void) {
    // Initialize GPIOs in Interrupt Mode (Both Edges)
    gpio::attach_interrupt(ENCODER_A, gpio::Mode::INT_RISING_FALLING,
                           gpio::Pull::NOPULL, gpio::Speed::VERY_HIGH,
                           handle_encoder_edge, NULL);
    gpio::attach_interrupt(ENCODER_B, gpio::Mode::INT_RISING_FALLING,
                           gpio::Pull::NOPULL, gpio::Speed::VERY_HIGH,
                           handle_encoder_edge, NULL);

    // Read initial state
    uint8_t a = gpio::read(ENCODER_A);
    uint8_t b = gpio::read(ENCODER_B);
    old_state = (a << 1) | b;
}

int64_t encoder::get_count(void) {
    // Atomic read might be needed depending on architecture,
    // but int64 read on 32-bit MCU is usually safe enough if transient glitches
    // are acceptable, otherwise disable IRQ briefly.
    int64_t c;
    __disable_irq();
    c = total_count;
    __enable_irq();
    return c * ENCODER_POLARITY;
}

void encoder::set_count(int32_t count) {
    __disable_irq();
    total_count = count * ENCODER_POLARITY;
    __enable_irq();
}

// --- Interrupt Handlers ---

// Inline function to handle state update logic
void handle_encoder_edge([[maybe_unused]] void* args) {
    // Fast read of IDR register (Direct register access for speed)
    // Assuming GPIOB for both. If mixed ports, read separately.
    // uint32_t gpio_val = GPIOB->IDR;

    // Extract bits 0 and 1 (modify if pins change)
    // uint8_t a = (gpio_val & GPIO_PIN_0) ? 1 : 0;
    // uint8_t b = (gpio_val & GPIO_PIN_1) ? 1 : 0;
    uint8_t a = gpio::read(ENCODER_A);
    uint8_t b = gpio::read(ENCODER_B);
    uint8_t new_state = (a << 1) | b;

    // Update count based on table
    total_count += ENCODER_QrT[old_state][new_state];
    old_state = new_state;
}

#endif
