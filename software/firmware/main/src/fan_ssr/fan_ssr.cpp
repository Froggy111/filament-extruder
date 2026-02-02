#include "fan_ssr.hpp"

#include <stm32h7xx_hal.h>

#include <cmath>

#include "config.hpp"
#include "gpio.hpp"

struct PortConfig {
    gpio::PinConfig pin = {};
    TIM_TypeDef* timer = nullptr;
    uint32_t channel = 0;
};

enum class State : uint8_t {
    DISABLED = 0,
    ENABLED = 1,
};

struct PortData {
    PortConfig config = {};
    fan_ssr::Mode mode = fan_ssr::Mode::FAN;
    State state = State::DISABLED;
    float frequency = 0.0f;
    uint16_t period = 1;
    float duty_cycle = 0.0f;
};

enum class FreqCalculationStatus : uint8_t {
    OK = 0,
    TARGET_FREQ_0 = 1,
    DEVIATION_TOO_LARGE = 2,
    RESOLUTION_TOO_LOW = 3,
};
struct FreqParams {
    FreqCalculationStatus status = FreqCalculationStatus::OK;
    uint16_t prescaler = 0;
    uint16_t period = 0;
    float frequency = 0;
};

enum class BlockInitStatus : uint8_t {
    OK = 0,
    FREQ_OUT_OF_RANGE = 1,
    FREQ_DEVIATION_TOO_LARGE = 2,
    RESOLUTION_TOO_LOW = 3,
    FREQ_TARGET_INVALID = 4,
    HAL_ERR = 5,
};

const PortConfig port_configs[12] = {
    {FAN_SSR1_PIN, BLOCK1_TIMER, FAN_SSR1_CHANNEL},
    {FAN_SSR2_PIN, BLOCK1_TIMER, FAN_SSR2_CHANNEL},
    {FAN_SSR3_PIN, BLOCK1_TIMER, FAN_SSR3_CHANNEL},
    {FAN_SSR4_PIN, BLOCK1_TIMER, FAN_SSR4_CHANNEL},
    {FAN_SSR5_PIN, BLOCK2_TIMER, FAN_SSR5_CHANNEL},
    {FAN_SSR6_PIN, BLOCK2_TIMER, FAN_SSR6_CHANNEL},
    {FAN_SSR7_PIN, BLOCK2_TIMER, FAN_SSR7_CHANNEL},
    {FAN_SSR8_PIN, BLOCK2_TIMER, FAN_SSR8_CHANNEL},
    {FAN_SSR9_PIN, BLOCK3_TIMER, FAN_SSR9_CHANNEL},
    {FAN_SSR10_PIN, BLOCK3_TIMER, FAN_SSR10_CHANNEL},
    {FAN_SSR11_PIN, BLOCK3_TIMER, FAN_SSR11_CHANNEL},
    {FAN_SSR12_PIN, BLOCK3_TIMER, FAN_SSR12_CHANNEL},
};
static PortData port_data[12] = {};
static TIM_HandleTypeDef timers[3] = {{}, {}, {}};
static float fan_off_below = 0.0f;

bool freq_in_range(fan_ssr::Mode mode, float freq) {
    switch (mode) {
        case fan_ssr::Mode::FAN:
            if (freq > FAN_PWM_FREQ_MAX || freq < FAN_PWM_FREQ_MIN) {
                return false;
            } else
                return true;
            break;
        case fan_ssr::Mode::SSR:
            if (freq > SSR_PWM_FREQ_MAX || freq < SSR_PWM_FREQ_MIN) {
                return false;
            } else
                return true;
            break;
    }
}

uint32_t get_timer_clock(TIM_TypeDef* timer);
static FreqParams calculate_frequency_parameters(
    float target_frequency, uint32_t clock_source_frequency);
BlockInitStatus init_block(fan_ssr::Mode mode, float freq, uint8_t block_idx);

fan_ssr::InitStatus fan_ssr::init(Mode block1_mode, float block1_freq,
                                  Mode block2_mode, float block2_freq,
                                  Mode block3_mode, float block3_freq,
                                  float fan_off_below_) {
    fan_off_below = fan_off_below_;
    BlockInitStatus block1_status = init_block(block1_mode, block1_freq, 0);
    switch (block1_status) {
        case BlockInitStatus::OK:
            break;
        case BlockInitStatus::FREQ_OUT_OF_RANGE:
            return InitStatus::BLOCK1_FREQ_OUT_OF_RANGE;
            break;
        case BlockInitStatus::FREQ_DEVIATION_TOO_LARGE:
            return InitStatus::BLOCK1_FREQ_DEVIATION_TOO_LARGE;
            break;
        case BlockInitStatus::RESOLUTION_TOO_LOW:
            return InitStatus::BLOCK1_RESOLUTION_TOO_LOW;
            break;
        case BlockInitStatus::FREQ_TARGET_INVALID:
            return InitStatus::BLOCK1_FREQ_TARGET_INVALID;
            break;
        case BlockInitStatus::HAL_ERR:
            return InitStatus::HAL_ERR;
            break;
    }

    BlockInitStatus block2_status = init_block(block2_mode, block2_freq, 1);
    switch (block2_status) {
        case BlockInitStatus::OK:
            break;
        case BlockInitStatus::FREQ_OUT_OF_RANGE:
            return InitStatus::BLOCK2_FREQ_OUT_OF_RANGE;
            break;
        case BlockInitStatus::FREQ_DEVIATION_TOO_LARGE:
            return InitStatus::BLOCK2_FREQ_DEVIATION_TOO_LARGE;
            break;
        case BlockInitStatus::RESOLUTION_TOO_LOW:
            return InitStatus::BLOCK2_RESOLUTION_TOO_LOW;
            break;
        case BlockInitStatus::FREQ_TARGET_INVALID:
            return InitStatus::BLOCK2_FREQ_TARGET_INVALID;
            break;
        case BlockInitStatus::HAL_ERR:
            return InitStatus::HAL_ERR;
            break;
    }

    BlockInitStatus block3_status = init_block(block3_mode, block3_freq, 2);
    switch (block3_status) {
        case BlockInitStatus::OK:
            break;
        case BlockInitStatus::FREQ_OUT_OF_RANGE:
            return InitStatus::BLOCK3_FREQ_OUT_OF_RANGE;
            break;
        case BlockInitStatus::FREQ_DEVIATION_TOO_LARGE:
            return InitStatus::BLOCK3_FREQ_DEVIATION_TOO_LARGE;
            break;
        case BlockInitStatus::RESOLUTION_TOO_LOW:
            return InitStatus::BLOCK3_RESOLUTION_TOO_LOW;
            break;
        case BlockInitStatus::FREQ_TARGET_INVALID:
            return InitStatus::BLOCK3_FREQ_TARGET_INVALID;
            break;
        case BlockInitStatus::HAL_ERR:
            return InitStatus::HAL_ERR;
            break;
    }

    return InitStatus::OK;
}

bool fan_ssr::enable_port(Port port) {
    uint8_t port_idx = (uint8_t)port;
    uint8_t block_idx = port_idx / 4;
    if (HAL_TIM_PWM_Start(&timers[block_idx],
                          port_data[port_idx].config.channel) != HAL_OK) {
        return false;
    } else
        return true;
}
bool fan_ssr::disable_port(Port port) {
    uint8_t port_idx = (uint8_t)port;
    uint8_t block_idx = port_idx / 4;
    if (HAL_TIM_PWM_Stop(&timers[block_idx],
                         port_data[port_idx].config.channel) != HAL_OK) {
        return false;
    } else
        return true;
}
bool fan_ssr::set_duty_cycle(Port port, float duty_cycle) {
    duty_cycle = std::fabs(duty_cycle);
    if (duty_cycle < 0.0f || duty_cycle > 1.0f) {
        return false;
    }
    if (duty_cycle < fan_off_below) {
        duty_cycle = 0.0f;
    }
    uint8_t port_idx = (uint8_t)port;
    uint8_t block_idx = port_idx / 4;
    uint16_t compare_val = duty_cycle * port_data[port_idx].period;
    if (compare_val > port_data[port_idx].period) {
        compare_val = port_data[port_idx].period;
    }
    __HAL_TIM_SET_COMPARE(&timers[block_idx],
                          port_data[port_idx].config.channel, compare_val);
    return true;
}

BlockInitStatus init_block(fan_ssr::Mode mode, float freq, uint8_t block_idx) {
    // safety checks
    if (!freq_in_range(mode, freq)) {
        return BlockInitStatus::FREQ_OUT_OF_RANGE;
    }
    FreqParams freq_params =
        calculate_frequency_parameters(freq, TIMER_INPUT_FREQ);
    switch (freq_params.status) {
        case FreqCalculationStatus::OK:
            break;
        case FreqCalculationStatus::TARGET_FREQ_0:
            return BlockInitStatus::FREQ_TARGET_INVALID;
            break;
        case FreqCalculationStatus::DEVIATION_TOO_LARGE:
            return BlockInitStatus::FREQ_DEVIATION_TOO_LARGE;
            break;
        case FreqCalculationStatus::RESOLUTION_TOO_LOW:
            return BlockInitStatus::RESOLUTION_TOO_LOW;
            break;
    }

    // initialise timer
    timers[block_idx].Instance = port_configs[4 * block_idx].timer;
    if (timers[block_idx].Instance == TIM4) {
        __HAL_RCC_TIM4_CLK_ENABLE();
    } else if (timers[block_idx].Instance == TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
    } else if (timers[block_idx].Instance == TIM23) {
        __HAL_RCC_TIM23_CLK_ENABLE();
    }
    timers[block_idx].Init.Prescaler = freq_params.prescaler;
    timers[block_idx].Init.Period = freq_params.period;
    timers[block_idx].Init.CounterMode = TIM_COUNTERMODE_UP;
    timers[block_idx].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timers[block_idx].Init.RepetitionCounter = 1;
    timers[block_idx].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_PWM_Init(&timers[block_idx]) != HAL_OK) {
        return BlockInitStatus::HAL_ERR;
    }

    TIM_MasterConfigTypeDef master_config = {};
    master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    master_config.MasterOutputTrigger2 = TIM_TRGO2_RESET;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&timers[block_idx],
                                              &master_config) != HAL_OK) {
        return BlockInitStatus::HAL_ERR;
    }

    TIM_OC_InitTypeDef oc_config = {};
    oc_config.OCMode = TIM_OCMODE_PWM1;          // duty cycle = pulse / period
    oc_config.Pulse = 0;                         // 0 duty cycle initially
    oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;  // high == on
    oc_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;   // high == on
    oc_config.OCFastMode = TIM_OCFAST_DISABLE;      // not used for PWM
    oc_config.OCIdleState = TIM_OCIDLESTATE_RESET;  // high side off by default
    oc_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;  // low side off by default
    for (uint32_t i = 4 * block_idx; i < 4 * block_idx + 4; i++) {
        if (HAL_TIM_OC_ConfigChannel(&timers[block_idx], &oc_config,
                                     port_configs[i].channel) != HAL_OK) {
            return BlockInitStatus::HAL_ERR;
        }
        __HAL_TIM_ENABLE_OCxPRELOAD(&timers[block_idx],
                                    port_configs[i].channel);
        gpio::init(port_configs[i].pin, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
                   gpio::Speed::LOW);
        // store initialised parameters to port data
        port_data[i] = {port_configs[i],    mode,
                        State::DISABLED,    freq_params.frequency,
                        freq_params.period, 0.0f};
    }

    return BlockInitStatus::OK;
}

static FreqParams calculate_frequency_parameters(
    float target_frequency, uint32_t clock_source_frequency) {
    FreqParams params = {};

    // safety checks
    if (target_frequency <= 0.0f || clock_source_frequency == 0) {
        params.status = FreqCalculationStatus::TARGET_FREQ_0;
        return params;
    }

    float total_ticks = (float)clock_source_frequency / target_frequency;

    // 16bit timer
    const float MAX_COUNTS = 65536.0f;

    // target freq too high
    if (total_ticks < (float)PWM_MIN_RESOLUTION) {
        params.status = FreqCalculationStatus::RESOLUTION_TOO_LOW;
        // Best effort: Set PSC to 1 (0 register), use max available ticks
        params.prescaler = 0;
        params.period = (uint32_t)total_ticks - 1;
        params.frequency = clock_source_frequency / (uint32_t)total_ticks;
        return params;
    }

    // 4. Calculate Optimal Prescaler (PSC)
    // We want the smallest integer PSC that allows TotalTicks to fit into
    // MAX_COUNTS PSC_mult = TotalTicks / 65536
    float psc_needed = total_ticks / MAX_COUNTS;

    // Ceiling ensures we don't overflow the period register
    uint32_t psc_mult = (uint32_t)ceilf(psc_needed);

    // Clamp PSC to minimum 1 (div by 1)
    if (psc_mult == 0) psc_mult = 1;

    // 5. Calculate Optimal Period (ARR)
    // Now that we have a hard integer prescaler, calculate the closest period
    // period
    float arr_counts_f = total_ticks / (float)psc_mult;

    // Round to nearest integer for best frequency accuracy
    uint32_t arr_counts = (uint32_t)roundf(arr_counts_f);

    // 6. Edge Case Handling
    // Rounding might have pushed us slightly over the 16-bit limit
    if (arr_counts > 65536) {
        // If we overflowed, we must increase prescaler and recalculate
        psc_mult++;
        arr_counts = (uint32_t)roundf(total_ticks / (float)psc_mult);
    }

    // 7. Assign Registers (0-indexed)
    // PSC register 0 = divide by 1
    // ARR register 99 = 100 counts
    params.prescaler = psc_mult - 1;
    params.period = arr_counts - 1;

    // 8. Calculate Real Achieved Frequency (Float for reporting)
    float real_freq =
        (float)clock_source_frequency / ((float)psc_mult * (float)arr_counts);

    params.frequency = real_freq;

    // Calculate deviation
    float deviation = fabsf(real_freq - target_frequency) / target_frequency;

    if (deviation > PWM_MAX_FREQ_DEVIATION) {
        params.status = FreqCalculationStatus::DEVIATION_TOO_LARGE;
        return params;
    }

    params.status = FreqCalculationStatus::OK;
    return params;
}
