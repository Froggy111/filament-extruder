#include "inverter.hpp"

#include <FreeRTOS.h>
#include <arm_math.h>
#include <stm32g4xx_hal.h>
#include <task.h>

#include <cmath>

#include "config.hpp"
#include "debug.hpp"
#include "error.hpp"
#include "gpio.hpp"

static TIM_HandleTypeDef timer;

static uint16_t resolution = 0;

struct PWMFreqParams {
    uint16_t prescaler = 0;
    uint16_t period = 0;
    uint32_t frequency = 0;
};

static PWMFreqParams timer_init(uint32_t pwm_freq);
static void gpio_init(void);
static PWMFreqParams calculate_frequency_parameters(
    uint32_t target_frequency, uint32_t clock_source_frequency);

// OCP autorecovery
static volatile uint32_t last_ocp_time = 0;
static volatile uint32_t ocp_retry_count = 0;
void ocp_autorecovery_task(void* args);
TaskHandle_t ocp_autorecovery_task_handle;

uint32_t inverter::init(uint32_t pwm_freq) {
    PWMFreqParams pwm_freq_params = timer_init(pwm_freq);
    gpio_init();
    set(0, 0, 0);  // all pull to ground
    xTaskCreate(ocp_autorecovery_task, "OCP autorecovery", 64, nullptr, 15,
                &ocp_autorecovery_task_handle);
    return pwm_freq_params.frequency;
}

void inverter::set(float u, float v, float w) {
    // clamp amplitudes
    u = std::fmax(0.0f, std::fmin(MAX_PWM_ONTIME, u));
    v = std::fmax(0.0f, std::fmin(MAX_PWM_ONTIME, v));
    w = std::fmax(0.0f, std::fmin(MAX_PWM_ONTIME, w));

    // convert amplitudes to duty cycles
    uint16_t u_count = u * resolution;
#if DIRECTION_REVERSED == true
    uint16_t v_count = w * resolution;
    uint16_t w_count = v * resolution;
#elif DIRECTION_REVERSED == false
    uint16_t v_count = v * resolution;
    uint16_t w_count = w * resolution;
#endif

    // set duty cycles
    __HAL_TIM_SET_COMPARE(&timer, U_PHASE_CHANNEL, u_count);
    __HAL_TIM_SET_COMPARE(&timer, V_PHASE_CHANNEL, v_count);
    __HAL_TIM_SET_COMPARE(&timer, W_PHASE_CHANNEL, w_count);
}

constexpr float COS_PI_6 = 0.86602540378f;  // cos(PI/6)
constexpr float TAN_PI_6 = 0.57735026919f;  // tan(PI/6)
constexpr float PI_DIV_3 = M_PI / 3.0f;
constexpr float SEC_PI_6 = 1.0f / COS_PI_6;

const float COS_LOOKUP[6] = {
    1.0f,   // cos(0)
    0.5f,   // cos(-60)
    -0.5f,  // cos(-120)
    -1.0f,  // cos(-180)
    -0.5f,  // cos(-240)
    0.5f    // cos(-300)
};
const float SIN_LOOKUP[6] = {
    0.0f,         // sin(0)
    -0.8660254f,  // sin(-60)
    -0.8660254f,  // sin(-120)
    0.0f,         // sin(-180)
    0.8660254f,   // sin(-240)
    0.8660254f    // sin(-300)
};

const uint8_t SECTOR_MAP[8] = {
    0, 1, 5, 0, 3, 2, 4, 0,
};
// const uint8_t SECTOR_MAP[8] = {0, 0, 2, 1, 4, 5, 3, 0};

inverter::SVPWMData inverter::svpwm_set(float theta, float V_d, float V_q,
                                        float V_dc) {
    return svpwm_set(std::sinf(theta), std::cosf(theta), V_d, V_q, V_dc);
}

inverter::SVPWMData inverter::svpwm_set(float sin_theta, float cos_theta,
                                        float V_d, float V_q, float V_dc) {
    // inverse Park transformation
    float V_alpha = V_d * cos_theta - V_q * sin_theta;
    float V_beta = V_d * sin_theta + V_q * cos_theta;

    // clamp V magnitude
    float V_mag_sq = V_alpha * V_alpha + V_beta * V_beta;
    float V_max = V_dc * COS_PI_6;
    float V_max_sq = V_max * V_max;
    if (V_mag_sq > V_max_sq) {
        float V_mag;
        arm_sqrt_f32(V_mag_sq, &V_mag);
        float scale_factor = V_max / V_mag;
        V_alpha *= scale_factor;
        V_beta *= scale_factor;
    }

    // faster sector finding
    // transform to 3 phase
    float U1 = V_beta;
    float U2 = -0.5f * V_beta + COS_PI_6 * V_alpha;
    float U3 = -0.5f * V_beta - COS_PI_6 * V_alpha;
    // map to sector
    uint8_t n = 0;
    if (U1 > 0.0f) n |= 1;
    if (U2 > 0.0f) n |= 2;
    if (U3 > 0.0f) n |= 4;
    uint8_t sector = SECTOR_MAP[n];

    // rotate V_alpha and V_beta to sectors
    float cos_rot = COS_LOOKUP[sector];
    float sin_rot = SIN_LOOKUP[sector];
    float V_x = V_alpha * cos_rot - V_beta * sin_rot;
    float V_y = V_alpha * sin_rot + V_beta * cos_rot;

    // calculate component voltages and duty cycles
    float inv_V_dc = 1.0f / V_dc;
    float duty_1 = (V_x - V_y * TAN_PI_6) * inv_V_dc;
    float duty_2 = (V_y * SEC_PI_6) * inv_V_dc;
    float duty_zero = (1.0f - duty_1 - duty_2) * 0.5f;

    float duty_U = 0, duty_V = 0, duty_W = 0;

    TargetSector sector_enum = (TargetSector)sector;

    switch (sector_enum) {
        case TargetSector::U:
            duty_U = duty_zero + duty_1 + duty_2;
            duty_V = duty_zero + duty_2;
            duty_W = duty_zero;
            break;
        case TargetSector::UV:
            duty_U = duty_zero + duty_1;
            duty_V = duty_zero + duty_1 + duty_2;
            duty_W = duty_zero;
            break;
        case TargetSector::V:
            duty_U = duty_zero;
            duty_V = duty_zero + duty_1 + duty_2;
            duty_W = duty_zero + duty_2;
            break;
        case TargetSector::VW:
            duty_U = duty_zero;
            duty_V = duty_zero + duty_1;
            duty_W = duty_zero + duty_1 + duty_2;
            break;
        case TargetSector::W:
            duty_U = duty_zero + duty_2;
            duty_V = duty_zero;
            duty_W = duty_zero + duty_1 + duty_2;
            break;
        case TargetSector::WU:
            duty_U = duty_zero + duty_1 + duty_2;
            duty_V = duty_zero;
            duty_W = duty_zero + duty_1;
            break;
    }
    float U_on_time = duty_U - duty_zero, V_on_time = duty_V - duty_zero,
          W_on_time = duty_W - duty_zero;

    set(duty_U, duty_V, duty_W);

    SVPWMData svpwm_data = {U_on_time, V_on_time, W_on_time};

    return svpwm_data;
}

static PWMFreqParams timer_init(uint32_t pwm_freq) {
    // init timer
    timer.Instance = INVERTER_TIMER;
    // assuming APB prescaler = 1
#if INVERTER_CLOCK == PCLK1
    uint32_t clk_freq = HAL_RCC_GetPCLK1Freq();
#elif INVERTER_CLOCK == PCLK2
    uint32_t clk_freq = HAL_RCC_GetPCLK2Freq();
#else
#error "Invalid clock source for inverter timer"
#endif
    // enable timer
    if (INVERTER_TIMER == TIM1) {
        __HAL_RCC_TIM1_CLK_ENABLE();
    } else if (INVERTER_TIMER == TIM8) {
        __HAL_RCC_TIM8_CLK_ENABLE();
    } else if (INVERTER_TIMER == TIM15) {
        __HAL_RCC_TIM15_CLK_ENABLE();
    }

    PWMFreqParams freq_params =
        calculate_frequency_parameters(pwm_freq, clk_freq);
    timer.Init.Prescaler = freq_params.prescaler;
    timer.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
    timer.Init.Period = freq_params.period;
    timer.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timer.Init.RepetitionCounter =
        1;  // interrupt only once per center-aligned cycle
    // changes to period will only apply after the current cycle completes
    timer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    resolution = freq_params.period;

    if (HAL_TIM_PWM_Init(&timer) != HAL_OK) {
        debug::fatal("Inverter: Timer PWM init failed.");
        error::handler();
    }

    TIM_MasterConfigTypeDef master_config = {};

    master_config.MasterOutputTrigger = TIM_TRGO_RESET;
    master_config.MasterOutputTrigger2 = TIM_TRGO2_OC4REF;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&timer, &master_config) !=
        HAL_OK) {
        debug::fatal("Inverter: Master config synchronization failed.");
        error::handler();
    }

    // configure U, V, W channels
    TIM_OC_InitTypeDef oc_config = {};
    oc_config.OCMode = TIM_OCMODE_PWM1;          // duty cycle = pulse / period
    oc_config.Pulse = 0;                         // 0 duty cycle initially
    oc_config.OCPolarity = TIM_OCPOLARITY_HIGH;  // high == on
    oc_config.OCNPolarity = TIM_OCNPOLARITY_HIGH;   // high == on
    oc_config.OCFastMode = TIM_OCFAST_DISABLE;      // not used for PWM
    oc_config.OCIdleState = TIM_OCIDLESTATE_RESET;  // high side off by default
    oc_config.OCNIdleState = TIM_OCNIDLESTATE_RESET;  // low side off by default
    if (HAL_TIM_PWM_ConfigChannel(&timer, &oc_config, U_PHASE_CHANNEL)) {
        debug::fatal("Inverter: U phase PWM channel config failed.");
        error::handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&timer, &oc_config, V_PHASE_CHANNEL)) {
        debug::fatal("Inverter: V phase PWM channel config failed.");
        error::handler();
    }
    if (HAL_TIM_PWM_ConfigChannel(&timer, &oc_config, W_PHASE_CHANNEL)) {
        debug::fatal("Inverter: W phase PWM channel config failed.");
        error::handler();
    }
    // enable preload for changing duty cycle
    __HAL_TIM_ENABLE_OCxPRELOAD(&timer, U_PHASE_CHANNEL);
    __HAL_TIM_ENABLE_OCxPRELOAD(&timer, V_PHASE_CHANNEL);
    __HAL_TIM_ENABLE_OCxPRELOAD(&timer, W_PHASE_CHANNEL);

    TIM_OC_InitTypeDef oc_config_adc = {};
    oc_config_adc.OCMode = TIM_OCMODE_PWM2;
    uint32_t offset_cycles = (uint32_t)std::ceilf(
        (float)ADC_TRIGGER_PRE_OFFSET_TIME * (float)clk_freq / 1e9f);
    // clamp CCR4
    if (offset_cycles > freq_params.period - MIN_TRIGGER_CYCLES) {
        offset_cycles = freq_params.period - MIN_TRIGGER_CYCLES;
    }
    // CCR4
    oc_config_adc.Pulse = freq_params.period - offset_cycles;
    oc_config_adc.OCPolarity = TIM_OCPOLARITY_HIGH;
    oc_config_adc.OCFastMode = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&timer, &oc_config_adc, TIM_CHANNEL_4) !=
        HAL_OK) {
        debug::fatal("Inverter: ADC trigger (CH4) channel config failed.");
        error::handler();
    }

    // configure deadtime
    TIM_BreakDeadTimeConfigTypeDef break_deadtime_config = {};
    break_deadtime_config.OffStateRunMode = TIM_OSSR_DISABLE;  // off by default
    break_deadtime_config.OffStateIDLEMode =
        TIM_OSSI_DISABLE;                                 // off by default
    break_deadtime_config.LockLevel = TIM_LOCKLEVEL_OFF;  // dont lock registers
    break_deadtime_config.DeadTime = (uint32_t)ceilf(
        (float)INVERTER_BREAKTIME *
        ((float)1e9 / (float)clk_freq));  // number of cycles to achieve
                                          // deadtime, rounded up for safety
    // break settings
    break_deadtime_config.BreakState = TIM_BREAK_ENABLE;
    break_deadtime_config.BreakPolarity = TIM_BREAKPOLARITY_HIGH;  // high == on
    break_deadtime_config.BreakFilter =
        8;  // trigger break immediately on break signal, without filter
    break_deadtime_config.BreakAFMode =
        TIM_BREAK_AFMODE_INPUT;  // break acts as an input
    break_deadtime_config.Break2State = TIM_BREAK2_DISABLE;
    break_deadtime_config.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
    break_deadtime_config.Break2Filter = 0;
    break_deadtime_config.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
    // dont automatically restart output on break
    break_deadtime_config.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&timer, &break_deadtime_config)) {
        debug::fatal("Inverter: Timer break and deadtime config failed.");
        error::handler();
    }
    TIMEx_BreakInputConfigTypeDef break_input_config = {};
    break_input_config.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
    break_input_config.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
    break_input_config.Source = OCP_U_PHASE_BREAKINPUTSOURCE;
    if (HAL_TIMEx_ConfigBreakInput(&timer, TIM_BREAKINPUT_BRK,
                                   &break_input_config) != HAL_OK) {
        debug::fatal("Inverter: Timer U phase break input config failed");
        error::handler();
    }
    break_input_config.Source = OCP_V_PHASE_BREAKINPUTSOURCE;
    if (HAL_TIMEx_ConfigBreakInput(&timer, TIM_BREAKINPUT_BRK,
                                   &break_input_config) != HAL_OK) {
        debug::fatal("Inverter: Timer V phase break input config failed");
        error::handler();
    }
    break_input_config.Source = OCP_W_PHASE_BREAKINPUTSOURCE;
    if (HAL_TIMEx_ConfigBreakInput(&timer, TIM_BREAKINPUT_BRK,
                                   &break_input_config) != HAL_OK) {
        debug::fatal("Inverter: Timer W phase break input config failed");
        error::handler();
    }

    // shift the interrupt over to trigger when PWM outputs are low
    HAL_TIM_GenerateEvent(&timer, TIM_EVENTSOURCE_UPDATE);
    __HAL_TIM_SET_COUNTER(&timer, freq_params.period);

    // clear break flags and enable break interrupt (for automatic recovery)
    __HAL_TIM_CLEAR_FLAG(&timer, TIM_FLAG_BREAK);
    __HAL_TIM_CLEAR_FLAG(&timer, TIM_FLAG_BREAK2);
    __HAL_TIM_ENABLE_IT(&timer, TIM_IT_BREAK);
    if (INVERTER_TIMER == TIM1) {
        HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
        HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 5, 0);
    } else if (INVERTER_TIMER == TIM8) {
        HAL_NVIC_EnableIRQ(TIM8_BRK_IRQn);
        HAL_NVIC_SetPriority(TIM8_BRK_IRQn, 5, 0);
    }

    // start PWM
    if (HAL_TIM_PWM_Start(&timer, U_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: Failed to start U phase PWM");
        error::handler();
    }
#ifdef INVERTER_COMPLEMENTARY_PWM
    if (HAL_TIMEx_PWMN_Start(&timer, U_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: Failed to start U_N phase PWM");
        error::handler();
    }
#endif
    if (HAL_TIM_PWM_Start(&timer, V_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: Failed to start V phase PWM");
        error::handler();
    }
#ifdef INVERTER_COMPLEMENTARY_PWM
    if (HAL_TIMEx_PWMN_Start(&timer, V_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: Failed to start V_N phase PWM");
        error::handler();
    }
#endif
    if (HAL_TIM_PWM_Start(&timer, W_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: Failed to start W phase PWM");
        error::handler();
    }
#ifdef INVERTER_COMPLEMENTARY_PWM
    if (HAL_TIMEx_PWMN_Start(&timer, W_PHASE_CHANNEL) != HAL_OK) {
        debug::fatal("Inverter: Failed to start W_N phase PWM");
        error::handler();
    }
#endif

    if (HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_4) != HAL_OK) {
        debug::fatal("Inverter: Failed to start ADC trigger PWM");
        error::handler();
    }

    return freq_params;
}

static void gpio_init(void) {
    gpio::init(INVERTER_U, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
#ifdef INVERTER_COMPLEMENTARY_PWM
    gpio::init(INVERTER_U_N, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
#endif
    gpio::init(INVERTER_V, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
#ifdef INVERTER_COMPLEMENTARY_PWM
    gpio::init(INVERTER_V_N, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
#endif
    gpio::init(INVERTER_W, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
#ifdef INVERTER_COMPLEMENTARY_PWM
    gpio::init(INVERTER_W_N, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
#endif

#ifdef INVERTER_USE_DRIVER_MODE
    gpio::init(INVERTER_DRIVER_MODE, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::write(INVERTER_DRIVER_MODE, INVERTER_DRIVER_MODE_STATE);
#endif

#ifdef INVERTER_USE_PHASE_RESET
    gpio::init(INVERTER_U_RST, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::write(INVERTER_U_RST, INVERTER_RST_STATE);
    gpio::init(INVERTER_V_RST, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::write(INVERTER_V_RST, INVERTER_RST_STATE);
    gpio::init(INVERTER_W_RST, gpio::Mode::OUTPUT_PP, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::write(INVERTER_W_RST, INVERTER_RST_STATE);
#endif
}

/**
 * @brief Calculate the prescaler and period parameters for achieving a
 * target PWM frequency for center-aligned PWM. Maximises ARR for resolution
 * @param target_frequency: Target frequency that will be achieved as closely as
 * possible
 * @param clock_source_frequency: Frequency of the clock source, either PCLK1 or
 * PCLK2 if APB prescalers are 1. TIM2..7 are on PCLK1, TIM1,8,20,15,16,17 are
 * on PCLK2
 * @returns prescaler, period and real frequency values
 */
static PWMFreqParams calculate_frequency_parameters(
    uint32_t target_frequency, uint32_t clock_source_frequency) {
    // Formula: Freq = Clk / ((PSC + 1) * (ARR + 1))
    // (PSC + 1) * (ARR + 1) = Clk / Freq
    // PSC is the prescaler, ARR is the period (autoreload)
    uint32_t divisor = clock_source_frequency / target_frequency;
    PWMFreqParams params = {0, 0, 0};
    // maximum resolution is 16 bits
    // check from highest resolution down
    for (uint32_t i = 16; i >= PWM_MIN_RESOLUTION; i--) {
        uint32_t period = 1 << i;
        // compute prescaler
        uint32_t prescaler = divisor / (period * 2);
        // compute actual frequency
        uint32_t actual_frequency =
            clock_source_frequency / (period * 2 * prescaler);
        float deviation =
            fabsf(((float)actual_frequency - (float)target_frequency)) /
            target_frequency;
        // set the current match
        params.period = period - 1;
        params.prescaler = prescaler - 1;
        params.frequency = actual_frequency;
        debug::debug(
            "PWM frequency calculation: Trying %u bits of resolution, obtained "
            "prescaler %u, and actual frequency %uHz deviating %f from target "
            "%uHz",
            i, prescaler, actual_frequency, deviation, target_frequency);

        // accept this combination
        if (deviation < PWM_MAX_FREQ_DEVIATION) {
            return params;
        }
    }

    debug::warn(
        "PWM frequency generated %uHz is more than %f deviation from target "
        "%uHz.",
        params.frequency, PWM_MAX_FREQ_DEVIATION, target_frequency);
    // accept the latest combination (period = 1 << PWM_MIN_RESOLUTION - 1)
    return params;
}

void inverter::disable_outputs(void) {
    HAL_TIM_PWM_Stop(&timer, U_PHASE_CHANNEL);
    HAL_TIM_PWM_Stop(&timer, V_PHASE_CHANNEL);
    HAL_TIM_PWM_Stop(&timer, W_PHASE_CHANNEL);
#ifdef INVERTER_COMPLEMENTARY_PWM
    HAL_TIMEx_PWMN_Stop(&timer, U_PHASE_CHANNEL);
    HAL_TIMEx_PWMN_Stop(&timer, V_PHASE_CHANNEL);
    HAL_TIMEx_PWMN_Stop(&timer, W_PHASE_CHANNEL);
#endif
    return;
}

void inverter::enable_outputs(void) {
    HAL_TIM_PWM_Start(&timer, U_PHASE_CHANNEL);
    HAL_TIM_PWM_Start(&timer, V_PHASE_CHANNEL);
    HAL_TIM_PWM_Start(&timer, W_PHASE_CHANNEL);
#ifdef INVERTER_COMPLEMENTARY_PWM
    HAL_TIMEx_PWMN_Start(&timer, U_PHASE_CHANNEL);
    HAL_TIMEx_PWMN_Start(&timer, V_PHASE_CHANNEL);
    HAL_TIMEx_PWMN_Start(&timer, W_PHASE_CHANNEL);
#endif
    return;
}

void inverter::timer_irq(void) { HAL_TIM_IRQHandler(&timer); }
void inverter::break_callback(void) {
    last_ocp_time = xTaskGetTickCountFromISR();
    ocp_retry_count++;
    return;
}
bool inverter::is_faulted(void) {
    return !(timer.Instance->BDTR & TIM_BDTR_MOE);
}

void ocp_autorecovery_task([[maybe_unused]] void* args) {
    for (;;) {
        if (!(timer.Instance->BDTR & TIM_BDTR_MOE)) {
            // faulted
            if (ocp_retry_count > OCP_RETRY_LIMIT) {
                debug::fatal(
                    "OCP: Too many retries, permanently locking inverter");
                vTaskDelete(ocp_autorecovery_task_handle);
            }
            if (xTaskGetTickCount() - last_ocp_time > OCP_RETRY_COOLDOWN) {
                __HAL_TIM_CLEAR_FLAG(&timer, TIM_FLAG_BREAK);
                __HAL_TIM_MOE_ENABLE(&timer);
            }
        } else if (xTaskGetTickCount() - last_ocp_time >
                   OCP_STABLE_CLEAR_DURATION) {
            ocp_retry_count = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelete(ocp_autorecovery_task_handle);
}
