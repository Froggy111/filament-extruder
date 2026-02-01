#include "stepper.hpp"

#include <FreeRTOS.h>
#include <stm32h7xx_hal.h>
#include <task.h>

#include <cmath>
#include <cstring>

#include "config.hpp"
#include "debug.hpp"
#include "gpio.hpp"

struct HardwareConfig {
    USART_TypeDef* UART = nullptr;
    gpio::PinConfig TX = {};
    gpio::PinConfig RX = {};
    gpio::PinConfig DIR = {};
    gpio::PinConfig STEP = {};
    gpio::PinConfig ENN = {};
    gpio::PinConfig INDEX = {};
    gpio::PinConfig DIAG = {};
    uint8_t address = 0;
    float shunt_resistance = 0.1f;
    TIM_TypeDef* timer = nullptr;
    uint32_t timer_frequency = 0;
};

enum class ReadRegStatus : uint8_t {
    OK,
    REQ_HAL_ERROR,
    RECV_HAL_ERROR,
    RECV_SYNC_MISMATCH,
    RECV_CRC_MISMATCH,
};
struct ReadRegReturn {
    uint32_t data = 0;
    ReadRegStatus status = ReadRegStatus::OK;
};
enum class LoadConfStatus : uint8_t {
    OK,
    WRITE_GCONF_ERR,
    WRITE_NODECONF_ERR,
    WRITE_IHOLD_IRUN_ERR,
    WRITE_TPOWERDOWN_ERR,
    WRITE_TPWMTHRS_ERR,
    WRITE_TCOOLTHRS_ERR,
    WRITE_SGTHRS_ERR,
    WRITE_COOLCONF_ERR,
    WRITE_CHOPCONF_ERR,
    WRITE_PWMCONF_ERR,
    READ_IFCNT_REQ_HAL_ERROR,
    READ_IFCNT_RECV_HAL_ERROR,
    READ_IFCNT_RECV_SYNC_MISMATCH,
    READ_IFCNT_RECV_CRC_MISMATCH,
    IFCNT_MISMATCH,
};

enum class MoveCommandState : uint8_t {
    NONE,
    POSITION,
    VELOCITY,
};
struct StepperState {
    MoveCommandState command_state = MoveCommandState::NONE;
    int32_t current_position = 0;
    int32_t target_position = 0;
    int32_t target_velocity = 0;
    bool step_pin_high = false;
};

const HardwareConfig hw_configs[2] = {
    {STEPPER1_UART, STEPPER1_TX, STEPPER1_RX, STEPPER1_DIR, STEPPER1_STEP,
     STEPPER1_ENN, STEPPER1_INDEX, STEPPER1_DIAG, STEPPER1_ADDR,
     STEPPER1_SHUNT_RESISTANCE, STEPPER1_TIMER, STEPPER1_TIMER_FREQ},
    {STEPPER2_UART, STEPPER2_TX, STEPPER2_RX, STEPPER2_DIR, STEPPER2_STEP,
     STEPPER2_ENN, STEPPER2_INDEX, STEPPER2_DIAG, STEPPER2_ADDR,
     STEPPER2_SHUNT_RESISTANCE, STEPPER1_TIMER, STEPPER1_TIMER_FREQ}};

static stepper::Config configs[2] = {{}, {}};
static StepperState state[2] = {{}, {}};
static UART_HandleTypeDef UARTs[2] = {{}, {}};
static TIM_HandleTypeDef timers[2] = {{}, {}};

// register addresses
namespace reg {

const uint8_t GCONF = 0x00;         // RW
const uint8_t GSTAT = 0x01;         // R+WC
const uint8_t IFCNT = 0x02;         // R
const uint8_t NODECONF = 0x03;      // W
const uint8_t OTP_PROG = 0x04;      // W
const uint8_t OTP_READ = 0x05;      // R
const uint8_t IOIN = 0x06;          // R
const uint8_t FACTORY_CONF = 0x07;  // RW
const uint8_t IHOLD_IRUN = 0x10;    // W
const uint8_t TPOWERDOWN = 0x11;    // W
const uint8_t TSTEP = 0x12;         // R
const uint8_t TPWMTHRS = 0x13;      // W
const uint8_t VACTUAL = 0x22;       // W
const uint8_t TCOOLTHRS = 0x14;     // W
const uint8_t SGTHRS = 0x40;        // W
const uint8_t SG_RESULT = 0x41;     // R
const uint8_t COOLCONF = 0x42;      // W
const uint8_t MSCNT = 0x6A;         // R
const uint8_t MSCURACT = 0x6B;      // R
const uint8_t CHOPCONF = 0x6C;      // RW
const uint8_t DRV_STATUS = 0x6F;    // R
const uint8_t PWMCONF = 0x70;       // RW
const uint8_t PWM_SCALE = 0x71;     // R
const uint8_t PWM_AUTO = 0x72;      // R

// GCONF
const uint8_t vref_selection_bit = 0;
const uint8_t sense_resistor_selection_bit = 1;
const uint8_t stealthchop_spreadcycle_selection_bit = 2;
const uint8_t direction_selection_bit = 3;
const uint8_t index_output_selection_bit = 4;
const uint8_t index_internal_step_selection_bit = 5;
const uint8_t pdn_disable_bit = 6;
const uint8_t microstep_conf_source_select_bit = 7;
const uint8_t multistep_filter_selection_bit = 8;
const uint8_t test_mode_bit = 9;

// GSTAT
const uint32_t reset_mask = 0b1;
const uint32_t driver_error_mask = 0b10;
const uint32_t charge_pump_undervoltage_mask = 0b100;

const uint32_t IFCNT_mask = 0xFF;

// IHOLD_IRUN
const uint8_t IHOLD_bit = 0;
const uint32_t IHOLD_mask = 0b11111 << IHOLD_bit;
const uint8_t IRUN_bit = 8;
const uint32_t IRUN_mask = 0b11111 << IRUN_bit;
const uint8_t IHOLD_delay_bit = 16;
const uint32_t IHOLD_delay_mask = 0b1111 << IHOLD_delay_bit;

const uint32_t TPOWERDOWN_mask = 0xFF;
const uint32_t TPWMTHRS_mask = 0x000FFFFF;
const uint32_t TCOOLTHRS_mask = 0x000FFFFF;
const uint32_t SGTHRS_mask = 0xFF;
const uint32_t SG_RESULT_mask = 0b1111111111;

// COOLCONF
const uint8_t cool_min_sg_val_bit = 0;
const uint32_t cool_min_sg_val_mask = 0xF << cool_min_sg_val_bit;
const uint8_t cool_current_up_step_width_bit = 5;
const uint32_t cool_current_up_step_width_mask =
    0b11 << cool_current_up_step_width_bit;
const uint8_t cool_sg_hysteresis_val_bit = 8;
const uint32_t cool_sg_hysteresis_val_mask = 0xF << cool_sg_hysteresis_val_bit;
const uint8_t cool_current_down_step_speed_bit = 13;
const uint32_t cool_current_down_step_speed_mask =
    0b11 << cool_current_down_step_speed_bit;
const uint8_t cool_min_current_bit = 15;
const uint32_t cool_min_current_mask = 0b1 << cool_min_current_bit;

const uint32_t MSCNT_mask = 0b1111111111;

// MSCURACT
const uint8_t current_B_bit = 0;
const uint32_t current_B_mask = 0b111111111 << current_B_bit;
const uint8_t current_A_bit = 16;
const uint32_t current_A_mask = 0b11111111 << current_A_bit;

// CHOPCONF
const uint8_t toff_bit = 0;
const uint32_t toff_mask = 0xF << toff_bit;
const uint8_t hysteresis_start_val_bit = 4;
const uint32_t hysteresis_start_val_mask = 0b111 << hysteresis_start_val_bit;
const uint8_t hysteresis_low_val_bit = 7;
const uint32_t hysteresis_low_val_mask = 0xF << hysteresis_low_val_bit;
const uint8_t tblank_bit = 15;
const uint32_t tblank_mask = 0b11 << tblank_bit;
const uint8_t vsense_bit = 16;
const uint8_t microstep_bit = 24;
const uint32_t microstep_mask = 0xF << microstep_bit;
const uint8_t ustep_intpol_bit = 28;
const uint8_t double_edge_step_pulses_bit = 29;
const uint8_t short_gnd_prot_disable_bit = 30;
const uint8_t low_side_short_prot_disable_bit = 31;

}  // namespace reg

bool init_driver(uint8_t idx);
bool write_reg(uint8_t idx, uint8_t addr, uint32_t data);
ReadRegReturn read_reg(uint8_t idx, uint8_t addr);
uint16_t usteps_to_val(stepper::MicroSteps usteps);
LoadConfStatus load_config(uint8_t idx);
bool init_timer(uint8_t idx);
void enable_irq(uint8_t idx);
void disable_irq(uint8_t idx);
void irq_handler(uint8_t idx);

stepper::InitStatus stepper::init(Config stepper1_config,
                                  Config stepper2_config) {
    configs[0] = stepper1_config;
    configs[1] = stepper2_config;

    // init UARTs
    if (!init_driver(0)) {
        return InitStatus::STEPPER1_UART_INIT_FAILED;
    }
    if (!init_driver(1)) {
        return InitStatus::STEPPER2_UART_INIT_FAILED;
    }

    // driver settings
    debug::debug("load stepper 1 conf");
    LoadConfStatus load_conf_status = load_config(0);
    switch (load_conf_status) {
        case LoadConfStatus::OK:
            break;
        case LoadConfStatus::WRITE_GCONF_ERR:
            return InitStatus::STEPPER1_WRITE_GCONF_ERR;
            break;
        case LoadConfStatus::WRITE_NODECONF_ERR:
            return InitStatus::STEPPER1_WRITE_NODECONF_ERR;
            break;
        case LoadConfStatus::WRITE_IHOLD_IRUN_ERR:
            return InitStatus::STEPPER1_WRITE_IHOLD_IRUN_ERR;
            break;
        case LoadConfStatus::WRITE_TPOWERDOWN_ERR:
            return InitStatus::STEPPER1_WRITE_TPOWERDOWN_ERR;
            break;
        case LoadConfStatus::WRITE_TPWMTHRS_ERR:
            return InitStatus::STEPPER1_WRITE_TPWMTHRS_ERR;
            break;
        case LoadConfStatus::WRITE_TCOOLTHRS_ERR:
            return InitStatus::STEPPER1_WRITE_TCOOLTHRS_ERR;
            break;
        case LoadConfStatus::WRITE_SGTHRS_ERR:
            return InitStatus::STEPPER1_WRITE_SGTHRS_ERR;
            break;
        case LoadConfStatus::WRITE_COOLCONF_ERR:
            return InitStatus::STEPPER1_WRITE_COOLCONF_ERR;
            break;
        case LoadConfStatus::WRITE_CHOPCONF_ERR:
            return InitStatus::STEPPER1_WRITE_CHOPCONF_ERR;
            break;
        case LoadConfStatus::WRITE_PWMCONF_ERR:
            return InitStatus::STEPPER1_WRITE_PWMCONF_ERR;
            break;
        case LoadConfStatus::READ_IFCNT_REQ_HAL_ERROR:
            return InitStatus::STEPPER1_READ_IFCNT_REQ_HAL_ERROR;
            break;
        case LoadConfStatus::READ_IFCNT_RECV_HAL_ERROR:
            return InitStatus::STEPPER1_READ_IFCNT_RECV_HAL_ERROR;
            break;
        case LoadConfStatus::READ_IFCNT_RECV_SYNC_MISMATCH:
            return InitStatus::STEPPER1_READ_IFCNT_RECV_SYNC_MISMATCH;
            break;
        case LoadConfStatus::READ_IFCNT_RECV_CRC_MISMATCH:
            return InitStatus::STEPPER1_READ_IFCNT_RECV_CRC_MISMATCH;
            break;
        case LoadConfStatus::IFCNT_MISMATCH:
            return InitStatus::STEPPER1_IFCNT_MISMATCH;
    }
    // load_conf_status = load_config(1);
    // switch (load_conf_status) {
    //     case LoadConfStatus::OK:
    //         break;
    //     case LoadConfStatus::WRITE_GCONF_ERR:
    //         return InitStatus::STEPPER2_WRITE_GCONF_ERR;
    //         break;
    //     case LoadConfStatus::WRITE_NODECONF_ERR:
    //         return InitStatus::STEPPER2_WRITE_NODECONF_ERR;
    //         break;
    //     case LoadConfStatus::WRITE_IHOLD_IRUN_ERR:
    //         return InitStatus::STEPPER2_WRITE_IHOLD_IRUN_ERR;
    //         break;
    //     case LoadConfStatus::WRITE_TPOWERDOWN_ERR:
    //         return InitStatus::STEPPER2_WRITE_TPOWERDOWN_ERR;
    //         break;
    //     case LoadConfStatus::WRITE_TPWMTHRS_ERR:
    //         return InitStatus::STEPPER2_WRITE_TPWMTHRS_ERR;
    //         break;
    //     case LoadConfStatus::WRITE_TCOOLTHRS_ERR:
    //         return InitStatus::STEPPER2_WRITE_TCOOLTHRS_ERR;
    //         break;
    //     case LoadConfStatus::WRITE_SGTHRS_ERR:
    //         return InitStatus::STEPPER2_WRITE_SGTHRS_ERR;
    //         break;
    //     case LoadConfStatus::WRITE_COOLCONF_ERR:
    //         return InitStatus::STEPPER2_WRITE_COOLCONF_ERR;
    //         break;
    //     case LoadConfStatus::WRITE_CHOPCONF_ERR:
    //         return InitStatus::STEPPER2_WRITE_CHOPCONF_ERR;
    //         break;
    //     case LoadConfStatus::WRITE_PWMCONF_ERR:
    //         return InitStatus::STEPPER2_WRITE_PWMCONF_ERR;
    //         break;
    //     case LoadConfStatus::READ_IFCNT_REQ_HAL_ERROR:
    //         return InitStatus::STEPPER2_READ_IFCNT_REQ_HAL_ERROR;
    //         break;
    //     case LoadConfStatus::READ_IFCNT_RECV_HAL_ERROR:
    //         return InitStatus::STEPPER2_READ_IFCNT_RECV_HAL_ERROR;
    //         break;
    //     case LoadConfStatus::READ_IFCNT_RECV_SYNC_MISMATCH:
    //         return InitStatus::STEPPER2_READ_IFCNT_RECV_SYNC_MISMATCH;
    //         break;
    //     case LoadConfStatus::READ_IFCNT_RECV_CRC_MISMATCH:
    //         return InitStatus::STEPPER2_READ_IFCNT_RECV_CRC_MISMATCH;
    //         break;
    //     case LoadConfStatus::IFCNT_MISMATCH:
    //         return InitStatus::STEPPER2_IFCNT_MISMATCH;
    // }

    if (!init_timer(0)) {
        return InitStatus::STEPPER1_TIM_INIT_FAILED;
    }
    if (!init_timer(1)) {
        return InitStatus::STEPPER2_TIM_INIT_FAILED;
    }

    return InitStatus::OK;
}

void stepper::enable(Port stepper) {
    uint8_t idx = (uint8_t)stepper;
    gpio::write(hw_configs[idx].ENN, gpio::LOW);
}
void stepper::disable(Port stepper) {
    uint8_t idx = (uint8_t)stepper;
    gpio::write(hw_configs[idx].ENN, gpio::HIGH);
}
void stepper::step(Port stepper) {
    uint8_t idx = (uint8_t)stepper;
    gpio::write(hw_configs[idx].STEP, gpio::HIGH);
#pragma unroll
    for (int i = 0; i < 100; i++) {
        __NOP();
    }
    gpio::write(hw_configs[idx].STEP, gpio::LOW);
}

void stepper::set_velocity(Port stepper, float velocity) {
    uint8_t idx = (uint8_t)stepper;
    float angular_velocity = velocity / configs[idx].rotation_distance;
    set_angular_velocity(stepper, angular_velocity);
    return;
}
void stepper::set_angular_velocity(Port stepper, float angular_velocity) {
    uint8_t idx = (uint8_t)stepper;
    state[idx].target_velocity = angular_velocity *
                                 configs[idx].steps_per_rotation *
                                 usteps_to_val(configs[idx].microsteps);
    state[idx].command_state = MoveCommandState::VELOCITY;
    uint32_t arr_value = hw_configs[idx].timer_frequency /
                         (std::abs(state[idx].target_velocity) * 2);
    if (state[idx].target_velocity > 0.0f) {
        gpio::write(hw_configs[idx].DIR, gpio::HIGH);
    } else {
        gpio::write(hw_configs[idx].DIR, gpio::LOW);
    }
    __HAL_TIM_SET_AUTORELOAD(&timers[idx], arr_value);
    enable_irq(idx);
    return;
}
void stepper::move(Port stepper, float position, float velocity) {
    uint8_t idx = (uint8_t)stepper;
    float angular_velocity = velocity / configs[idx].rotation_distance;
    state[idx].target_velocity = angular_velocity *
                                 configs[idx].steps_per_rotation *
                                 usteps_to_val(configs[idx].microsteps);
    state[idx].target_position = position * configs[idx].steps_per_rotation *
                                 usteps_to_val(configs[idx].microsteps);
    state[idx].command_state = MoveCommandState::POSITION;
    uint32_t arr_value = hw_configs[idx].timer_frequency /
                         (std::abs(state[idx].target_velocity) * 2);
    if (state[idx].target_position > state[idx].current_position) {
        gpio::write(hw_configs[idx].DIR, gpio::HIGH);
    } else if (state[idx].target_position < state[idx].current_position) {
        gpio::write(hw_configs[idx].DIR, gpio::LOW);
    } else {
        return;
    }
    __HAL_TIM_SET_AUTORELOAD(&timers[idx], arr_value);
    enable_irq(idx);
    return;
}
void stepper::stop(Port stepper) {
    uint8_t idx = (uint8_t)stepper;
    state[idx].command_state = MoveCommandState::NONE;
    disable_irq(idx);
    return;
}

bool init_driver(uint8_t idx) {
    if (hw_configs[idx].UART == USART1) {
        __HAL_RCC_USART1_CLK_ENABLE();
    } else if (hw_configs[idx].UART == USART10) {
        __HAL_RCC_USART10_CLK_ENABLE();
    }
    UARTs[idx].Instance = hw_configs[idx].UART;
    UARTs[idx].Init.BaudRate = configs[idx].baud_rate;
    UARTs[idx].Init.WordLength = UART_WORDLENGTH_8B;
    UARTs[idx].Init.StopBits = UART_STOPBITS_1;
    UARTs[idx].Init.Parity = UART_PARITY_NONE;
    UARTs[idx].Init.Mode = UART_MODE_TX_RX;
    UARTs[idx].Init.HwFlowCtl = UART_HWCONTROL_NONE;
    UARTs[idx].Init.OverSampling = UART_OVERSAMPLING_16;
    UARTs[idx].Init.ClockPrescaler = UART_PRESCALER_DIV1;
    UARTs[idx].Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED;
    if (HAL_UART_Init(&UARTs[idx]) != HAL_OK) {
        return false;
    }
    gpio::init(hw_configs[idx].TX, gpio::Mode::AF_PP, gpio::Pull::UP,
               gpio::Speed::VERY_HIGH);
    gpio::init(hw_configs[idx].RX, gpio::Mode::AF_PP, gpio::Pull::UP,
               gpio::Speed::VERY_HIGH);
    gpio::init(hw_configs[idx].DIR, gpio::Mode::OUTPUT_PP_, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::write(hw_configs[idx].DIR, gpio::LOW);
    gpio::init(hw_configs[idx].STEP, gpio::Mode::OUTPUT_PP_, gpio::Pull::NOPULL,
               gpio::Speed::VERY_HIGH);
    gpio::write(hw_configs[idx].STEP, gpio::LOW);
    gpio::init(hw_configs[idx].ENN, gpio::Mode::OUTPUT_PP_, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::write(hw_configs[idx].ENN, gpio::HIGH);
    gpio::init(hw_configs[idx].INDEX, gpio::Mode::INPUT, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::init(hw_configs[idx].DIAG, gpio::Mode::INPUT, gpio::Pull::NOPULL,
               gpio::Speed::LOW);

    return true;
}

LoadConfStatus load_config(uint8_t idx) {
    stepper::Config config = configs[idx];
    // calculate register values
    const float V_FS = 0.18f;
    const float ISENSE_FACTOR =
        std::sqrtf(2) * (hw_configs[idx].shunt_resistance + 0.02f) / V_FS;
    uint32_t IRUN_val = 32 * config.running_current * ISENSE_FACTOR - 1;
    uint32_t IHOLD_val = 32 * config.holding_current * ISENSE_FACTOR - 1;
    const float TMC_CLK = 12e6;
    const float SC_thrs_usteps_per_second = config.stealthchop_rpm_threshold /
                                            60.0f * config.steps_per_rotation *
                                            usteps_to_val(config.microsteps);
    const float CS_thrs_usteps_per_second = config.coolstep_rpm_threshold /
                                            60.0f * config.steps_per_rotation *
                                            usteps_to_val(config.microsteps);
    uint32_t TPWMTHRS_val = TMC_CLK / SC_thrs_usteps_per_second;
    uint32_t TCOOLTHRS_val = TMC_CLK / CS_thrs_usteps_per_second;

    // GCONF
    uint32_t GCONF_val = 0;
    GCONF_val |= 0UL << reg::vref_selection_bit;  // use internal 2.5V reference
    GCONF_val |=
        0UL << reg::sense_resistor_selection_bit;  // use external shunts
    GCONF_val |= (uint32_t)config.chopper_mode
                 << reg::stealthchop_spreadcycle_selection_bit;
    GCONF_val |= (uint32_t)config.direction << reg::direction_selection_bit;
    GCONF_val |= 0UL << reg::index_output_selection_bit;  // output full steps
    GCONF_val |=
        0UL << reg::index_internal_step_selection_bit;  // use index output
                                                        // selection
    GCONF_val |= 1UL << reg::pdn_disable_bit;  // disable PDN_UART input
    GCONF_val |=
        1UL
        << reg::microstep_conf_source_select_bit;  // set microsteps with MRES
    GCONF_val |=
        1UL << reg::multistep_filter_selection_bit;  // use multistep filter
    GCONF_val |= 0UL << reg::test_mode_bit;          // normal operation

    uint32_t NODECONF_val = 15UL << 8;  // 3 * 8 bit send delay

    // IHOLD_IRUN
    uint32_t IHOLD_IRUN_val = 0;
    IHOLD_IRUN_val |= (IHOLD_val << reg::IHOLD_bit) & reg::IHOLD_mask;
    IHOLD_IRUN_val |= (IRUN_val << reg::IRUN_bit) & reg::IRUN_mask;
    IHOLD_IRUN_val |= (1UL << reg::IHOLD_delay_bit) &
                      reg::IHOLD_delay_mask;  // fastest smooth ramp

    uint32_t TPOWERDOWN_val = 255UL;  // highest setting

    // CHOPCONF
    uint32_t CHOPCONF_val = 0;
    CHOPCONF_val |=
        (3UL << reg::toff_bit) & reg::toff_mask;  // stealthchop value
    CHOPCONF_val |= (5UL << reg::hysteresis_start_val_bit) &
                    reg::hysteresis_start_val_mask;  // stealthchop value
    CHOPCONF_val |= (0UL << reg::hysteresis_low_val_bit) &
                    reg::hysteresis_low_val_mask;  // stealthchop value
    CHOPCONF_val |=
        (0UL << reg::tblank_bit) & reg::tblank_mask;  // 16 clock blank time
    CHOPCONF_val |= 1UL << reg::vsense_bit;           // 0.18V V_FS
    CHOPCONF_val |= ((uint32_t)config.microsteps << reg::microstep_bit) &
                    reg::microstep_mask;
    CHOPCONF_val |= 1UL << reg::ustep_intpol_bit;  // use 256ustep interpolation
    CHOPCONF_val |=
        0UL << reg::double_edge_step_pulses_bit;  // regular step pulses
    CHOPCONF_val |=
        0UL << reg::short_gnd_prot_disable_bit;  // enable short to
                                                 // ground protection
    CHOPCONF_val |=
        0UL << reg::low_side_short_prot_disable_bit;  // enable low side short
                                                      // protection

    // TODO : add configuration for coolstep and stallguard
    uint8_t write_count = 0;
    if (!write_reg(idx, reg::GCONF, GCONF_val)) {
        return LoadConfStatus::WRITE_GCONF_ERR;
    }
    debug::debug("read ifcnt reg");
    vTaskDelay(100);
    ReadRegReturn ifcnt_result = read_reg(idx, reg::IFCNT);
    switch (ifcnt_result.status) {
        case ReadRegStatus::OK:
            break;
        case ReadRegStatus::REQ_HAL_ERROR:
            debug::debug("err");
            return LoadConfStatus::READ_IFCNT_REQ_HAL_ERROR;
            break;
        case ReadRegStatus::RECV_HAL_ERROR:
            debug::debug("err");
            return LoadConfStatus::READ_IFCNT_RECV_HAL_ERROR;
            break;
        case ReadRegStatus::RECV_SYNC_MISMATCH:
            debug::debug("err");
            return LoadConfStatus::READ_IFCNT_RECV_SYNC_MISMATCH;
            break;
        case ReadRegStatus::RECV_CRC_MISMATCH:
            debug::debug("err");
            return LoadConfStatus::READ_IFCNT_RECV_CRC_MISMATCH;
            break;
    }
    write_count = ifcnt_result.data & 0xFF;
    debug::debug("read ifcnt reg");

    if (!write_reg(idx, reg::NODECONF, NODECONF_val)) {
        return LoadConfStatus::WRITE_NODECONF_ERR;
    }
    write_count++;
    if (!write_reg(idx, reg::IHOLD_IRUN, IHOLD_IRUN_val)) {
        return LoadConfStatus::WRITE_IHOLD_IRUN_ERR;
    }
    write_count++;
    if (!write_reg(idx, reg::TPOWERDOWN, TPOWERDOWN_val)) {
        return LoadConfStatus::WRITE_TPOWERDOWN_ERR;
    }
    write_count++;
    if (!write_reg(idx, reg::TPWMTHRS, TPWMTHRS_val)) {
        return LoadConfStatus::WRITE_TPWMTHRS_ERR;
    }
    write_count++;
    if (!write_reg(idx, reg::TCOOLTHRS, TCOOLTHRS_val)) {
        return LoadConfStatus::WRITE_TCOOLTHRS_ERR;
    }
    write_count++;
    if (!write_reg(idx, reg::CHOPCONF, CHOPCONF_val)) {
        return LoadConfStatus::WRITE_CHOPCONF_ERR;
    }
    write_count++;

    // verify IFCNT
    ifcnt_result = read_reg(idx, reg::IFCNT);
    switch (ifcnt_result.status) {
        case ReadRegStatus::OK:
            break;
        case ReadRegStatus::REQ_HAL_ERROR:
            return LoadConfStatus::READ_IFCNT_REQ_HAL_ERROR;
            break;
        case ReadRegStatus::RECV_HAL_ERROR:
            return LoadConfStatus::READ_IFCNT_RECV_HAL_ERROR;
            break;
        case ReadRegStatus::RECV_SYNC_MISMATCH:
            return LoadConfStatus::READ_IFCNT_RECV_SYNC_MISMATCH;
            break;
        case ReadRegStatus::RECV_CRC_MISMATCH:
            return LoadConfStatus::READ_IFCNT_RECV_CRC_MISMATCH;
            break;
    }
    if (write_count != (ifcnt_result.data & 0xFF)) {
        return LoadConfStatus::IFCNT_MISMATCH;
    }

    return LoadConfStatus::OK;
}

bool write_reg(uint8_t idx, uint8_t addr, uint32_t data) {
    uint8_t packet[8] = {};
    packet[0] = 0b00000101;               // sync + reserved
    packet[1] = hw_configs[idx].address;  // node address
    packet[2] =
        0b10000000 | (addr & 0b01111111);  // RW + 7 bit register address
    packet[3] = (data >> 24) & 0xFF;
    packet[4] = (data >> 16) & 0xFF;
    packet[5] = (data >> 8) & 0xFF;
    packet[6] = data & 0xFF;
    // calculate CRC
    uint8_t crc = 0;
    for (int i = 0; i < 7; i++) {
        uint8_t current_byte = packet[i];
        for (int j = 0; j < 8; j++) {
            // if MSB of current crc matches LSB of current data byte
            if ((crc >> 7) ^ (current_byte & 0b00000001)) {
                crc = (crc << 1) ^ 0x07;  // XOR with polynomial
            } else {
                crc = (crc << 1);
            }
            current_byte = current_byte >> 1;
        }
    }
    packet[7] = crc;
    if (HAL_HalfDuplex_EnableTransmitter(&UARTs[idx]) != HAL_OK) {
        return false;
    }
    if (HAL_UART_Transmit(&UARTs[idx], packet, sizeof(packet), HAL_MAX_DELAY) !=
        HAL_OK) {
        return false;
    }
    return true;
}

ReadRegReturn read_reg(uint8_t idx, uint8_t addr) {
    ReadRegReturn result = {};
    uint8_t packet[4] = {};
    packet[0] = 0b00000101;  // sync + reserved
    packet[1] = hw_configs[idx].address;
    packet[2] = addr & 0b01111111;  // RW + 7 bit register address
    // calculate CRC
    uint8_t crc = 0;
    for (int i = 0; i < 3; i++) {
        uint8_t current_byte = packet[i];
        for (int j = 0; j < 8; j++) {
            // if MSB of current crc matches LSB of current data byte
            if ((crc >> 7) ^ (current_byte & 0b00000001)) {
                crc = (crc << 1) ^ 0x07;  // XOR with polynomial
            } else {
                crc = (crc << 1);
            }
            current_byte = current_byte >> 1;
        }
    }
    packet[3] = crc;

    // disable receiver
    if (HAL_HalfDuplex_EnableTransmitter(&UARTs[idx]) != HAL_OK) {
        result.status = ReadRegStatus::REQ_HAL_ERROR;
        return result;
    }
    if (HAL_UART_Transmit(&UARTs[idx], packet, sizeof(packet), HAL_MAX_DELAY) !=
        HAL_OK) {
        result.status = ReadRegStatus::REQ_HAL_ERROR;
        return result;
    }

    // enable receiver
    uint8_t rx_buffer[16] = {};

    if (HAL_HalfDuplex_EnableReceiver(&UARTs[idx]) != HAL_OK) {
        result.status = ReadRegStatus::RECV_HAL_ERROR;
        return result;
    }
    volatile HAL_StatusTypeDef status =
        HAL_UART_Receive(&UARTs[idx], rx_buffer, sizeof(rx_buffer), 100);
    if (status != HAL_OK && status != HAL_TIMEOUT) {
        result.status = ReadRegStatus::RECV_HAL_ERROR;
        return result;
    }
    // find sync byte
    int packet_start_idx = -1;
    for (int i = 0; i < 9; i++) {
        if (rx_buffer[i] == 0b00000101 && rx_buffer[i + 1] == 0xFF) {
            packet_start_idx = i;
            break;
        }
    }
    if (packet_start_idx < 0) {
        result.status = ReadRegStatus::RECV_SYNC_MISMATCH;
        return result;
    }
    uint8_t* rx_packet = &rx_buffer[packet_start_idx];
    // verify CRC
    uint8_t rx_crc = 0;
    for (int i = 0; i < 7; i++) {
        uint8_t current_byte = rx_packet[i];
        for (int j = 0; j < 8; j++) {
            // if MSB of current rx_crc matches LSB of current data byte
            if ((rx_crc >> 7) ^ (current_byte & 0b00000001)) {
                rx_crc = (rx_crc << 1) ^ 0x07;  // XOR with polynomial
            } else {
                rx_crc = (rx_crc << 1);
            }
            current_byte = current_byte >> 1;
        }
    }
    if (rx_crc != rx_packet[7]) {
        result.status = ReadRegStatus::RECV_CRC_MISMATCH;
        return result;
    }
    // read ok
    result.status = ReadRegStatus::OK;
    result.data = ((uint32_t)rx_packet[6]) | ((uint32_t)rx_packet[5] << 8) |
                  ((uint32_t)rx_packet[4] << 16) |
                  ((uint32_t)rx_packet[3] << 24);
    return result;
}

uint16_t usteps_to_val(stepper::MicroSteps usteps) {
    switch (usteps) {
        case stepper::MicroSteps::MS256:
            return 256;
            break;
        case stepper::MicroSteps::MS128:
            return 128;
            break;
        case stepper::MicroSteps::MS64:
            return 64;
            break;
        case stepper::MicroSteps::MS32:
            return 32;
            break;
        case stepper::MicroSteps::MS16:
            return 16;
            break;
        case stepper::MicroSteps::MS8:
            return 8;
            break;
        case stepper::MicroSteps::MS4:
            return 4;
            break;
        case stepper::MicroSteps::MS2:
            return 2;
            break;
        case stepper::MicroSteps::FULLSTEP:
            return 1;
            break;
    }
}

bool init_timer(uint8_t idx) {
    uint16_t prescaler = TIMER_INPUT_FREQ / hw_configs[idx].timer_frequency - 1;
    timers[idx].Instance = hw_configs[idx].timer;
    if (timers[idx].Instance == TIM12) {
        __HAL_RCC_TIM12_CLK_ENABLE();
    } else if (timers[idx].Instance == TIM13) {
        __HAL_RCC_TIM13_CLK_ENABLE();
    }
    timers[idx].Init.Prescaler = prescaler;
    timers[idx].Init.Period = 65535;
    timers[idx].Init.CounterMode = TIM_COUNTERMODE_UP;
    timers[idx].Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    timers[idx].Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    if (HAL_TIM_Base_Init(&timers[idx]) != HAL_OK) {
        return false;
    }

    TIM_MasterConfigTypeDef master_config = {};
    master_config.MasterOutputTrigger = TIM_TRGO_ENABLE;
    master_config.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&timers[idx], &master_config) !=
        HAL_OK) {
        return false;
    }

    if (timers[idx].Instance == TIM12) {
        HAL_NVIC_SetPriority(TIM8_BRK_TIM12_IRQn, 0, 0);
    } else if (timers[idx].Instance == TIM13) {
        HAL_NVIC_SetPriority(TIM8_UP_TIM13_IRQn, 0, 0);
    }
    HAL_TIM_Base_Start_IT(&timers[idx]);

    return true;
}

void enable_irq(uint8_t idx) {
    if (timers[idx].Instance == TIM12) {
        HAL_NVIC_EnableIRQ(TIM8_BRK_TIM12_IRQn);
    } else if (timers[idx].Instance == TIM13) {
        HAL_NVIC_EnableIRQ(TIM8_UP_TIM13_IRQn);
    }
    return;
}

void disable_irq(uint8_t idx) {
    if (timers[idx].Instance == TIM12) {
        HAL_NVIC_DisableIRQ(TIM8_BRK_TIM12_IRQn);
    } else if (timers[idx].Instance == TIM13) {
        HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn);
    }
    return;
}

void irq_handler(uint8_t idx) {
    switch (state[idx].command_state) {
        case MoveCommandState::POSITION:
            if (state[idx].current_position == state[0].target_position) {
                state[idx].command_state = MoveCommandState::NONE;
                disable_irq(idx);
                break;
            }
            if (state[idx].step_pin_high) {
                if (state[idx].target_velocity > 0) {
                    state[idx].current_position++;
                } else {
                    state[idx].current_position--;
                }
            }
            gpio::invert(hw_configs[idx].STEP);
            break;
        case MoveCommandState::VELOCITY:
            if (state[idx].step_pin_high) {
                if (state[idx].target_velocity > 0) {
                    state[idx].current_position++;
                } else {
                    state[idx].current_position--;
                }
            }
            gpio::invert(hw_configs[idx].STEP);
            break;
        case MoveCommandState::NONE:
            disable_irq(idx);
            break;
    }
    return;
}

void stepper::stepper1_irq_handler(void) {
    irq_handler(0);
    return;
}
void stepper::stepper1_hal_irq(void) {
    HAL_TIM_IRQHandler(&timers[0]);
    return;
}
void stepper::stepper2_irq_handler(void) {
    irq_handler(1);
    return;
}
void stepper::stepper2_hal_irq(void) {
    HAL_TIM_IRQHandler(&timers[1]);
    return;
}
