#pragma once

#include <stm32h7xx_hal.h>

namespace stepper {

enum class ChopperMode : uint8_t {
    stealthchop = 0,
    spreadcycle = 1,
};
enum class MicroSteps : uint8_t {
    MS256 = 0b0000,
    MS128 = 0b0001,
    MS64 = 0b0010,
    MS32 = 0b0011,
    MS16 = 0b0100,
    MS8 = 0b0101,
    MS4 = 0b0110,
    MS2 = 0b0111,
    FULLSTEP = 0b1000,
};
enum class Direction : uint8_t {
    NONINVERTED = 0,
    INVERTED = 1,
};

struct Config {
    uint32_t baud_rate = 500000;
    float rotation_distance = 0.0f;
    uint32_t steps_per_rotation = 200;
    MicroSteps microsteps = MicroSteps::MS256;
    Direction direction = Direction::NONINVERTED;
    float running_current = 0.0f;
    float holding_current = 0.0f;
    ChopperMode chopper_mode = ChopperMode::stealthchop;
    float stealthchop_rpm_threshold = 0.0f;
    bool use_stallguard = false;
    uint32_t stallguard_threshold = 10;
    float coolstep_rpm_threshold = 0.0f;
    float coolstep_smartenergy_min = 0.0f;
    float coolstep_smartenergy_max = 0.0f;
};

enum class Port : uint8_t {
    P1 = 0,
    P2 = 1,
};

enum class InitStatus : uint8_t {
    OK,
    STEPPER1_UART_INIT_FAILED,
    STEPPER2_UART_INIT_FAILED,
    STEPPER1_WRITE_GCONF_ERR,
    STEPPER2_WRITE_GCONF_ERR,
    STEPPER1_WRITE_NODECONF_ERR,
    STEPPER2_WRITE_NODECONF_ERR,
    STEPPER1_WRITE_IHOLD_IRUN_ERR,
    STEPPER2_WRITE_IHOLD_IRUN_ERR,
    STEPPER1_WRITE_TPOWERDOWN_ERR,
    STEPPER2_WRITE_TPOWERDOWN_ERR,
    STEPPER1_WRITE_TPWMTHRS_ERR,
    STEPPER2_WRITE_TPWMTHRS_ERR,
    STEPPER1_WRITE_TCOOLTHRS_ERR,
    STEPPER2_WRITE_TCOOLTHRS_ERR,
    STEPPER1_WRITE_SGTHRS_ERR,
    STEPPER2_WRITE_SGTHRS_ERR,
    STEPPER1_WRITE_COOLCONF_ERR,
    STEPPER2_WRITE_COOLCONF_ERR,
    STEPPER1_WRITE_CHOPCONF_ERR,
    STEPPER2_WRITE_CHOPCONF_ERR,
    STEPPER1_WRITE_PWMCONF_ERR,
    STEPPER2_WRITE_PWMCONF_ERR,
    STEPPER1_READ_IFCNT_REQ_HAL_ERROR,
    STEPPER2_READ_IFCNT_REQ_HAL_ERROR,
    STEPPER1_READ_IFCNT_RECV_HAL_ERROR,
    STEPPER2_READ_IFCNT_RECV_HAL_ERROR,
    STEPPER1_READ_IFCNT_RECV_SYNC_MISMATCH,
    STEPPER2_READ_IFCNT_RECV_SYNC_MISMATCH,
    STEPPER1_READ_IFCNT_RECV_CRC_MISMATCH,
    STEPPER2_READ_IFCNT_RECV_CRC_MISMATCH,
    STEPPER1_IFCNT_MISMATCH,
    STEPPER2_IFCNT_MISMATCH,
};

InitStatus init(Config stepper1_config, Config stepper2_config);

void enable(Port stepper);
void disable(Port stepper);
void step(Port stepper);

}  // namespace stepper
