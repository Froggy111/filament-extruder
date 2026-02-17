#pragma once

#include <stm32h7xx_hal.h>

namespace heaters {

enum class Zone : uint8_t {
    Z1 = 0,
    Z2 = 1,
    Z3 = 2,
    Z4 = 3,
};

void init(void);
void start(void);
void stop(void);

void set_temp(Zone zone, float temp);
float get_temp(Zone zone);
float get_target_temp(Zone zone);
float get_duty_cycle(Zone zone);
bool is_running(void);
void set_pid(Zone zone, float Kp, float Ki, float Kd);
void get_pid(Zone zone, float *Kp, float *Ki, float *Kd);

void start_autotune(Zone zone, float temp, uint8_t cycles = 8);
bool is_tuning(Zone zone);

}  // namespace heaters
