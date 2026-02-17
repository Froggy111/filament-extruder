#include "heaters.hpp"

#include <FreeRTOS.h>
#include <task.h>

#include <cmath>

#include "PID.hpp"
#include "config.hpp"
#include "fan_ssr.hpp"
#include "thermistor.hpp"

struct HardwareConfig {
    fan_ssr::Port ssr;
    thermistor::Port thermistor;
};

struct AutoTune {
    uint8_t target_cycles = 0;
    uint8_t current_cycles = 0;
    bool is_heating = false;
    float max_temp = 0.0f;
    float min_temp = 0.0f;
    TickType_t last_downwards_crossover = 0;

    float period_sum = 0.0f;
    float amplitude_sum = 0.0f;
};

struct State {
    float duty_cycle = 0.0f;
    float current_temp = 0.0f;
    float target_temp = 0.0f;

    // safety
    float cumulative_error = 0.0f;
    float last_check_temp = 0.0f;
    TickType_t last_check_tick = 0.0f;
    bool check_gain_active = false;

    float Kp = 0.0f;
    float Ki = 0.0f;
    float Kd = 0.0f;

    bool is_autotuning = false;
    AutoTune autotuning = {};
};

const HardwareConfig hwconfigs[4] = {
    {ZONE1_HEATER_SSR, ZONE1_HEATER_THERMISTOR},
    {ZONE2_HEATER_SSR, ZONE2_HEATER_THERMISTOR},
    {ZONE3_HEATER_SSR, ZONE3_HEATER_THERMISTOR},
    {ZONE4_HEATER_SSR, ZONE4_HEATER_THERMISTOR},
};

PID pid[4] = {
    PID(ZONE1_HEATER_PID[0], ZONE1_HEATER_PID[1], ZONE1_HEATER_PID[2],
        HEATER_PID_FREQ),
    PID(ZONE2_HEATER_PID[0], ZONE2_HEATER_PID[1], ZONE2_HEATER_PID[2],
        HEATER_PID_FREQ),
    PID(ZONE3_HEATER_PID[0], ZONE3_HEATER_PID[1], ZONE3_HEATER_PID[2],
        HEATER_PID_FREQ),
    PID(ZONE4_HEATER_PID[0], ZONE4_HEATER_PID[1], ZONE4_HEATER_PID[2],
        HEATER_PID_FREQ),
};

State state[4] = {};

static bool running = false;

void heater_task(void *args);
void reset_safety(heaters::Zone zone, float current_temp);

void heaters::init(void) {
    running = false;
    for (int i = 0; i < 4; i++) {
        pid[i].reset();
        state[i].current_temp = thermistor::read(hwconfigs[i].thermistor);
        state[i].target_temp = 0.0f;
        state[i].duty_cycle = 0.0f;
        state[i].Kp = ZONE1_HEATER_PID[0];
        state[i].Ki = ZONE1_HEATER_PID[1];
        state[i].Kd = ZONE1_HEATER_PID[2];
        reset_safety((Zone)i, state[i].current_temp);
    }
    xTaskCreate(heater_task, "heater task", 1024, NULL, 2, NULL);
    return;
}

void heaters::start(void) {
    for (int i = 0; i < 4; i++) {
        pid[i].reset();
        state[i].current_temp = thermistor::read(hwconfigs[i].thermistor);
        state[i].target_temp = 0.0f;
        state[i].duty_cycle = 0.0f;
        reset_safety((Zone)i, state[i].current_temp);
        fan_ssr::enable_port(hwconfigs[i].ssr);
        fan_ssr::set_duty_cycle(hwconfigs[i].ssr, 0.0f);
    }
    running = true;
    return;
}

void heaters::stop(void) {
    running = false;
    for (int i = 0; i < 4; i++) {
        pid[i].reset();
        state[i].current_temp = thermistor::read(hwconfigs[i].thermistor);
        state[i].target_temp = 0.0f;
        state[i].duty_cycle = 0.0f;
        reset_safety((Zone)i, state[i].current_temp);
        fan_ssr::set_duty_cycle(hwconfigs[i].ssr, 0.0f);
        fan_ssr::disable_port(hwconfigs[i].ssr);
    }
}

void heaters::set_temp(Zone zone, float temp) {
    uint8_t idx = (uint8_t)zone;
    if (state[idx].is_autotuning) {
        state[idx].is_autotuning = false;
    }
    if (std::fabs(state[idx].target_temp - temp) > 1.0f) {
        state[idx].current_temp = thermistor::read(hwconfigs[idx].thermistor);
        reset_safety(zone, state[idx].current_temp);
    }
    state[idx].target_temp = temp;
    pid[idx].set(temp);
    return;
}

float heaters::get_temp(Zone zone) {
    uint8_t idx = (uint8_t)zone;
    return state[idx].current_temp;
}

float heaters::get_target_temp(Zone zone) {
    uint8_t idx = (uint8_t)zone;
    return state[idx].target_temp;
}

float heaters::get_duty_cycle(Zone zone) {
    uint8_t idx = (uint8_t)zone;
    return state[idx].duty_cycle;
}

bool heaters::is_running(void) { return running; }

void heaters::set_pid(Zone zone, float Kp, float Ki, float Kd) {
    uint8_t idx = (uint8_t)zone;
    pid[idx].set_params(Kp, Ki, Kd, HEATER_PID_FREQ);
    state[idx].Kp = Kp;
    state[idx].Ki = Ki;
    state[idx].Kd = Kd;
    return;
}

void heaters::get_pid(Zone zone, float *Kp, float *Ki, float *Kd) {
    uint8_t idx = (uint8_t)zone;
    if (Kp) *Kp = state[idx].Kp;
    if (Ki) *Ki = state[idx].Ki;
    if (Kd) *Kd = state[idx].Kd;
    return;
}

void heaters::start_autotune(Zone zone, float temp, uint8_t cycles) {
    uint8_t idx = (uint8_t)zone;
    if (!running) return;
    state[idx].target_temp = temp;
    state[idx].is_autotuning = true;
    if (cycles < HEATER_TUNING_MIN_CYCLES) {
        cycles = HEATER_TUNING_MIN_CYCLES;
    }
    state[idx].autotuning.target_cycles = cycles;
    state[idx].autotuning.current_cycles = 0;
    state[idx].autotuning.is_heating = false;
    state[idx].autotuning.max_temp = -1000.0f;
    state[idx].autotuning.min_temp = 1000.0f;

    state[idx].autotuning.last_downwards_crossover = xTaskGetTickCount();
    state[idx].autotuning.period_sum = 0.0f;
    state[idx].autotuning.amplitude_sum = 0.0f;

    reset_safety(zone, thermistor::read(hwconfigs[idx].thermistor));
    return;
}

bool heaters::is_tuning(Zone zone) {
    uint8_t idx = (uint8_t)zone;
    return state[idx].is_autotuning;
}

void heater_task([[maybe_unused]] void *args) {
    TickType_t wake_tick = xTaskGetTickCount();
    for (;;) {
        if (!running) {
            vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(HEATER_PID_PERIOD));
            continue;
        }
        for (int i = 0; i < 4; i++) {
            state[i].current_temp = thermistor::read(hwconfigs[i].thermistor);

            // PROTECTIONS
            if (state[i].current_temp < HEATER_MIN_TEMP ||
                state[i].current_temp > HEATER_MAX_TEMP) {
                heaters::stop();
            }

            // autotune
            if (state[i].is_autotuning) {
                reset_safety((heaters::Zone)i, state[i].current_temp);
                float temp = state[i].current_temp;
                float target = state[i].target_temp;
                state[i].autotuning.max_temp =
                    std::fmax(state[i].autotuning.max_temp, temp);
                state[i].autotuning.min_temp =
                    std::fmin(state[i].autotuning.min_temp, temp);
                if (state[i].autotuning.is_heating && temp > target) {
                    // stop heating when crossed above target
                    state[i].autotuning.is_heating = false;
                    state[i].duty_cycle = 0.0f;
                    state[i].autotuning.max_temp = temp;
                } else if (!state[i].autotuning.is_heating && temp < target) {
                    // start heating when crossed below target
                    state[i].autotuning.is_heating = true;
                    state[i].duty_cycle = HEATER_MAX_DUTY_CYCLE;
                    state[i].autotuning.min_temp = temp;
                    TickType_t now = xTaskGetTickCount();
                    state[i].autotuning.current_cycles += 1;

                    float period =
                        (float)pdTICKS_TO_MS(
                            now -
                            state[i].autotuning.last_downwards_crossover) /
                        1000.0f;
                    float amplitude = (state[i].autotuning.max_temp -
                                       state[i].autotuning.min_temp) /
                                      2.0f;
                    state[i].autotuning.last_downwards_crossover = now;

                    // skip a few cycles to stabilise
                    if (state[i].autotuning.current_cycles >
                        HEATER_TUNING_STABILISATION_CYCLES) {
                        state[i].autotuning.period_sum += period;
                        state[i].autotuning.amplitude_sum += amplitude;
                    }

                    state[i].autotuning.max_temp = -1000.0f;
                    state[i].autotuning.min_temp = 1000.0f;

                    if (state[i].autotuning.current_cycles >=
                        state[i].autotuning.target_cycles) {
                        float period_avg = state[i].autotuning.period_sum /
                                           (state[i].autotuning.target_cycles -
                                            HEATER_TUNING_STABILISATION_CYCLES);
                        float amplitude_avg =
                            state[i].autotuning.amplitude_sum /
                            (state[i].autotuning.target_cycles -
                             HEATER_TUNING_STABILISATION_CYCLES);

                        // ultimate gain Ku
                        // Ku = (4 * output_amplitude) / (pi * amplitude)
                        float Ku = (4.0f * HEATER_MAX_DUTY_CYCLE) /
                                   (M_PI * amplitude_avg);
                        // Tyreus-Luyben
                        float Kp_new = 0.45f * Ku;
                        float Ti = 2.2f * period_avg;
                        float Td = period_avg / 6.3f;
                        float Ki_new = Kp_new / Ti;
                        float Kd_new = Kp_new * Td;

                        heaters::set_pid((heaters::Zone)i, Kp_new, Ki_new,
                                         Kd_new);

                        state[i].is_autotuning = false;
                        state[i].duty_cycle = 0.0f;
                        pid[i].reset();
                    }
                }
                fan_ssr::set_duty_cycle(hwconfigs[i].ssr, state[i].duty_cycle);
                continue;
            }

            // normal PID control
            if (state[i].target_temp > 0.0f) {
                float abs_error =
                    std::fabs(state[i].target_temp - state[i].current_temp);
                if (abs_error < HEATER_SAFETY_HYSTERESIS) {
                    state[i].cumulative_error = 0.0f;
                    state[i].last_check_temp = state[i].current_temp;
                    state[i].last_check_tick = xTaskGetTickCount();
                } else {
                    state[i].cumulative_error +=
                        abs_error * HEATER_PID_PERIOD_SECONDS;
                    if (state[i].cumulative_error > HEATER_SAFETY_MAX_ERROR) {
                        heaters::stop();
                        continue;
                    }
                    if (state[i].target_temp > state[i].current_temp) {
                        TickType_t current_tick = xTaskGetTickCount();
                        float elapsed_seconds =
                            (float)(current_tick - state[i].last_check_tick) *
                            (portTICK_PERIOD_MS / 1000.0f);
                        if (elapsed_seconds >= HEATER_SAFETY_CHECK_GAIN_TIME) {
                            if ((state[i].current_temp -
                                 state[i].last_check_temp) >=
                                HEATER_SAFETY_HEATING_GAIN) {
                                state[i].last_check_temp =
                                    state[i].current_temp;
                                state[i].last_check_tick = current_tick;
                            } else {
                                heaters::stop();
                                continue;
                            }
                        }
                    } else {
                        state[i].last_check_temp = state[i].current_temp;
                        state[i].last_check_tick = xTaskGetTickCount();
                    }
                }
            } else {
                reset_safety((heaters::Zone)i, state[i].current_temp);
            }

            // PID control
            pid[i].update(state[i].current_temp);
            state[i].duty_cycle = pid[i].get();
            if (state[i].duty_cycle > HEATER_MAX_DUTY_CYCLE) {
                pid[i].anti_windup(HEATER_MAX_DUTY_CYCLE);
                state[i].duty_cycle = HEATER_MAX_DUTY_CYCLE;
            } else if (state[i].duty_cycle < 0.0f) {
                pid[i].anti_windup(0.0f);
                state[i].duty_cycle = 0.0f;
            }
            // force off if commanded
            if (state[i].target_temp <= 0.0f) {
                state[i].duty_cycle = 0.0f;
            }
            fan_ssr::set_duty_cycle(hwconfigs[i].ssr, state[i].duty_cycle);
        }
        vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(HEATER_PID_PERIOD));
    }
}

void reset_safety(heaters::Zone zone, float current_temp) {
    uint8_t idx = (uint8_t)zone;
    state[idx].cumulative_error = 0.0f;
    state[idx].last_check_temp = current_temp;
    state[idx].last_check_tick = xTaskGetTickCount();
    state[idx].check_gain_active = true;
}
