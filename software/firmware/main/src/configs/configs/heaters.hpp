#pragma once

#include "fan_ssr.hpp"
#include "thermistor.hpp"

const fan_ssr::Port ZONE1_HEATER_SSR = fan_ssr::Port::P4;
const thermistor::Port ZONE1_HEATER_THERMISTOR = thermistor::Port::P8;
const fan_ssr::Port ZONE2_HEATER_SSR = fan_ssr::Port::P3;
const thermistor::Port ZONE2_HEATER_THERMISTOR = thermistor::Port::P7;
const fan_ssr::Port ZONE3_HEATER_SSR = fan_ssr::Port::P2;
const thermistor::Port ZONE3_HEATER_THERMISTOR = thermistor::Port::P6;
const fan_ssr::Port ZONE4_HEATER_SSR = fan_ssr::Port::P1;
const thermistor::Port ZONE4_HEATER_THERMISTOR = thermistor::Port::P5;

const uint32_t HEATER_PID_FREQ = 5;
const uint32_t HEATER_PID_PERIOD = 1000 / HEATER_PID_FREQ;  // in ms
const float HEATER_PID_PERIOD_SECONDS = HEATER_PID_PERIOD / 1000.0f;

const float ZONE1_HEATER_PID[3] = {9.55f, 6.78f, 0.97f};
const float ZONE2_HEATER_PID[3] = {0.0f, 0.0f, 0.0f};
const float ZONE3_HEATER_PID[3] = {0.0f, 0.0f, 0.0f};
const float ZONE4_HEATER_PID[3] = {0.0f, 0.0f, 0.0f};
const float HEATER_MIN_TEMP = 0.0f;
const float HEATER_MAX_TEMP = 450.0f;
const float HEATER_MAX_DUTY_CYCLE = 1.0f;
const float HEATER_SAFETY_MAX_ERROR = 100000.0f;
const float HEATER_SAFETY_CHECK_GAIN_TIME = 60.0f;
const float HEATER_SAFETY_HYSTERESIS = 5.0f;
const float HEATER_SAFETY_HEATING_GAIN = 2.0f;

const uint8_t HEATER_TUNING_STABILISATION_CYCLES = 3;
const uint8_t HEATER_TUNING_MIN_CYCLES = 5;
