#pragma once

#include <math.h>
#include <stdint.h>

#include "configs/encoder.hpp"

// #define USE_ENDSTOP

#define DIRECTION_REVERSED false

const uint32_t PWM_FREQUENCY = 24000;
const uint32_t NUM_WINDING_SETS = 11;
const float COIL_TO_COIL_RESISTANCE = 4.8;  // in ohms
// delta windings
const float COIL_RESISTANCE = COIL_TO_COIL_RESISTANCE / 2.0f;  // in ohms
const float COIL_RESISTANCE_INV = 1 / COIL_RESISTANCE;
const float MOTOR_INDUCTANCE = 2.6e-3f;       // 2.6 mH
const float MOTOR_KV = 61 * (2 * M_PI / 60);  // radians per second per volt
const float BACK_EMF_CONSTANT = 1 / MOTOR_KV;
const float MOTOR_FRICTION = 0.005f;  // friction torque, in Nm

const float BACK_EMF_COMPENSATION_THRESHOLD_VOLTAGE = 0.1f;
const float BACK_EMF_COMPENSATION_THRESHOLD_VELOCITY =
    BACK_EMF_COMPENSATION_THRESHOLD_VOLTAGE / BACK_EMF_CONSTANT;

// 24khz torque, 3khz velocity, 1khz position
const uint8_t FOC_CYCLES_PER_VELOCITY_LOOP = 8;
const uint8_t FOC_CYCLES_PER_POSITION_LOOP = 24;

const float CURRENT_RESPONSE_FREQUENCY = 500;
const float CURRENT_CUTOFF_FREQUENCY = 8000;
const float CURRENT_SAMPLING_PERIOD = 1.0f / PWM_FREQUENCY;
const float CURRENT_LOWPASS_ALPHA =
    (2.0f * M_PI * CURRENT_SAMPLING_PERIOD * CURRENT_CUTOFF_FREQUENCY) /
    (2.0f * M_PI * CURRENT_SAMPLING_PERIOD * CURRENT_CUTOFF_FREQUENCY + 1);

const float VELOCITY_PLL_BANDWIDTH = 200;
const float VELOCITY_SAMPLING_PERIOD = 1.0f / PWM_FREQUENCY;

const float ENCODER_RADIANS_PER_PULSE = (2.0f * M_PI) / ENCODER_RESOLUTION;

const float ELECTRICAL_OFFSET = 0.0f * M_PI / 180.0f;  // shouldnt be needed

// CALIBRATION SETTINGS
const float ZERO_ENCODER_SWEEP_VOLTAGE = 3.0f;
const float ZERO_ENCODER_SWEEP_STEPS = 100;
const float ZERO_ENCODER_SWEEP_INCREMENT =
    (2.0f * M_PI) / ZERO_ENCODER_SWEEP_STEPS;
const float ZERO_ENCODER_HOLD_VOLTAGE = 6.0f;
const float ZERO_ENCODER_HOLD_TIME = 500;  // in ms
const float ZERO_ENCODER_CRASH_REVERSE_THETA_INCREMENT = M_PI / 180.0f;

const uint16_t ENCODER_NONLINEARITY_LUT_SIZE = 1 << 10;
const uint16_t ENCODER_NONLINEARITY_LUT_INCREMENT =
    (ENCODER_RESOLUTION + 1) / ENCODER_NONLINEARITY_LUT_SIZE;
const float ENCODER_NONLINEARITY_HOLD_VOLTAGE = 3.0f;
const float ENCODER_NONLINEARITY_HOLD_TIME = 50;  // in ms
const float ENCODER_NONLINEARITY_WINDOW_RADIUS = 16;

const uint16_t MOTOR_COGGING_LUT_SIZE = 1 << 10;
const float MOTOR_COGGING_LUT_INCREMENT = (2 * M_PI) / MOTOR_COGGING_LUT_SIZE;
const float MOTOR_COGGING_INITIAL_HOLD_TIME = 5000;  // in ms
const float MOTOR_COGGING_HOLD_TIME = 200;           // in ms
const float MOTOR_COGGING_WINDOW_RADIUS = 1;

// linear parameters
// NOTE : comment this out to not use linear motion
// #define USE_LINEAR_MOTION
#ifdef USE_LINEAR_MOTION
const float ROTATION_DISTANCE = 20.0f;  // 20mm per rotation
const float DISTANCE_PER_RADIAN = ROTATION_DISTANCE / (2.0f * M_PI);
const float RADIANS_PER_DISTANCE = 1 / DISTANCE_PER_RADIAN;

// mm/s
const float ZERO_POSITION_LINEAR_VELOCITY = 100;
const float ZERO_POSITION_ANGULAR_VELOCITY =
    ZERO_POSITION_LINEAR_VELOCITY / DISTANCE_PER_RADIAN;
const float ZERO_ENDSTOP_LINEAR_POSITION = -5;
const float ZERO_ENDSTOP_ANGULAR_POSITION =
    ZERO_ENDSTOP_LINEAR_POSITION / DISTANCE_PER_RADIAN;
#endif

// PID
// current is PI
// direct-axis current (magnetic flux)
const float CURRENT_D_KP = 1;
const float CURRENT_D_KI = 1;
// quadrature-axis current (torque generating)
const float CURRENT_Q_KP = 1;
const float CURRENT_Q_KI = 1;
// velocity is PI/PID
const float VELOCITY_KP = 0.2;
const float VELOCITY_KI = 0;
const float VELOCITY_KD = 0;
// position is P/PI/PID
const float POSITION_KP = 0;
const float POSITION_KI = 0;
const float POSITION_KD = 0;

// OCP
const uint32_t OCP_RETRY_LIMIT = 5;
const uint32_t OCP_RETRY_COOLDOWN = 100;          // in ms
const uint32_t OCP_STABLE_CLEAR_DURATION = 2000;  // in ms

// limits and defaults
const float OCP_MAX_CURRENT = 8.0f;
const float OCP_MIN_CURRENT = 0.0f;
const float OCP_DEFAULT_CURRENT = 3.0f;
