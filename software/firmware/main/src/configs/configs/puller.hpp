#pragma once

#include "gpio.hpp"
#include "stepper.hpp"

const stepper::Port PULLER_STEPPER = stepper::Port::P2;
const float PULLER_PITCH = 20.0f * M_PI;  // in mm
const stepper::Config PULLER_STEPPER_CONF = {250000,
                                             PULLER_PITCH,
                                             200,
                                             stepper::MicroSteps::MS32,
                                             stepper::Direction::NONINVERTED,
                                             0.2f,
                                             0.2f,
                                             stepper::ChopperMode::spreadcycle,
                                             0.0f,
                                             false,
                                             0,
                                             10000.0f,
                                             0.0f,
                                             0.0f};
const uint32_t PULLER_PERIOD = 100;              // in ms
const float PULLER_STARTING_VELOCITY = 50.0f;    // in mm/s
const uint32_t PULLER_ADJUSTMENT_DELAY = 10000;  // in ms
const float PULLER_ADJUSTMENT_MULTIPLIER = 0.75f;
const uint8_t PULLER_SENSING_WINDOW = 10;  // how many samples to take
const float PULLER_TARGET_DIAMETER = 1.75f;
const float PULLER_TARGET_CROSS_SECTION_AREA =
    (PULLER_TARGET_DIAMETER / 2.0f) * (PULLER_TARGET_DIAMETER / 2.0f) * M_PI;
const float PULLER_DIAMETER_DEADZONE = 0.01f;
