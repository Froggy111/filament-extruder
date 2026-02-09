#pragma once

#include "gpio.hpp"
#include "stepper.hpp"

const gpio::PinConfig SPOOLER_BACK_ENDSTOP = {GPIOA, gpio::Pin::PIN10,
                                              gpio::AF::NONE};
const gpio::PinConfig SPOOLER_FRONT_ENDSTOP = {GPIOA, gpio::Pin::PIN9,
                                               gpio::AF::NONE};
const stepper::Port SPOOLER_STEPPER = stepper::Port::P2;
const float SPOOLER_PITCH = 1.0f;   // in mm
const float SPOOL_WIDTH = 60.0f;    // in mm
const float SPOOLER_OFFSET = 5.0f;  // in mm
const stepper::Config SPOOLER_STEPPER_CONF = {250000,
                                              1,
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

const float SPOOLER_HOME_VELOCITY = 1.0f;  // in mm/s
const float SPOOLER_VELOCITY = 2.0f;       // in mm/s
const float SPOOLER_DUTY_CYCLE = 0.5f;
