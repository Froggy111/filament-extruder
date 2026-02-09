#include "main.hpp"

#include <FreeRTOS.h>
#include <stdio.h>
#include <stm32h7xx_hal.h>
#include <task.h>

#include <cmath>

#include "clock.hpp"
#include "debug.hpp"
#include "encoder.hpp"
#include "error.hpp"
#include "fan_ssr.hpp"
#include "gpio.hpp"
#include "indicator.hpp"
#include "motor.hpp"
#include "mpu.hpp"
#include "spooler.hpp"
#include "stepper.hpp"
#include "thermistor.hpp"
#include "usb.hpp"

void main_task(void *args);

gpio::PinConfig LED = {GPIOA, gpio::Pin::PIN15, gpio::AF::NONE};

stepper::Config stepper1_conf = {250000,
                                 20,
                                 200,
                                 stepper::MicroSteps::MS32,
                                 stepper::Direction::NONINVERTED,
                                 0.1f,
                                 0.1f,
                                 stepper::ChopperMode::spreadcycle,
                                 300.0f,
                                 false,
                                 0,
                                 10000.0f,
                                 0.0f,
                                 0.0f};
stepper::Config stepper2_conf = stepper1_conf;

int main(void) {
    mpu::init();
    HAL_Init();
    clock::init();

    xTaskCreate(main_task, "main task", 1024, NULL, 1, NULL);
    vTaskStartScheduler();
    // should never reach here
    __disable_irq();
    while (1) {
        // should never reach here
    }
}

void main_task([[maybe_unused]] void *args) {
    gpio::init(LED, gpio::Mode::OUTPUT_PP_, gpio::Pull::NOPULL,
               gpio::Speed::LOW);
    gpio::write(LED, 1);
    usb::init();

    vTaskDelay(pdMS_TO_TICKS(100));

    indicator::init();

    fan_ssr::InitStatus fan_ssr_init_status =
        fan_ssr::init(fan_ssr::Mode::FAN, 50.0f, fan_ssr::Mode::FAN, 50.0f,
                      fan_ssr::Mode::FAN, 50.0f, 0.25f);
    switch (fan_ssr_init_status) {
        case fan_ssr::InitStatus::BLOCK1_FREQ_OUT_OF_RANGE:
        case fan_ssr::InitStatus::BLOCK2_FREQ_OUT_OF_RANGE:
        case fan_ssr::InitStatus::BLOCK3_FREQ_OUT_OF_RANGE:
            debug::error("FAN/SSR frequency out of range");
            break;
        case fan_ssr::InitStatus::BLOCK1_FREQ_DEVIATION_TOO_LARGE:
        case fan_ssr::InitStatus::BLOCK2_FREQ_DEVIATION_TOO_LARGE:
        case fan_ssr::InitStatus::BLOCK3_FREQ_DEVIATION_TOO_LARGE:
            debug::error("FAN/SSR frequency deviation too large");
            break;
        case fan_ssr::InitStatus::BLOCK1_RESOLUTION_TOO_LOW:
        case fan_ssr::InitStatus::BLOCK2_RESOLUTION_TOO_LOW:
        case fan_ssr::InitStatus::BLOCK3_RESOLUTION_TOO_LOW:
            debug::error("FAN/SSR resolution too low");
            break;
        case fan_ssr::InitStatus::BLOCK1_FREQ_TARGET_INVALID:
        case fan_ssr::InitStatus::BLOCK2_FREQ_TARGET_INVALID:
        case fan_ssr::InitStatus::BLOCK3_FREQ_TARGET_INVALID:
            debug::error("FAN/SSR frequency target invalid");
            break;
        case fan_ssr::InitStatus::HAL_ERR:
            debug::error("FAN/SSR HAL error");
            break;
        case fan_ssr::InitStatus::OK:
            debug::debug("FAN/SSR initialisation successful");
            break;
    }
    if (fan_ssr_init_status != fan_ssr::InitStatus::OK) {
        vTaskDelay(5000);
        error::handler();
    }

    stepper::InitStatus stepper_init_status =
        stepper::init(stepper1_conf, SPOOLER_STEPPER_CONF);
    switch (stepper_init_status) {
        case stepper::InitStatus::OK:
            debug::debug("stepper initialisation successful");
            break;
    }
    if (stepper_init_status != stepper::InitStatus::OK) {
        debug::error("stepper initialisation unsuccessful");
        vTaskDelay(5000);
        error::handler();
    }

    encoder::init();

    if (!motor::init(0.5f)) {
        debug::error("motor initialisation unsuccessful");
        vTaskDelay(5000);
        error::handler();
    }

    thermistor::InitStatus therm_init_status = thermistor::init();
    switch (therm_init_status) {
        case thermistor::InitStatus::OK:
            debug::debug("thermistor initialisation successful");
            break;
        case thermistor::InitStatus::ADC1_INIT_FAILED:
            debug::error("thermistor ADC1 initialisation failed");
            break;
        case thermistor::InitStatus::ADC1_CALIBRATION_FAILED:
            debug::error("thermistor ADC1 calibration failed");
            break;
        case thermistor::InitStatus::ADC2_INIT_FAILED:
            debug::error("thermistor ADC2 initialisation failed");
            break;
        case thermistor::InitStatus::ADC2_CALIBRATION_FAILED:
            debug::error("thermistor ADC2 calibration failed");
            break;
        case thermistor::InitStatus::ADC3_INIT_FAILED:
            debug::error("thermistor ADC3 initialisation failed");
            break;
        case thermistor::InitStatus::ADC3_CALIBRATION_FAILED:
            debug::error("thermistor ADC3 calibration failed");
            break;
        case thermistor::InitStatus::CHANNEL_INIT_FAILED:
            debug::error("thermistor channel initialisation failed");
            break;
    }
    if (therm_init_status != thermistor::InitStatus::OK) {
        debug::error("thermistor initialisation unsuccessful");
        vTaskDelay(5000);
        error::handler();
    }

    // if (!fan_ssr::enable_port(fan_ssr::Port::P5)) {
    //     debug::error("FAN/SSR enable port error");
    //     vTaskDelay(5000);
    //     error::handler();
    // } else {
    //     debug::debug("FAN/SSR enable port successful");
    // };
    //
    // stepper::enable(stepper::Port::P1);
    // stepper::enable(stepper::Port::P2);

    // spooler::init();
    // spooler::start();

    float duty_cycle = 0.0f;
    bool up = true;
    for (;;) {
        if (up) {
            duty_cycle += 0.01f;
        } else {
            duty_cycle -= 0.01f;
        }
        if (duty_cycle > 0.2f) {
            duty_cycle -= 0.01f;
            up = false;
        } else if (duty_cycle < -0.2f) {
            duty_cycle += 0.01f;
            up = true;
        }
        gpio::invert(LED);
        // fan_ssr::set_duty_cycle(fan_ssr::Port::P5, 0.0f);
        // HAL_Delay(1);
        debug::debug("Hello World!");
        // int64_t encoder_reading = encoder::get_count();
        // debug::log("encoder count: %lld", encoder_reading);
        // float indicator_reading = indicator::read();
        // debug::log("indicator reading: %f", indicator_reading);
        // uint32_t start_cycles = DWT->CYCCNT;
        // float thermistor_reading = thermistor::read(thermistor::Port::P1);
        // uint32_t end_cycles = DWT->CYCCNT;
        // uint32_t total_cycles = end_cycles - start_cycles;
        // uint32_t cpu_freq = HAL_RCC_GetSysClockFreq();
        // float time_us = ((float)total_cycles * 1.0e6f) / cpu_freq;
        // debug::log("thermistor reading: %f, time taken: %fus, cpu freq: %u",
        //            thermistor_reading, time_us, cpu_freq);
        // motor::set_duty_cycle(0.49f);
        // stepper::set_angular_velocity(stepper::Port::P1, duty_cycle
        // / 5.0f); stepper::set_angular_velocity(stepper::Port::P2,
        // duty_cycle);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
