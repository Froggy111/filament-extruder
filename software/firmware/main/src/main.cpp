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
#include "ethernet.hpp"
#include "fan_ssr.hpp"
#include "gpio.hpp"
#include "heaters.hpp"
#include "indicator.hpp"
#include "motor.hpp"
#include "mpu.hpp"
#include "puller.hpp"
#include "spooler.hpp"
#include "stepper.hpp"
#include "thermistor.hpp"
#include "usb.hpp"

void main_task(void *args);
void blink_task(void *args);

gpio::PinConfig LED = {GPIOA, gpio::Pin::PIN15, gpio::AF::NONE};

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

    vTaskDelay(pdMS_TO_TICKS(2000));

    indicator::init();

    fan_ssr::InitStatus fan_ssr_init_status =
        fan_ssr::init(fan_ssr::Mode::SSR, HEATER_PID_FREQ, fan_ssr::Mode::SSR,
                      1.0f, fan_ssr::Mode::FAN, 50.0f, 0.25f);
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
        stepper::init(SPOOLER_STEPPER_CONF, PULLER_STEPPER_CONF);
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

    debug::log("initialising heaters");
    heaters::init();

    debug::log("initialising puller");
    puller::init();

    debug::log("initialising spooler");
    spooler::init();

    debug::log("initialising ethernet");
    // vTaskDelay(2000);
    if (!ethernet::init()) {
        debug::error("ethernet init unsuccessful");
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

    xTaskCreate(blink_task, "blink task", 512, NULL, 1, NULL);
    vTaskDelete(NULL);
}

void blink_task([[maybe_unused]] void *args) {
    for (;;) {
        gpio::invert(LED);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
