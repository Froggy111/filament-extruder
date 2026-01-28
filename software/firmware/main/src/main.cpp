#include "main.hpp"

#include <FreeRTOS.h>
#include <stdio.h>
#include <stm32h7xx_hal.h>
#include <task.h>

#include <cmath>

#include "clock.hpp"
#include "debug.hpp"
#include "error.hpp"
#include "fan_ssr.hpp"
#include "gpio.hpp"
#include "mpu.hpp"
#include "usb.hpp"

void main_task(void *args);

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

    vTaskDelay(pdMS_TO_TICKS(100));

    fan_ssr::InitStatus fan_ssr_init_status =
        fan_ssr::init(fan_ssr::Mode::FAN, 50.0f, fan_ssr::Mode::FAN, 50.0f,
                      fan_ssr::Mode::FAN, 50.0f);
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

    if (!fan_ssr::enable_port(fan_ssr::Port::P5)) {
        debug::error("FAN/SSR enable port error");
        vTaskDelay(5000);
        error::handler();
    } else {
        debug::debug("FAN/SSR enable port successful");
    };
    fan_ssr::set_duty_cycle(fan_ssr::Port::P5, 1.0f);
    vTaskDelay(500);

    float duty_cycle = 0.0f;
    bool up = true;
    for (;;) {
        if (up) {
            duty_cycle += 0.05f;
        } else {
            duty_cycle -= 0.05f;
        }
        if (duty_cycle > 1.0f) {
            duty_cycle -= 0.05f;
            up = false;
        } else if (duty_cycle < 0.0f) {
            duty_cycle += 0.05f;
            up = true;
        }
        gpio::invert(LED);
        fan_ssr::set_duty_cycle(fan_ssr::Port::P5, 1.0f);
        // HAL_Delay(1);
        debug::debug("Hello World!");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
