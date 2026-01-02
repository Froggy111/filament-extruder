#include "main.hpp"

#include <FreeRTOS.h>
#include <stdio.h>
#include <stm32h7xx_hal.h>
#include <task.h>

#include <cmath>

#include "clock.hpp"
#include "debug.hpp"
#include "gpio.hpp"
#include "usb.hpp"

void main_task(void *args);

gpio::PinConfig LED = {GPIOA, gpio::Pin::PIN15, gpio::AF::NONE};

int main(void) {
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

    for (;;) {
        gpio::invert(LED);
        // HAL_Delay(1);
        debug::debug("Hello World!");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
