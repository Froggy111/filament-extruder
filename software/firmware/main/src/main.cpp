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

void MPU_Config(void) {
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    /* Disables the MPU */
    HAL_MPU_Disable();

    /** Initializes and configures the Region and the memory to be protected
     */
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    /* Enables the MPU */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

int main(void) {
    MPU_Config();
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
