#include <stm32h7xx_hal.h>

#include "clock.hpp"
#include "config.hpp"
#include "encoder.hpp"
#include "stepper.hpp"

extern "C" {

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_HS;

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void) {
    while (1) {
    }
}

/**
 * @brief This function handles Hard fault interrupt.
 */
void HardFault_Handler(void) {
    while (1) {
    }
}

/**
 * @brief This function handles Memory management fault.
 */
void MemManage_Handler(void) {
    while (1) {
    }
}

/**
 * @brief This function handles Prefetch fault, memory access fault.
 */
void BusFault_Handler(void) {
    while (1) {
    }
}

/**
 * @brief This function handles Undefined instruction or illegal state.
 */
void UsageFault_Handler(void) {
    while (1) {
    }
}

/**
 * @brief This function handles Debug monitor.
 */
void DebugMon_Handler(void) {}

/******************************************************************************/
/* STM32G4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32g4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles USB low priority interrupt remap.
 */
// void USB_LP_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_FS); }
void OTG_HS_IRQHandler(void) { HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS); }

/**
 * @brief This function handles TIM2 Systick interrupt.
 */
void TIM2_IRQHandler(void) { clock::systick_handler(); }

void TIM8_BRK_TIM12_IRQHandler(void) {
    if (STEPPER1_TIMER == TIM12) {
        stepper::stepper1_hal_irq();
    }
    return;
}
void TIM8_UP_TIM13_IRQHandler(void) {
    if (STEPPER2_TIMER == TIM13) {
        stepper::stepper2_hal_irq();
    }
    return;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer) {
    if (timer->Instance == TIM2) {
        HAL_IncTick();
    } else if (timer->Instance == STEPPER1_TIMER) {
        stepper::stepper1_irq_handler();
    } else if (timer->Instance == STEPPER2_TIMER) {
        stepper::stepper2_irq_handler();
    }
    return;
}

/**
 * @brief This function handles EXTI line0 interrupt.
 */
void EXTI0_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0); }

/**
 * @brief This function handles EXTI line1 interrupt.
 */
void EXTI1_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1); }

/**
 * @brief This function handles EXTI line2 interrupt.
 */
void EXTI2_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2); }

/**
 * @brief This function handles EXTI line3 interrupt.
 */
void EXTI3_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3); }

/**
 * @brief This function handles EXTI line4 interrupt.
 */
void EXTI4_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4); }

/**
 * @brief This function handles EXTI line[9:5] interrupts.
 */
void EXTI9_5_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5); }

/**
 * @brief This function handles EXTI line[15:10] interrupts.
 */
void EXTI15_10_IRQHandler(void) { HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10); }
}
