#include "clock.hpp"

#include <stm32h7xx_hal.h>

#include "error.hpp"

void clock::init() {
    RCC_OscInitTypeDef osc_init = {0};
    RCC_ClkInitTypeDef clk_init = {0};

    // 3.3V VCC
    if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK) {
        error::handler();
    }
    // VOS0 for 550MHz
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
    // wait until voltage scaling is ready
    while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
    }

    // HSE (using)
    osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc_init.HSEState = RCC_HSE_ON;
    // PLL1
    osc_init.PLL.PLLState = RCC_PLL_ON;
    osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    /**
     * SYSCLK: 550MHz
     * HSE: 25MHz
     *
     * PLLM = 5 => 25MHz / 5 = 5MHz (Ref)
     * PLLN = 110 => 5MHz * 110 = 550MHz (VCO)
     * PLLP = 1  => 550MHz / 1 = 550MHz (CPU)
     */
    osc_init.PLL.PLLM = 5;
    osc_init.PLL.PLLN = 110;
    osc_init.PLL.PLLP = 1;
    osc_init.PLL.PLLQ = 4;  // 550MHz / 4 = 137.5MHz
    osc_init.PLL.PLLR = 2;  // 550MHz / 2 = 275MHz
    osc_init.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
    osc_init.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    osc_init.PLL.PLLFRACN = 0;

    if (HAL_RCC_OscConfig(&osc_init) != HAL_OK) {
        error::handler();
    }

    // CPU, AHB and APB buses clocks
    clk_init.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |
                         RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
    clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clk_init.SYSCLKDivider = RCC_SYSCLK_DIV1;  // CPU = 550MHz
    clk_init.AHBCLKDivider = RCC_HCLK_DIV2;    // AHB = 275MHz
    clk_init.APB1CLKDivider = RCC_APB1_DIV2;   // APB1 = 137.5MHz
    clk_init.APB2CLKDivider = RCC_APB2_DIV2;   // APB2 = 137.5MHz
    clk_init.APB3CLKDivider = RCC_APB3_DIV2;   // APB3 = 137.5MHz
    clk_init.APB4CLKDivider = RCC_APB4_DIV2;   // APB4 = 137.5MHz

    // VOS0, 275MHz AXI clk, need 3 wait states
    if (HAL_RCC_ClockConfig(&clk_init, FLASH_LATENCY_3) != HAL_OK) {
        error::handler();
    }

    // PLL2 & PLL3
    RCC_PeriphCLKInitTypeDef periph_clk_init;
    periph_clk_init.PeriphClockSelection =
        RCC_PERIPHCLK_ADC | RCC_PERIPHCLK_OSPI | RCC_PERIPHCLK_USART1 |
        RCC_PERIPHCLK_USART10 | RCC_PERIPHCLK_USB;
    /**
     * HSE: 25MHz
     *
     * PLL2:
     * PLLM = 5 => 25MHz / 5 = 5MHz (Ref)
     * PLLN = 64 => 5MHz * 64 = 320MHz (VCO)
     * PLLP = 4  => 320MHz / 4 = 80MHz (ADC clk)
     * PLLQ = 2 => 320MHz / 2 = 160MHz
     * PLLR = 2 => 320MHz / 2 = 160MHz (QSPI clk)
     *
     * PLL3:
     * PLLM = 5 => 25MHz / 5 = 5MHz (Ref)
     * PLLN = 48 => 5MHz * 48 = 240MHz (VCO)
     * PLLP = 2 => 240MHz / 2 = 120MHz
     * PLLQ = 5 => 240MHz / 5 => 48MHz (USB clk)
     * PLLR = 2 => 240MHz / 2 = 120MHz
     */
    periph_clk_init.PLL2.PLL2M = 5;
    periph_clk_init.PLL2.PLL2N = 64;
    periph_clk_init.PLL2.PLL2P = 4;
    periph_clk_init.PLL2.PLL2Q = 2;
    periph_clk_init.PLL2.PLL2R = 2;
    periph_clk_init.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_2;
    periph_clk_init.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
    periph_clk_init.PLL2.PLL2FRACN = 0;
    periph_clk_init.PLL3.PLL3M = 5;
    periph_clk_init.PLL3.PLL3N = 48;
    periph_clk_init.PLL3.PLL3P = 2;
    periph_clk_init.PLL3.PLL3Q = 5;
    periph_clk_init.PLL3.PLL3R = 2;
    periph_clk_init.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_2;
    periph_clk_init.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
    periph_clk_init.PLL3.PLL3FRACN = 0;

    periph_clk_init.Usart16ClockSelection = RCC_USART16910CLKSOURCE_D2PCLK2;
    periph_clk_init.Usart234578ClockSelection =
        RCC_USART234578CLKSOURCE_D2PCLK1;
    periph_clk_init.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
    periph_clk_init.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
    periph_clk_init.OspiClockSelection = RCC_OSPICLKSOURCE_PLL2;

    if (HAL_RCCEx_PeriphCLKConfig(&periph_clk_init) != HAL_OK) {
        error::handler();
    }

    HAL_EnableCompensationCell();
}

extern "C" {
void SystemClock_Config(void) { clock::init(); }
}
