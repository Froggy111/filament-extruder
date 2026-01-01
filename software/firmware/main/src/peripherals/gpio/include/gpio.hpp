#pragma once

#include <stdint.h>
#include <stm32h7xx_hal.h>

namespace gpio {

enum class Pin : uint16_t;
enum class Mode : uint32_t;
enum class Pull : uint32_t;
enum class Speed : uint32_t;
enum class AF : uint8_t;

constexpr bool LOW = false;
constexpr bool HIGH = true;

struct PinConfig {
    GPIO_TypeDef *port;
    Pin pin;
    AF alternate_function;
};

using InterruptFn = void (*)(void *);

void init(const PinConfig &pin_config, Mode mode, Pull pull, Speed speed);

void attach_interrupt(const PinConfig &pin_config, Mode mode, Pull pull,
                      Speed speed, InterruptFn callback, void *args);

inline bool read(const PinConfig &pin_config) {
    return (bool)HAL_GPIO_ReadPin(pin_config.port, (uint16_t)pin_config.pin);
}
inline void write(const PinConfig &pin_config, bool state) {
    HAL_GPIO_WritePin(pin_config.port, (uint16_t)pin_config.pin,
                      (GPIO_PinState)state);
}
inline void invert(const PinConfig &pin_config) {
    HAL_GPIO_TogglePin(pin_config.port, (uint16_t)pin_config.pin);
}

enum class Pin : uint16_t {
    PIN0 = GPIO_PIN_0,
    PIN1 = GPIO_PIN_1,
    PIN2 = GPIO_PIN_2,
    PIN3 = GPIO_PIN_3,
    PIN4 = GPIO_PIN_4,
    PIN5 = GPIO_PIN_5,
    PIN6 = GPIO_PIN_6,
    PIN7 = GPIO_PIN_7,
    PIN8 = GPIO_PIN_8,
    PIN9 = GPIO_PIN_9,
    PIN10 = GPIO_PIN_10,
    PIN11 = GPIO_PIN_11,
    PIN12 = GPIO_PIN_12,
    PIN13 = GPIO_PIN_13,
    PIN14 = GPIO_PIN_14,
    PIN15 = GPIO_PIN_15,
    ALL = GPIO_PIN_All
};

enum class Mode : uint32_t {
    INPUT = GPIO_MODE_INPUT, /*!< Input Floating Mode                   */
    OUTPUT_PP_ = GPIO_MODE_OUTPUT_PP, /*!< Output Push Pull Mode */
    OUTPUT_OD_ = GPIO_MODE_OUTPUT_OD, /*!< Output Open Drain Mode */
    AF_PP = GPIO_MODE_AF_PP,   /*!< Alternate Function Push Pull Mode     */
    AF_OD = GPIO_MODE_AF_OD,   /*!< Alternate Function Open Drain Mode    */
    ANALOG = GPIO_MODE_ANALOG, /*!< Analog Mode  */
    INT_RISING = GPIO_MODE_IT_RISING,   /*!< External Interrupt Mode with
                                           Rising   edge trigger detection   */
    INT_FALLING = GPIO_MODE_IT_FALLING, /*!< External Interrupt Mode with
                                           Falling edge trigger detection */
    INT_RISING_FALLING =
        GPIO_MODE_IT_RISING_FALLING,     /*!< External Interrupt Mode with
                                            Rising/Falling edge trigger
                                            detection   */
    EVT_RISING = GPIO_MODE_EVT_RISING,   /*!< External Event Mode with Rising
                                            edge trigger detection */
    EVT_FALLING = GPIO_MODE_EVT_FALLING, /*!< External Event Mode with Falling
                                            edge trigger detection */
    EVT_RISING_FALLING =
        GPIO_MODE_EVT_RISING_FALLING /*!< External Event Mode with
                                        Rising/Falling edge trigger
                                        detection */
};

enum class Pull : uint32_t {
    DOWN = GPIO_PULLDOWN,
    UP = GPIO_PULLUP,
    NOPULL = GPIO_NOPULL
};

enum class Speed : uint32_t {
    LOW = GPIO_SPEED_FREQ_LOW,
    MEDIUM = GPIO_SPEED_FREQ_MEDIUM,
    HIGH = GPIO_SPEED_FREQ_HIGH,
#ifdef GPIO_SPEED_FREQ_VERY_HIGH
    VERY_HIGH = GPIO_SPEED_FREQ_VERY_HIGH,
#else
#error GPIO_SPEED_FREQ_VERY_HIGH not available.
#endif
};

// STM32H7xx specific Alternate Function mappings
#ifdef STM32H7
enum class AF : uint8_t {
    NONE = 0xFF,

    /**
     * @brief   AF 0 selection
     */
    AF0_RTC_50Hz = 0x00, /* RTC_50Hz Alternate Function mapping */
    AF0_MCO = 0x00,      /* MCO (MCO1 and MCO2) Alternate Function mapping */
    AF0_SWJ = 0x00,      /* SWJ (SWD and JTAG) Alternate Function mapping */
    AF0_LCDBIAS = 0x00,  /* LCDBIAS Alternate Function mapping */
    AF0_TRACE = 0x00,    /* TRACE Alternate Function mapping */
#if defined(PWR_CPUCR_RETDS_CD)
    AF0_CSLEEP = 0x00,  /* CSLEEP Alternate Function mapping */
    AF0_CSTOP = 0x00,   /* CSTOP Alternate Function mapping */
    AF0_NDSTOP2 = 0x00, /* NDSTOP2 Alternate Function mapping */
#endif                  /* PWR_CPUCR_RETDS_CD */
#if defined(PWR_CPUCR_PDDS_D2)
    AF0_C1DSLEEP = 0x00, /* Cortex-M7 Deep Sleep Alternate Function mapping */
    AF0_C1SLEEP = 0x00,  /* Cortex-M7 Sleep Alternate Function mapping */
    AF0_D1PWREN = 0x00,  /* Domain 1 PWR enable Alternate Function mapping */
    AF0_D2PWREN = 0x00,  /* Domain 2 PWR enable Alternate Function mapping */
#if defined(DUAL_CORE)
    AF0_C2DSLEEP = 0x00, /* Cortex-M4 Deep Sleep Alternate Function mapping */
    AF0_C2SLEEP = 0x00,  /* Cortex-M4 Sleep Alternate Function mapping */
#endif                   /* DUAL_CORE */
#endif                   /* PWR_CPUCR_PDDS_D2 */

    /**
     * @brief   AF 1 selection
     */
    AF1_TIM1 = 0x01,   /* TIM1 Alternate Function mapping */
    AF1_TIM2 = 0x01,   /* TIM2 Alternate Function mapping */
    AF1_TIM16 = 0x01,  /* TIM16 Alternate Function mapping */
    AF1_TIM17 = 0x01,  /* TIM17 Alternate Function mapping */
    AF1_LPTIM1 = 0x01, /* LPTIM1 Alternate Function mapping */
#if defined(HRTIM1)
    AF1_HRTIM1 = 0x01, /* HRTIM1 Alternate Function mapping */
#endif                 /* HRTIM1 */
#if defined(SAI4)
    AF1_SAI4 = 0x01, /* SAI4 Alternate Function mapping */
#endif               /* SAI4 */
    AF1_FMC = 0x01,  /* FMC Alternate Function mapping */

    /**
     * @brief   AF 2 selection
     */
    AF2_TIM3 = 0x02,  /* TIM3 Alternate Function mapping */
    AF2_TIM4 = 0x02,  /* TIM4 Alternate Function mapping */
    AF2_TIM5 = 0x02,  /* TIM5 Alternate Function mapping */
    AF2_TIM12 = 0x02, /* TIM12 Alternate Function mapping */
    AF2_SAI1 = 0x02,  /* SAI1 Alternate Function mapping */
#if defined(HRTIM1)
    AF2_HRTIM1 = 0x02, /* HRTIM1 Alternate Function mapping */
#endif                 /* HRTIM1 */
    AF2_TIM15 = 0x02,  /* TIM15 Alternate Function mapping */
#if defined(FDCAN3)
    AF2_FDCAN3 = 0x02, /* FDCAN3 Alternate Function mapping */
#endif                 /*FDCAN3*/

    /**
     * @brief   AF 3 selection
     */
    AF3_TIM8 = 0x03,   /* TIM8 Alternate Function mapping */
    AF3_LPTIM2 = 0x03, /* LPTIM2 Alternate Function mapping */
    AF3_DFSDM1 = 0x03, /* DFSDM Alternate Function mapping */
    AF3_LPTIM3 = 0x03, /* LPTIM3 Alternate Function mapping */
    AF3_LPTIM4 = 0x03, /* LPTIM4 Alternate Function mapping */
    AF3_LPTIM5 = 0x03, /* LPTIM5 Alternate Function mapping */
    AF3_LPUART = 0x03, /* LPUART Alternate Function mapping */
#if defined(OCTOSPIM)
    AF3_OCTOSPIM_P1 =
        0x03, /* OCTOSPI Manager Port 1 Alternate Function mapping */
    AF3_OCTOSPIM_P2 =
        0x03, /* OCTOSPI Manager Port 2 Alternate Function mapping */
#endif        /* OCTOSPIM */
#if defined(HRTIM1)
    AF3_HRTIM1 = 0x03, /* HRTIM1 Alternate Function mapping */
#endif                 /* HRTIM1 */
    AF3_LTDC = 0x03,   /* LTDC Alternate Function mapping */

    /**
     * @brief   AF 4 selection
     */
    AF4_I2C1 = 0x04, /* I2C1 Alternate Function mapping */
    AF4_I2C2 = 0x04, /* I2C2 Alternate Function mapping */
    AF4_I2C3 = 0x04, /* I2C3 Alternate Function mapping */
    AF4_I2C4 = 0x04, /* I2C4 Alternate Function mapping */
#if defined(I2C5)
    AF4_I2C5 = 0x04,   /* I2C5 Alternate Function mapping */
#endif                 /* I2C5*/
    AF4_TIM15 = 0x04,  /* TIM15 Alternate Function mapping */
    AF4_CEC = 0x04,    /* CEC Alternate Function mapping */
    AF4_LPTIM2 = 0x04, /* LPTIM2 Alternate Function mapping */
    AF4_USART1 = 0x04, /* USART1 Alternate Function mapping */
#if defined(USART10)
    AF4_USART10 = 0x04, /* USART10 Alternate Function mapping */
#endif                  /*USART10*/
    AF4_DFSDM1 = 0x04,  /* DFSDM  Alternate Function mapping */
#if defined(DFSDM2_BASE)
    AF4_DFSDM2 = 0x04, /* DFSDM2 Alternate Function mapping */
#endif                 /* DFSDM2_BASE */
    AF4_DCMI = 0x04,   /* DCMI Alternate Function mapping */
#if defined(PSSI)
    AF4_PSSI = 0x04, /* PSSI Alternate Function mapping */
#endif               /* PSSI */
#if defined(OCTOSPIM)
    AF4_OCTOSPIM_P1 =
        0x04, /* OCTOSPI Manager Port 1 Alternate Function mapping */
#endif        /* OCTOSPIM */

    /**
     * @brief   AF 5 selection
     */
    AF5_SPI1 = 0x05, /* SPI1 Alternate Function mapping */
    AF5_SPI2 = 0x05, /* SPI2 Alternate Function mapping */
    AF5_SPI3 = 0x05, /* SPI3 Alternate Function mapping */
    AF5_SPI4 = 0x05, /* SPI4 Alternate Function mapping */
    AF5_SPI5 = 0x05, /* SPI5 Alternate Function mapping */
    AF5_SPI6 = 0x05, /* SPI6 Alternate Function mapping */
    AF5_CEC = 0x05,  /* CEC  Alternate Function mapping */
#if defined(FDCAN3)
    AF5_FDCAN3 = 0x05, /* FDCAN3 Alternate Function mapping */
#endif                 /*FDCAN3*/

    /**
     * @brief   AF 6 selection
     */
    AF6_SPI2 = 0x06, /* SPI2 Alternate Function mapping */
    AF6_SPI3 = 0x06, /* SPI3 Alternate Function mapping */
    AF6_SAI1 = 0x06, /* SAI1 Alternate Function mapping */
    AF6_I2C4 = 0x06, /* I2C4 Alternate Function mapping */
#if defined(I2C5)
    AF6_I2C5 = 0x06,   /* I2C5 Alternate Function mapping */
#endif                 /* I2C5*/
    AF6_DFSDM1 = 0x06, /* DFSDM Alternate Function mapping */
    AF6_UART4 = 0x06,  /* UART4 Alternate Function mapping */
#if defined(DFSDM2_BASE)
    AF6_DFSDM2 = 0x06, /* DFSDM2 Alternate Function mapping */
#endif                 /* DFSDM2_BASE */
#if defined(SAI3)
    AF6_SAI3 = 0x06, /* SAI3 Alternate Function mapping */
#endif               /* SAI3 */
#if defined(OCTOSPIM)
    AF6_OCTOSPIM_P1 =
        0x06, /* OCTOSPI Manager Port 1 Alternate Function mapping */
#endif        /* OCTOSPIM */

    /**
     * @brief   AF 7 selection
     */
    AF7_SPI2 = 0x07,   /* SPI2 Alternate Function mapping */
    AF7_SPI3 = 0x07,   /* SPI3 Alternate Function mapping */
    AF7_SPI6 = 0x07,   /* SPI6 Alternate Function mapping */
    AF7_USART1 = 0x07, /* USART1 Alternate Function mapping */
    AF7_USART2 = 0x07, /* USART2 Alternate Function mapping */
    AF7_USART3 = 0x07, /* USART3 Alternate Function mapping */
    AF7_USART6 = 0x07, /* USART6 Alternate Function mapping */
    AF7_UART7 = 0x07,  /* UART7 Alternate Function mapping */
    AF7_SDMMC1 = 0x07, /* SDMMC1 Alternate Function mapping */

    /**
     * @brief   AF 8 selection
     */
    AF8_SPI6 = 0x08, /* SPI6 Alternate Function mapping */
#if defined(SAI2)
    AF8_SAI2 = 0x08,   /* SAI2 Alternate Function mapping */
#endif                 /*SAI2*/
    AF8_UART4 = 0x08,  /* UART4 Alternate Function mapping */
    AF8_UART5 = 0x08,  /* UART5 Alternate Function mapping */
    AF8_UART8 = 0x08,  /* UART8 Alternate Function mapping */
    AF8_SPDIF = 0x08,  /* SPDIF Alternate Function mapping */
    AF8_LPUART = 0x08, /* LPUART Alternate Function mapping */
    AF8_SDMMC1 = 0x08, /* SDMMC1 Alternate Function mapping */
#if defined(SAI4)
    AF8_SAI4 = 0x08, /* SAI4 Alternate Function mapping */
#endif               /* SAI4 */

    /**
     * @brief   AF 9 selection
     */
    AF9_FDCAN1 = 0x09, /* FDCAN1 Alternate Function mapping */
    AF9_FDCAN2 = 0x09, /* FDCAN2 Alternate Function mapping */
    AF9_TIM13 = 0x09,  /* TIM13 Alternate Function mapping */
    AF9_TIM14 = 0x09,  /* TIM14 Alternate Function mapping */
    AF9_SDMMC2 = 0x09, /* SDMMC2 Alternate Function mapping */
    AF9_LTDC = 0x09,   /* LTDC Alternate Function mapping */
    AF9_SPDIF = 0x09,  /* SPDIF Alternate Function mapping */
    AF9_FMC = 0x09,    /* FMC Alternate Function mapping */
#if defined(QUADSPI)
    AF9_QUADSPI = 0x09, /* QUADSPI Alternate Function mapping */
#endif                  /* QUADSPI */
#if defined(SAI4)
    AF9_SAI4 = 0x09, /* SAI4 Alternate Function mapping */
#endif               /* SAI4 */
#if defined(OCTOSPIM)
    AF9_OCTOSPIM_P1 =
        0x09, /* OCTOSPI Manager Port 1 Alternate Function mapping */
    AF9_OCTOSPIM_P2 =
        0x09, /* OCTOSPI Manager Port 2 Alternate Function mapping */
#endif        /* OCTOSPIM */

/**
 * @brief   AF 10 selection
 */
#if defined(SAI2)
    AF10_SAI2 = 0x0A,   /* SAI2 Alternate Function mapping */
#endif                  /*SAI2*/
    AF10_SDMMC2 = 0x0A, /* SDMMC2 Alternate Function mapping */
#if defined(USB2_OTG_FS)
    AF10_OTG2_FS = 0x0A, /* OTG2_FS Alternate Function mapping */
#endif                   /*USB2_OTG_FS*/
    AF10_COMP1 = 0x0A,   /* COMP1 Alternate Function mapping */
    AF10_COMP2 = 0x0A,   /* COMP2 Alternate Function mapping */
#if defined(LTDC)
    AF10_LTDC = 0x0A,     /* LTDC Alternate Function mapping */
#endif                    /*LTDC*/
    AF10_CRS_SYNC = 0x0A, /* CRS Sync Alternate Function mapping */
#if defined(QUADSPI)
    AF10_QUADSPI = 0x0A, /* QUADSPI Alternate Function mapping */
#endif                   /* QUADSPI */
#if defined(SAI4)
    AF10_SAI4 = 0x0A, /* SAI4 Alternate Function mapping */
#endif                /* SAI4 */
#if !defined(USB2_OTG_FS)
    AF10_OTG1_FS = 0x0A, /* OTG1_FS Alternate Function mapping */
#endif                   /* !USB2_OTG_FS */
    AF10_OTG1_HS = 0x0A, /* OTG1_HS Alternate Function mapping */
#if defined(OCTOSPIM)
    AF10_OCTOSPIM_P1 =
        0x0A,         /* OCTOSPI Manager Port 1 Alternate Function mapping */
#endif                /* OCTOSPIM */
    AF10_TIM8 = 0x0A, /* TIM8 Alternate Function mapping */
    AF10_FMC = 0x0A,  /* FMC Alternate Function mapping */

    /**
     * @brief   AF 11 selection
     */
    AF11_SWP = 0x0B,    /* SWP Alternate Function mapping */
    AF11_MDIOS = 0x0B,  /* MDIOS Alternate Function mapping */
    AF11_UART7 = 0x0B,  /* UART7 Alternate Function mapping */
    AF11_SDMMC2 = 0x0B, /* SDMMC2 Alternate Function mapping */
    AF11_DFSDM1 = 0x0B, /* DFSDM1 Alternate Function mapping */
    AF11_COMP1 = 0x0B,  /* COMP1 Alternate Function mapping */
    AF11_COMP2 = 0x0B,  /* COMP2 Alternate Function mapping */
    AF11_TIM1 = 0x0B,   /* TIM1 Alternate Function mapping */
    AF11_TIM8 = 0x0B,   /* TIM8 Alternate Function mapping */
    AF11_I2C4 = 0x0B,   /* I2C4 Alternate Function mapping */
#if defined(DFSDM2_BASE)
    AF11_DFSDM2 = 0x0B, /* DFSDM2 Alternate Function mapping */
#endif                  /* DFSDM2_BASE */
#if defined(USART10)
    AF11_USART10 = 0x0B, /* USART10 Alternate Function mapping */
#endif                   /* USART10 */
#if defined(UART9)
    AF11_UART9 = 0x0B, /* UART9 Alternate Function mapping */
#endif                 /* UART9 */
#if defined(ETH)
    AF11_ETH = 0x0B, /* ETH Alternate Function mapping */
#endif               /* ETH */
#if defined(LTDC)
    AF11_LTDC = 0x0B, /* LTDC Alternate Function mapping */
#endif                /*LTDC*/
#if defined(OCTOSPIM)
    AF11_OCTOSPIM_P1 =
        0x0B, /* OCTOSPI Manager Port 1 Alternate Function mapping */
#endif        /* OCTOSPIM */

    /**
     * @brief   AF 12 selection
     */
    AF12_FMC = 0x0C,    /* FMC Alternate Function mapping */
    AF12_SDMMC1 = 0x0C, /* SDMMC1 Alternate Function mapping */
    AF12_MDIOS = 0x0C,  /* MDIOS Alternate Function mapping */
    AF12_COMP1 = 0x0C,  /* COMP1 Alternate Function mapping */
    AF12_COMP2 = 0x0C,  /* COMP2 Alternate Function mapping */
    AF12_TIM1 = 0x0C,   /* TIM1 Alternate Function mapping */
    AF12_TIM8 = 0x0C,   /* TIM8 Alternate Function mapping */
#if defined(LTDC)
    AF12_LTDC = 0x0C, /* LTDC Alternate Function mapping */
#endif                /*LTDC*/
#if defined(USB2_OTG_FS)
    AF12_OTG1_FS = 0x0C, /* OTG1_FS Alternate Function mapping */
#endif                   /* USB2_OTG_FS */
#if defined(OCTOSPIM)
    AF12_OCTOSPIM_P1 =
        0x0C, /* OCTOSPI Manager Port 1 Alternate Function mapping */
#endif        /* OCTOSPIM */

    /**
     * @brief   AF 13 selection
     */
    AF13_DCMI = 0x0D,  /* DCMI Alternate Function mapping */
    AF13_COMP1 = 0x0D, /* COMP1 Alternate Function mapping */
    AF13_COMP2 = 0x0D, /* COMP2 Alternate Function mapping */
#if defined(LTDC)
    AF13_LTDC = 0x0D, /* LTDC Alternate Function mapping */
#endif                /*LTDC*/
#if defined(DSI)
    AF13_DSI = 0x0D, /* DSI Alternate Function mapping */
#endif               /* DSI */
#if defined(PSSI)
    AF13_PSSI = 0x0D, /* PSSI Alternate Function mapping */
#endif                /* PSSI */
    AF13_TIM1 = 0x0D, /* TIM1 Alternate Function mapping */
#if defined(TIM23)
    AF13_TIM23 = 0x0D, /* TIM23 Alternate Function mapping */
#endif                 /*TIM23*/

    /**
     * @brief   AF 14 selection
     */
    AF14_LTDC = 0x0E,  /* LTDC Alternate Function mapping */
    AF14_UART5 = 0x0E, /* UART5 Alternate Function mapping */
#if defined(TIM24)
    AF14_TIM24 = 0x0E, /* TIM24 Alternate Function mapping */
#endif                 /*TIM24*/

    /**
     * @brief   AF 15 selection
     */
    AF15_EVENTOUT = 0x0F /* EVENTOUT Alternate Function mapping */
};
#endif

}  // namespace gpio
