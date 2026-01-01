#include "spi.hpp"

#include <stm32g4xx_hal.h>

#include "debug.hpp"
#include "error.hpp"
#include "gpio.hpp"

// INFO : SYSCLK = 160MHz, 100ns = 16 cycles
inline void delay_100ns(void) {
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
    __asm__ __volatile__("nop");
}

void spi::init(Config &spi) {
    debug::debug("Initialising SPI...");
    if (spi.instance == SPI1) {
        debug::debug("Enabling SPI1 clock");
        __HAL_RCC_SPI1_CLK_ENABLE();
    } else if (spi.instance == SPI2) {
        __HAL_RCC_SPI2_CLK_ENABLE();
    } else if (spi.instance == SPI3) {
        __HAL_RCC_SPI3_CLK_ENABLE();
    }
    spi.handle.Instance = spi.instance;
    spi.handle.Init.Mode = (uint32_t)spi.mode;
    spi.handle.Init.Direction = (uint32_t)spi.direction;
    spi.handle.Init.DataSize = SPI_DATASIZE_8BIT;
    spi.handle.Init.CLKPolarity = (uint32_t)spi.polarity;
    spi.handle.Init.CLKPhase = (uint32_t)spi.phase;
    spi.handle.Init.NSS = SPI_NSS_SOFT;
    spi.handle.Init.BaudRatePrescaler = (uint32_t)spi.baud_rate;
    spi.handle.Init.FirstBit = (uint32_t)spi.first_bit;
    spi.handle.Init.TIMode = SPI_TIMODE_DISABLE;
    spi.handle.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    spi.handle.Init.CRCPolynomial = 10;

    if (HAL_SPI_Init(&spi.handle) != HAL_OK) {
        debug::error("SPI initialisation failed.");
        error::handler();
    }

    if (spi.use_dma) {
        if (spi.rx_dma_channel != nullptr) {
            if (spi.rx_dma_channel >= DMA1_Channel1 &&
                spi.rx_dma_channel <= DMA1_Channel6) {
                __HAL_RCC_DMA1_CLK_ENABLE();
                __HAL_RCC_DMAMUX1_CLK_ENABLE();
            }
            if (spi.rx_dma_channel >= DMA2_Channel1 &&
                spi.rx_dma_channel <= DMA2_Channel6) {
                __HAL_RCC_DMA2_CLK_ENABLE();
                __HAL_RCC_DMAMUX1_CLK_SLEEP_ENABLE();
            }
            spi.rx_dma.Instance = spi.rx_dma_channel;
            spi.rx_dma.Init.Request = spi.rx_dma_request;
            spi.rx_dma.Init.Direction = DMA_PERIPH_TO_MEMORY;
            spi.rx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
            spi.rx_dma.Init.MemInc = DMA_MINC_ENABLE;
            spi.rx_dma.Init.PeriphDataAlignment = DMA_MDATAALIGN_BYTE;
            spi.rx_dma.Init.Mode = DMA_NORMAL;
            spi.rx_dma.Init.Priority = DMA_PRIORITY_MEDIUM;
            if (HAL_DMA_Init(&spi.rx_dma) != HAL_OK) {
                debug::error("SPI RX DMA initialisation failed.");
                error::handler();
            }
            if (spi.rx_dma_channel == DMA1_Channel5) {
                HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 2, 0);
                HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
            }
            __HAL_LINKDMA(&spi.handle, hdmarx, spi.rx_dma);
        }
        if (spi.tx_dma_channel != nullptr &&
            spi.direction == Direction::bidirectional) {
            if (spi.tx_dma_channel >= DMA1_Channel1 &&
                spi.tx_dma_channel <= DMA1_Channel6) {
                __HAL_RCC_DMA1_CLK_ENABLE();
                __HAL_RCC_DMAMUX1_CLK_ENABLE();
            }
            if (spi.tx_dma_channel >= DMA2_Channel1 &&
                spi.tx_dma_channel <= DMA2_Channel6) {
                __HAL_RCC_DMA2_CLK_ENABLE();
                __HAL_RCC_DMAMUX1_CLK_SLEEP_ENABLE();
            }
            spi.tx_dma.Instance = spi.tx_dma_channel;
            spi.tx_dma.Init.Request = spi.tx_dma_request;
            spi.tx_dma.Init.Direction = DMA_MEMORY_TO_PERIPH;
            spi.tx_dma.Init.PeriphInc = DMA_PINC_DISABLE;
            spi.tx_dma.Init.MemInc = DMA_MINC_ENABLE;
            spi.tx_dma.Init.PeriphDataAlignment = DMA_MDATAALIGN_BYTE;
            spi.tx_dma.Init.Mode = DMA_NORMAL;
            spi.tx_dma.Init.Priority = DMA_PRIORITY_MEDIUM;
            if (HAL_DMA_Init(&spi.tx_dma) != HAL_OK) {
                debug::error("SPI TX DMA initialisation failed.");
                error::handler();
            }
            __HAL_LINKDMA(&spi.handle, hdmatx, spi.tx_dma);
        }
    }

    if (spi.instance == SPI1) {
        HAL_NVIC_SetPriority(SPI1_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(SPI1_IRQn);
    }

    gpio::init(spi.SCLK, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::VERY_HIGH);
    gpio::init(spi.MISO, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
               gpio::Speed::VERY_HIGH);
    if (spi.direction == Direction::bidirectional) {
        gpio::init(spi.MOSI, gpio::Mode::AF_PP, gpio::Pull::NOPULL,
                   gpio::Speed::VERY_HIGH);
    }
    gpio::init(spi.NCS, gpio::Mode::OUTPUT_PP_, gpio::Pull::UP,
               gpio::Speed::VERY_HIGH);

    debug::debug("SPI initialisation successful.");
    return;
}

bool spi::receive(Config &spi, uint8_t *buffer, uint16_t size,
                  uint32_t timeout) {
    gpio::write(spi.NCS, gpio::LOW);
    delay_100ns();
    HAL_StatusTypeDef status =
        HAL_SPI_Receive(&spi.handle, buffer, size, timeout);
    delay_100ns();
    gpio::write(spi.NCS, gpio::HIGH);
    if (status != HAL_OK) {
        debug::error("SPI receive failed.");
        return false;
    }
    return true;
}

void spi::start_dma_receive(Config &spi, uint8_t *buffer, uint16_t size) {
    if (HAL_SPI_GetState(&spi.handle) == HAL_SPI_STATE_READY) {
        gpio::write(spi.NCS, gpio::LOW);
        if (HAL_SPI_Receive_DMA(&spi.handle, buffer, size) != HAL_OK) {
            gpio::write(spi.NCS, gpio::HIGH);
        };
    }
    return;
}

bool spi::is_dma_receive_done(Config &spi) {
    HAL_SPI_StateTypeDef state = HAL_SPI_GetState(&spi.handle);
    if (state == HAL_SPI_STATE_ERROR) {
        __HAL_SPI_CLEAR_OVRFLAG(&spi.handle);
    }
    return (state == HAL_SPI_STATE_READY) || (state == HAL_SPI_STATE_ERROR);
}

void spi::end_dma_receive(Config &spi) {
    gpio::write(spi.NCS, gpio::HIGH);
    return;
}
