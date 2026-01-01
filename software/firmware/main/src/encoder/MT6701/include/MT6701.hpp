#pragma once

#include "config.hpp"
#if ENCODER_TYPE == MT6701

#include <stm32g4xx_hal.h>

namespace encoder {

void init(void);

/**
 * @brief Get summed count
 * @returns summed count
 */
int64_t get_count(void);
void start_DMA_read(void);
// blocks until reading is ready
int64_t get_DMA_reading(void);

// used in core_interrupts.cpp
void DMA_RX_handler(void);
void SPI_handler(void);
void rx_complete_callback(void);

}  // namespace encoder

#endif
