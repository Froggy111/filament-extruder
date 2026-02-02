#pragma once

#include "config.hpp"
#if ENCODER_TYPE == ABZ

#include <stm32h7xx_hal.h>

namespace encoder {

void init(void);

int64_t get_count(void);
void set_count(int32_t count);

}  // namespace encoder

#endif
