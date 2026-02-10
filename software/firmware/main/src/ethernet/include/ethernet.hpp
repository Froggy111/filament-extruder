#pragma once

namespace ethernet {
bool init(void);

// used in core_interrupts.cpp
void irq_handler(void);
}  // namespace ethernet
