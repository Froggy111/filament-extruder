#pragma once
namespace clock {

void init(void);

// used in core_interrupts.cpp
void systick_handler(void);

}  // namespace clock
