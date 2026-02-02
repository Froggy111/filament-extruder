#pragma once

namespace motor {
bool init(float max_duty_cycle);
void set_duty_cycle(float duty_cycle);
}  // namespace motor
