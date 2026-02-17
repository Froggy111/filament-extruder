#pragma once

namespace spooler {
void init(void);
void home(void);
void start(void);
void stop(void);
bool is_running(void);
float get_position(void);
}  // namespace spooler
