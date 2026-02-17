#pragma once

namespace puller {
void init(void);
void start(void);
void stop(void);
void set_velocity(float velocity);
bool is_running(void);
float get_diameter(void);
float get_filtered_diameter(void);
float get_flow_rate(void);
float get_velocity(void);
}  // namespace puller
