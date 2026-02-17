#include "puller.hpp"

#include <FreeRTOS.h>
#include <string.h>
#include <task.h>

#include <cmath>

#include "config.hpp"
#include "indicator.hpp"
#include "stepper.hpp"

static float current_filament_diameter = 0.0f;
static float filament_diameter[PULLER_SENSING_WINDOW] = {};
static float filtered_diameter = 0.0f;
static float current_velocity = 0.0f;
static float current_volume_flow = 0.0f;
static bool running = false;
TickType_t adjustment_wait_tick;

void puller_task(void* args);
float get_median_diameter(void);

void puller::init(void) {
    running = false;
    stepper::enable(PULLER_STEPPER);
    current_filament_diameter = indicator::read();
    adjustment_wait_tick = xTaskGetTickCount();
    xTaskCreate(puller_task, "puller task", 512, NULL, 3, NULL);
    return;
}

void puller::start(void) {
    stepper::set_velocity(PULLER_STEPPER, PULLER_STARTING_VELOCITY);
    current_filament_diameter = indicator::read();
    adjustment_wait_tick = xTaskGetTickCount() + PULLER_ADJUSTMENT_DELAY;
    running = true;
    return;
}

void puller::stop(void) {
    running = false;
    stepper::stop(PULLER_STEPPER);
    current_velocity = 0.0f;
    adjustment_wait_tick = xTaskGetTickCount();
    return;
}

bool puller::is_running(void) { return running; }

float puller::get_diameter(void) { return current_filament_diameter; }

float puller::get_filtered_diameter(void) { return filtered_diameter; }

float puller::get_flow_rate(void) { return current_volume_flow; }

float puller::get_velocity(void) { return current_velocity; }

void puller_task([[maybe_unused]] void* args) {
    TickType_t wake_tick = xTaskGetTickCount();
    uint8_t sample_idx = 0;
    for (;;) {
        vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(PULLER_PERIOD));
        if (!running) {
            continue;
        }
        current_filament_diameter = indicator::read();
        if (xTaskGetTickCount() < adjustment_wait_tick) {
            continue;
        }
        if (sample_idx < PULLER_SENSING_WINDOW) {
            // fill up sample buffer
            filament_diameter[sample_idx] = current_filament_diameter;
            sample_idx++;
        } else {
            // adjust
            filtered_diameter = get_median_diameter();
            float filtered_radius = filtered_diameter / 2.0f;
            current_volume_flow =
                current_velocity * filtered_radius * filtered_radius * M_PI;
            float ideal_velocity =
                current_volume_flow / PULLER_TARGET_CROSS_SECTION_AREA;
            // to prevent overshooting
            float target_velocity =
                current_velocity + (ideal_velocity - current_velocity) *
                                       PULLER_ADJUSTMENT_MULTIPLIER;
            if (std::fabs(filtered_diameter - PULLER_TARGET_DIAMETER) >
                // skip adjustment if within deadzone
                PULLER_DIAMETER_DEADZONE) {
                // actually adjust
                stepper::set_velocity(PULLER_STEPPER, target_velocity);
                current_velocity = target_velocity;
            }
            sample_idx = 0;
            adjustment_wait_tick =
                xTaskGetTickCount() + PULLER_ADJUSTMENT_DELAY;
            continue;
        }
    }
}

float get_median_diameter(void) {
    float sorted[PULLER_SENSING_WINDOW] = {};
    memcpy(sorted, filament_diameter, sizeof(filament_diameter));

    for (int i = 1; i < PULLER_SENSING_WINDOW; i++) {
        float key = sorted[i];
        int j = i - 1;

        while (j >= 0 && sorted[j] > key) {
            sorted[j + 1] = sorted[j];
            j = j - 1;
        }
        sorted[j + 1] = key;
    }

    if (PULLER_SENSING_WINDOW % 2 != 0) {
        return sorted[PULLER_SENSING_WINDOW / 2];
    } else {
        return (sorted[(PULLER_SENSING_WINDOW - 1) / 2] +
                sorted[PULLER_SENSING_WINDOW / 2]) /
               2.0f;
    }
}
