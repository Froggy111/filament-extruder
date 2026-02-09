#include "spooler.hpp"

#include <FreeRTOS.h>
#include <task.h>

#include "config.hpp"
#include "debug.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "stepper.hpp"

static int64_t encoder_count = 0;
static float position = 0;
static bool running = false;
static TickType_t wake_tick = 0;

void spooler_task(void* args);
inline bool front_endstop_triggered(void) {
    return gpio::read(SPOOLER_FRONT_ENDSTOP);
}
inline bool back_endstop_triggered(void) {
    return gpio::read(SPOOLER_BACK_ENDSTOP);
}

void spooler::init(void) {
    gpio::init(SPOOLER_BACK_ENDSTOP, gpio::Mode::INPUT, gpio::Pull::UP,
               gpio::Speed::LOW);
    gpio::init(SPOOLER_FRONT_ENDSTOP, gpio::Mode::INPUT, gpio::Pull::UP,
               gpio::Speed::LOW);
    stepper::enable(SPOOLER_STEPPER);
    xTaskCreate(spooler_task, "spooler task", 512, NULL, 2, NULL);
    debug::log("starting home");
    spooler::home();
    encoder::set_count(0);
    encoder_count = encoder::get_count();
    return;
}

void spooler::home(void) {
    stepper::set_velocity(SPOOLER_STEPPER, -SPOOLER_HOME_VELOCITY);
    while (!back_endstop_triggered());
    stepper::set_velocity(SPOOLER_STEPPER, 0.0f);
    stepper::stop(SPOOLER_STEPPER);
    stepper::zero_position(SPOOLER_STEPPER);
    stepper::move(SPOOLER_STEPPER, SPOOLER_OFFSET, SPOOLER_VELOCITY);
    while (!stepper::move_complete(SPOOLER_STEPPER));
    stepper::zero_position(SPOOLER_STEPPER);
    encoder::set_count(0);
    encoder_count = encoder::get_count();
    return;
}

void spooler::start(void) {
    running = true;
    motor::set_duty_cycle(SPOOLER_DUTY_CYCLE);
}

void spooler::stop(void) {
    running = false;
    motor::set_duty_cycle(0.0f);
}

const float DIST_PER_ENCODER_COUNT = 1.75f / (float)ENCODER_RESOLUTION;
const float LOOPS_PER_ROUND = SPOOL_WIDTH / 1.75f;
const uint32_t ENCODER_COUNTS_PER_LAYER =
    (uint32_t)((float)ENCODER_RESOLUTION * LOOPS_PER_ROUND);
const int64_t period = 2 * (int64_t)ENCODER_COUNTS_PER_LAYER;
void spooler_task([[maybe_unused]] void* args) {
    for (;;) {
        if (!running) {
            vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(1));
            wake_tick = xTaskGetTickCount();
            continue;
        }
        encoder_count = encoder::get_count();
        int64_t rev_count = encoder_count % period;
        if (rev_count < 0) {
            rev_count += period;
        }
        if (rev_count >= ENCODER_COUNTS_PER_LAYER) {
            rev_count = 2 * ENCODER_COUNTS_PER_LAYER - rev_count;
        }
        position = rev_count * DIST_PER_ENCODER_COUNT;
        stepper::move(SPOOLER_STEPPER, position, SPOOLER_VELOCITY);
        vTaskDelayUntil(&wake_tick, pdMS_TO_TICKS(1));
    }
}
