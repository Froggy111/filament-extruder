#include "PID.hpp"

PID::PID(float Kp, float Ki, float Kd, float frequency,
         bool use_filtered_derivative)
    : Kp(Kp),
      Ki(Ki),
      Kd(Kd),
      frequency(frequency),
      inv_frequency(1.0f / frequency),
      curr_err(0.0f),
      sum_err(0.0f),
      past_err(0.0f),
      target(0.0f),
      actual(0.0f),
      use_filtered_derivative(use_filtered_derivative) {}

void PID::update(float actual) {
    past_err = curr_err;
    curr_err = target - actual;
    sum_err += curr_err * inv_frequency;
    this->actual = actual;
    return;
}

void PID::update_derivative(float derivative) {
    filtered_derivative = derivative;
}

void PID::set(float target) {
    this->target = target;
    return;
}

float PID::get(void) {
    float p = curr_err * Kp;
    float i = sum_err * Ki;
    float d;
    if (use_filtered_derivative) {
        d = filtered_derivative * Kd;
    } else {
        d = (curr_err - past_err) * frequency * Kd;
    }
    float pid = p + i + d;
    return pid;
}

float PID::get_target(void) { return target; }

float PID::get_actual(void) { return actual; }

void PID::set_params(float Kp, float Ki, float Kd, float frequency) {
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
    this->frequency = frequency;
    this->inv_frequency = 1.0f / frequency;
    return;
}

void PID::anti_windup(float limit) {
    float p = curr_err * Kp;
    float i_limit = limit - p;
    sum_err = i_limit / Ki;
    return;
}

void PID::reset(void) {
    curr_err = 0, sum_err = 0, past_err = 0;
    target = 0;
    actual = 0;
    filtered_derivative = 0;
}
