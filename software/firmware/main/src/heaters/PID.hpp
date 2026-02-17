#pragma once

// INFO : The frequency is how frequently the loop runs (to keep units
// INFO : in seconds for intuitive Ki and Kd values). D errors are multiplied by
// INFO : this and I errors are divided by this.
class PID {
   public:
    PID(float Kp, float Ki, float Kd, float frequency,
        bool use_filtered_derivative = false);
    void update(float actual);
    void update_derivative(float derivative);
    void set(float target);
    float get(void);
    float get_target(void);
    float get_actual(void);
    void set_params(float Kp, float Ki, float Kd, float frequency);
    void anti_windup(float limit);
    void reset(void);

   private:
    float Kp = 0, Ki = 0, inv_Ki = 0, Kd = 0, frequency = 1.0f,
          inv_frequency = 1.0f;
    float curr_err = 0, sum_err = 0, past_err = 0;
    float target = 0;
    float actual = 0;
    bool use_filtered_derivative = false;
    float filtered_derivative = 0;
};
