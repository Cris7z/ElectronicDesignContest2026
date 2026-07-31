#ifndef WHEEL_SPEED_PI_H
#define WHEEL_SPEED_PI_H

#include <stdbool.h>
#include <stdint.h>

#define WHEEL_SPEED_PI_WINDOW_TICKS 4U

typedef struct {
    float counts_per_command_window;
    float measurement_filter_alpha;
    float kp;
    float ki_per_second;
    float integral_limit;
    float correction_limit;
    float enable_threshold;
} wheel_speed_pi_config_t;

typedef struct {
    float left_target;
    float right_target;
    float left_measured;
    float right_measured;
    float left_correction;
    float right_correction;
    bool updated;
} wheel_speed_pi_output_t;

typedef struct {
    wheel_speed_pi_config_t config;
    wheel_speed_pi_output_t output;
    int64_t left_count_previous;
    int64_t right_count_previous;
    int64_t left_count_sum;
    int64_t right_count_sum;
    float left_target_sum;
    float right_target_sum;
    float left_measured_filtered;
    float right_measured_filtered;
    float left_integral;
    float right_integral;
    float left_target_previous;
    float right_target_previous;
    uint8_t window_ticks;
    bool encoder_seeded;
    bool measurement_seeded;
} wheel_speed_pi_t;

bool wheel_speed_pi_init(wheel_speed_pi_t *controller,
                         const wheel_speed_pi_config_t *config);
void wheel_speed_pi_reset(wheel_speed_pi_t *controller);

/* Counts and targets use the same sign convention: physical forward is
 * positive. Call once per 5 ms control tick. The PI calculation updates once
 * per four calls and is diagnostic-only until an app explicitly consumes its
 * correction outputs. */
void wheel_speed_pi_step(wheel_speed_pi_t *controller,
                         int64_t left_forward_count,
                         int64_t right_forward_count,
                         float left_target,
                         float right_target);

#endif
