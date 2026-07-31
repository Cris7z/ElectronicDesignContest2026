#include "line_tracker.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static const line_tracker_config_t k_config = {
    .white_adc = {170U, 170U, 170U, 170U, 170U, 170U, 170U, 170U},
    .black_adc = {4095U, 4095U, 4095U, 4095U, 4095U, 4095U, 4095U, 4095U},
    .x_mm = {-35.0f, -25.0f, -15.0f, -5.0f, 5.0f, 15.0f, 25.0f, 35.0f},
    .black_on_strength = 600U,
    .black_off_strength = 400U,
    .centroid_min_strength = 150U,
    .max_track_black_count = 3U,
    .lost_limit_ticks = 3U,
    .lost_confirm_ticks = 3U,
    .cross_confirm_ticks = 2U,
    .center_gap_limit_ticks = 2U,
    .center_gap_error_limit = 0.20f,
    .speed_start_duty = 0.08f,
    .speed_max_duty = 0.45f,
    .speed_min_duty = 0.20f,
    .speed_filter_alpha = 0.20f,
    .speed_deadband_weight = 1.0f,
    .speed_full_slow_weight = 4.0f,
    .speed_accel_step = 0.005f,
    .speed_decel_step = 0.008f,
    .search_inner_duty = 0.03f,
    .search_outer_duty = 0.08f,
    .pid_p_yaw = 10.0f,
    .pid_i_yaw = 0.0f,
    .pid_d_yaw = 0.0f,
    .pid_integral_limit = 5.0f,
    .error_filter_alpha = 0.60f,
    .pid_d_filter_alpha = 0.35f,
    .yaw_limit_duty = 0.45f,
    .yaw_slew_step = 0.45f,
    .edge_yaw_duty = 0.30f,
    .yaw_gain_min = 0.60f,
    .yaw_gain_start_weight = 1.0f,
    .yaw_gain_full_weight = 4.0f,
    .edge_blend_start_weight = 5.0f,
    .edge_blend_full_weight = 7.0f,
    .steering_polarity = 1.0f,
    .duty_limit = 0.45f
};

static line_tracker_input_t input_with_black(unsigned index)
{
    line_tracker_input_t input = {.line_valid = true};
    unsigned i;

    for (i = 0U; i < LINE_TRACKER_SENSOR_COUNT; ++i) {
        input.raw_adc[i] = 170U;
    }
    if (index < LINE_TRACKER_SENSOR_COUNT) {
        input.raw_adc[index] = 4095U;
    }
    return input;
}

static uint16_t raw_for_strength(uint16_t strength)
{
    return (uint16_t)(170U + (((uint32_t)strength * (4095U - 170U)) / 1000U));
}

static float wheel_difference(const line_tracker_t *tracker)
{
    return tracker->output.right_duty - tracker->output.left_duty;
}

static void start(line_tracker_t *tracker)
{
    line_tracker_input_t input = input_with_black(3U);

    input.start_event = true;
    line_tracker_step(tracker, &input);
    assert(tracker->output.state == LINE_TRACKER_RUN);
    input.start_event = false;
    line_tracker_step(tracker, &input);
}

static void test_pd_direction_and_ramp(void)
{
    line_tracker_t tracker;
    line_tracker_input_t left_line;

    assert(line_tracker_init(&tracker, &k_config));
    start(&tracker);
    assert(tracker.output.left_duty > 0.0f);
    assert(tracker.output.right_duty > 0.0f);
    left_line = input_with_black(0U);
    line_tracker_step(&tracker, &left_line);
    assert(tracker.output.error < 0.0f);
    assert(tracker.output.left_duty < tracker.output.right_duty);
}

static void test_hysteresis(void)
{
    line_tracker_t tracker;
    line_tracker_input_t input;

    assert(line_tracker_init(&tracker, &k_config));
    start(&tracker);
    input = input_with_black(3U);
    input.raw_adc[3] = 2132U; /* Strength about 500: remains black after ON. */
    line_tracker_step(&tracker, &input);
    assert((tracker.output.black_mask & (1U << 3U)) != 0U);
    input.raw_adc[3] = 1348U; /* Strength about 300: falls below OFF. */
    line_tracker_step(&tracker, &input);
    assert((tracker.output.black_mask & (1U << 3U)) == 0U);
}

static void test_lost_line_fault(void)
{
    line_tracker_t tracker;
    line_tracker_input_t missing;

    assert(line_tracker_init(&tracker, &k_config));
    start(&tracker);
    line_tracker_step(&tracker, &(line_tracker_input_t){.line_valid = true,
                                                         .raw_adc = {4095U, 170U, 170U, 170U, 170U, 170U, 170U, 170U}});
    missing = input_with_black(8U);
    line_tracker_step(&tracker, &missing);
    assert(tracker.output.state == LINE_TRACKER_RUN);
    assert(tracker.output.left_duty != tracker.output.right_duty);
    line_tracker_step(&tracker, &missing);
    line_tracker_step(&tracker, &missing);
    line_tracker_step(&tracker, &missing);
    assert(tracker.output.state == LINE_TRACKER_RUN);
    line_tracker_step(&tracker, &missing);
    assert(tracker.output.state == LINE_TRACKER_FAULT);
    assert(tracker.output.left_duty == 0.0f);
}

static void test_center_gap_is_not_immediate_fault(void)
{
    line_tracker_t tracker;
    line_tracker_input_t missing;

    assert(line_tracker_init(&tracker, &k_config));
    start(&tracker);
    missing = input_with_black(8U);
    line_tracker_step(&tracker, &missing);
    line_tracker_step(&tracker, &missing);
    assert(tracker.output.state == LINE_TRACKER_RUN);
    assert(tracker.output.left_duty > 0.0f);
    assert(tracker.output.left_duty == tracker.output.right_duty);
    line_tracker_step(&tracker, &missing);
    line_tracker_step(&tracker, &missing);
    line_tracker_step(&tracker, &missing);
    assert(tracker.output.state == LINE_TRACKER_RUN);
    line_tracker_step(&tracker, &missing);
    line_tracker_step(&tracker, &missing);
    assert(tracker.output.state == LINE_TRACKER_FAULT);
}

static void test_wide_black_pattern_crosses_safely(void)
{
    line_tracker_t tracker;
    line_tracker_input_t input = input_with_black(0U);

    assert(line_tracker_init(&tracker, &k_config));
    start(&tracker);
    input.raw_adc[1] = 4095U;
    input.raw_adc[2] = 4095U;
    input.raw_adc[3] = 4095U;
    line_tracker_step(&tracker, &input);
    assert(tracker.output.black_count == 4U);
    assert(tracker.output.state == LINE_TRACKER_RUN);
    assert(!tracker.output.cross_mode);
    line_tracker_step(&tracker, &input);
    assert(tracker.output.cross_mode);
    assert(tracker.output.left_duty == k_config.speed_min_duty);
    assert(tracker.output.right_duty == k_config.speed_min_duty);
}

static void test_invalid_line_sample_faults_immediately(void)
{
    line_tracker_t tracker;

    assert(line_tracker_init(&tracker, &k_config));
    start(&tracker);
    line_tracker_step(&tracker, &(line_tracker_input_t){.line_valid = false});
    assert(tracker.output.state == LINE_TRACKER_FAULT);
    assert(tracker.output.left_duty == 0.0f);
    assert(tracker.output.right_duty == 0.0f);
}

static void test_independent_limit_keeps_inner_wheel_forward(void)
{
    line_tracker_t tracker;
    line_tracker_config_t config = k_config;
    line_tracker_input_t centre = input_with_black(3U);
    line_tracker_input_t edge = input_with_black(0U);
    unsigned tick;

    config.error_filter_alpha = 1.0f;
    config.pid_p_yaw = 10.0f;
    config.pid_d_yaw = 0.0f;
    centre.raw_adc[4] = 4095U;
    assert(line_tracker_init(&tracker, &config));
    centre.start_event = true;
    line_tracker_step(&tracker, &centre);
    centre.start_event = false;
    for (tick = 0U; tick < 80U; ++tick) {
        line_tracker_step(&tracker, &centre);
    }
    line_tracker_step(&tracker, &edge);
    assert(wheel_difference(&tracker) < 0.32f);
    assert(wheel_difference(&tracker) > 0.30f);
    assert(fabsf(tracker.output.right_duty - config.duty_limit) < 0.0001f);
    assert(tracker.output.left_duty > 0.0f);
}

static void test_first_line_sample_has_no_derivative_kick(void)
{
    line_tracker_t tracker;
    line_tracker_config_t config = k_config;
    line_tracker_input_t left_line = input_with_black(2U);

    config.pid_p_yaw = 0.0f;
    config.pid_i_yaw = 0.0f;
    config.pid_d_yaw = 125.0f;
    assert(line_tracker_init(&tracker, &config));
    left_line.start_event = true;
    line_tracker_step(&tracker, &left_line);
    assert(tracker.output.state == LINE_TRACKER_RUN);
    left_line.start_event = false;
    line_tracker_step(&tracker, &left_line);
    assert(tracker.output.left_duty == tracker.output.right_duty);
    line_tracker_step(&tracker, &(line_tracker_input_t){.line_valid = true,
                                                         .raw_adc = {170U, 170U, 170U, 4095U,
                                                                     170U, 170U, 170U, 170U}});
    assert(tracker.output.left_duty != tracker.output.right_duty);
}

static void test_error_filter_smooths_steering_input(void)
{
    line_tracker_t tracker;
    line_tracker_input_t left_line;

    assert(line_tracker_init(&tracker, &k_config));
    start(&tracker);
    left_line = input_with_black(2U);
    line_tracker_step(&tracker, &left_line);
    assert(tracker.output.error < 0.0f);
    assert(tracker.filtered_error < 0.0f);
    assert(tracker.filtered_error > tracker.output.error);
}

static void test_analog_centroid_uses_partial_neighbor(void)
{
    line_tracker_t tracker;
    line_tracker_input_t input = input_with_black(3U);

    assert(line_tracker_init(&tracker, &k_config));
    start(&tracker);
    input.raw_adc[4] = raw_for_strength(500U);
    line_tracker_step(&tracker, &input);
    assert(tracker.output.black_count == 1U);
    assert(tracker.output.error < 0.0f);
    assert(tracker.output.error > -0.10f);
}

static void test_speed_curve_and_ramp(void)
{
    line_tracker_t tracker;
    line_tracker_config_t config = k_config;
    line_tracker_input_t centre = input_with_black(3U);
    line_tracker_input_t edge = input_with_black(0U);
    float before_decel;
    unsigned tick;

    config.pid_p_yaw = 0.0f;
    config.pid_d_yaw = 0.0f;
    config.error_filter_alpha = 1.0f;
    centre.raw_adc[4] = 4095U;
    assert(line_tracker_init(&tracker, &config));
    centre.start_event = true;
    line_tracker_step(&tracker, &centre);
    centre.start_event = false;
    line_tracker_step(&tracker, &centre);
    assert(fabsf(tracker.ramped_base_duty -
                 (config.speed_start_duty + config.speed_accel_step)) < 0.0001f);
    for (tick = 0U; tick < 80U; ++tick) {
        line_tracker_step(&tracker, &centre);
    }
    assert(fabsf(tracker.ramped_base_duty - config.speed_max_duty) < 0.0001f);
    before_decel = tracker.ramped_base_duty;
    line_tracker_step(&tracker, &edge);
    assert(fabsf((before_decel - tracker.ramped_base_duty) -
                 config.speed_decel_step) < 0.0001f);
    assert(tracker.ramped_base_duty >= config.speed_min_duty);
}

static void test_speed_deadband_keeps_centre_sensor_at_maximum(void)
{
    line_tracker_t tracker;
    line_tracker_config_t config = k_config;
    line_tracker_input_t centre = input_with_black(3U);
    line_tracker_input_t one_centre_sensor = input_with_black(3U);
    unsigned tick;

    config.pid_p_yaw = 0.0f;
    config.pid_d_yaw = 0.0f;
    centre.raw_adc[4] = 4095U;
    assert(line_tracker_init(&tracker, &config));
    centre.start_event = true;
    line_tracker_step(&tracker, &centre);
    centre.start_event = false;
    for (tick = 0U; tick < 80U; ++tick) {
        line_tracker_step(&tracker, &centre);
    }
    assert(fabsf(tracker.ramped_base_duty - config.speed_max_duty) < 0.0001f);
    line_tracker_step(&tracker, &one_centre_sensor);
    assert(fabsf(tracker.ramped_base_duty - config.speed_max_duty) < 0.0001f);
}

static void test_one_frame_edge_spike_is_not_a_throttle_pulse(void)
{
    line_tracker_t tracker;
    line_tracker_config_t config = k_config;
    line_tracker_input_t centre = input_with_black(3U);
    line_tracker_input_t edge = input_with_black(0U);
    float before_spike;
    unsigned tick;

    config.pid_p_yaw = 0.0f;
    config.pid_d_yaw = 0.0f;
    centre.raw_adc[4] = 4095U;
    assert(line_tracker_init(&tracker, &config));
    centre.start_event = true;
    line_tracker_step(&tracker, &centre);
    centre.start_event = false;
    for (tick = 0U; tick < 80U; ++tick) {
        line_tracker_step(&tracker, &centre);
    }
    before_spike = tracker.ramped_base_duty;
    line_tracker_step(&tracker, &edge);
    assert((before_spike - tracker.ramped_base_duty) <=
           (config.speed_decel_step + 0.0001f));
    assert(tracker.ramped_base_duty > config.speed_max_duty - 0.0100f);
}

static void test_centroid_threshold_is_soft(void)
{
    line_tracker_t tracker;
    line_tracker_input_t below = input_with_black(3U);
    line_tracker_input_t at = input_with_black(3U);
    float below_error;

    assert(line_tracker_init(&tracker, &k_config));
    start(&tracker);
    below.raw_adc[4] = raw_for_strength(149U);
    line_tracker_step(&tracker, &below);
    below_error = tracker.output.error;
    at.raw_adc[4] = raw_for_strength(150U);
    line_tracker_step(&tracker, &at);
    assert(fabsf(tracker.output.error - below_error) < 0.0001f);
}

static void test_smooth_gain_and_edge_blend(void)
{
    line_tracker_t tracker;
    line_tracker_config_t config = k_config;
    line_tracker_input_t weight_one = input_with_black(3U);
    line_tracker_input_t weight_three = input_with_black(2U);
    line_tracker_input_t weight_four = input_with_black(1U);
    line_tracker_input_t weight_six = input_with_black(0U);
    float at_three;
    float at_four;

    config.pid_p_yaw = 10.0f;
    config.pid_d_yaw = 0.0f;
    config.error_filter_alpha = 1.0f;
    assert(line_tracker_init(&tracker, &config));
    weight_one.start_event = true;
    line_tracker_step(&tracker, &weight_one);
    weight_one.start_event = false;
    line_tracker_step(&tracker, &weight_one);
    line_tracker_step(&tracker, &weight_three);
    at_three = wheel_difference(&tracker);
    weight_four.raw_adc[2] = 4095U; /* Average of weights 5 and 3 is 4. */
    line_tracker_step(&tracker, &weight_four);
    at_four = wheel_difference(&tracker);
    assert(at_four > at_three);
    assert(at_four - at_three < 0.30f);
    config.pid_p_yaw = 0.0f;
    assert(line_tracker_init(&tracker, &config));
    weight_six.start_event = true;
    line_tracker_step(&tracker, &weight_six);
    weight_six.start_event = false;
    weight_six.raw_adc[1] = 4095U; /* Average of weights 7 and 5 is 6. */
    line_tracker_step(&tracker, &weight_six);
    assert(wheel_difference(&tracker) > 0.29f);
    assert(wheel_difference(&tracker) < 0.31f);
}

static void test_final_yaw_slew_limits_edge_transition(void)
{
    line_tracker_t tracker;
    line_tracker_config_t config = k_config;
    line_tracker_input_t centre = input_with_black(3U);
    line_tracker_input_t edge = input_with_black(0U);
    float previous_yaw;

    config.pid_p_yaw = 0.0f;
    config.pid_d_yaw = 0.0f;
    config.yaw_slew_step = 0.08f;
    assert(line_tracker_init(&tracker, &config));
    centre.start_event = true;
    line_tracker_step(&tracker, &centre);
    centre.start_event = false;
    line_tracker_step(&tracker, &centre);
    previous_yaw = tracker.output.debug_final_yaw;
    line_tracker_step(&tracker, &edge);
    assert(fabsf(tracker.output.debug_final_yaw - previous_yaw) <=
           config.yaw_slew_step + 0.0001f);
    assert(fabsf(tracker.output.debug_final_yaw) > 0.079f);
}

int main(void)
{
    test_pd_direction_and_ramp();
    test_hysteresis();
    test_lost_line_fault();
    test_center_gap_is_not_immediate_fault();
    test_wide_black_pattern_crosses_safely();
    test_invalid_line_sample_faults_immediately();
    test_first_line_sample_has_no_derivative_kick();
    test_error_filter_smooths_steering_input();
    test_analog_centroid_uses_partial_neighbor();
    test_speed_curve_and_ramp();
    test_speed_deadband_keeps_centre_sensor_at_maximum();
    test_one_frame_edge_spike_is_not_a_throttle_pulse();
    test_centroid_threshold_is_soft();
    test_independent_limit_keeps_inner_wheel_forward();
    test_smooth_gain_and_edge_blend();
    test_final_yaw_slew_limits_edge_transition();
    puts("line_tracker host tests: PASS");
    return 0;
}
