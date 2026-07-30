#include "line_tracker.h"

#include <assert.h>
#include <stdio.h>

static const line_tracker_config_t k_config = {
    .white_adc = {170U, 170U, 170U, 170U, 170U, 170U, 170U, 170U},
    .black_adc = {4095U, 4095U, 4095U, 4095U, 4095U, 4095U, 4095U, 4095U},
    .x_mm = {-35.0f, -25.0f, -15.0f, -5.0f, 5.0f, 15.0f, 25.0f, 35.0f},
    .black_on_strength = 600U,
    .black_off_strength = 400U,
    .max_track_black_count = 3U,
    .lost_limit_ticks = 3U,
    .center_gap_limit_ticks = 2U,
    .center_gap_error_limit = 0.20f,
    .base_duty = 0.10f,
    .wide_line_duty = 0.08f,
    .search_inner_duty = 0.03f,
    .search_outer_duty = 0.08f,
    .start_ramp_step = 0.02f,
    .kp = 0.10f,
    .kd = 0.02f,
    .correction_limit = 0.10f,
    .duty_limit = 0.25f
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
    assert(tracker.output.state == LINE_TRACKER_FAULT);
}

static void test_wide_black_pattern_stops(void)
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
    assert(tracker.output.state == LINE_TRACKER_FAULT);
    assert(tracker.output.left_duty == 0.0f);
}

int main(void)
{
    test_pd_direction_and_ramp();
    test_hysteresis();
    test_lost_line_fault();
    test_center_gap_is_not_immediate_fault();
    test_wide_black_pattern_stops();
    puts("line_tracker host tests: PASS");
    return 0;
}
