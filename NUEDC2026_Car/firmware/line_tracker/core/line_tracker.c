#include "line_tracker.h"

#include <string.h>

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static float absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint8_t count_bits(uint8_t bits)
{
    uint8_t count = 0U;

    while (bits != 0U) {
        count = (uint8_t)(count + (bits & 1U));
        bits = (uint8_t)(bits >> 1U);
    }
    return count;
}

static bool config_valid(const line_tracker_config_t *config)
{
    uint8_t index;

    if ((config == NULL) || (config->black_on_strength <=
                             config->black_off_strength) ||
        (config->max_track_black_count == 0U) ||
        (config->max_track_black_count >= LINE_TRACKER_SENSOR_COUNT) ||
        (config->lost_limit_ticks == 0U) ||
        (config->duty_limit <= 0.0f) || (config->base_duty <= 0.0f)) {
        return false;
    }
    for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
        if (config->black_adc[index] <= config->white_adc[index]) {
            return false;
        }
    }
    return true;
}

static void stop_output(line_tracker_t *tracker)
{
    tracker->output.left_duty = 0.0f;
    tracker->output.right_duty = 0.0f;
}

void line_tracker_reset(line_tracker_t *tracker)
{
    if (tracker == NULL) {
        return;
    }
    memset(&tracker->output, 0, sizeof(tracker->output));
    tracker->output.state = LINE_TRACKER_WAIT;
    tracker->previous_error = 0.0f;
    tracker->last_seen_error = 0.0f;
    tracker->ramped_base_duty = 0.0f;
    tracker->center_gap_ticks = 0U;
    tracker->has_seen_track = false;
}

bool line_tracker_init(line_tracker_t *tracker,
                       const line_tracker_config_t *config)
{
    if ((tracker == NULL) || !config_valid(config)) {
        return false;
    }
    memset(tracker, 0, sizeof(*tracker));
    tracker->config = *config;
    line_tracker_reset(tracker);
    return true;
}

void line_tracker_force_fault(line_tracker_t *tracker)
{
    if (tracker == NULL) {
        return;
    }
    tracker->output.state = LINE_TRACKER_FAULT;
    stop_output(tracker);
}

static void update_line(line_tracker_t *tracker,
                        const line_tracker_input_t *input)
{
    const uint8_t previous_black_mask = tracker->output.black_mask;
    float weighted_sum = 0.0f;
    float strength_sum = 0.0f;
    uint8_t index;

    tracker->output.black_mask = 0U;
    if (!input->line_valid) {
        tracker->output.black_count = 0U;
        return;
    }
    for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
        const int32_t span = (int32_t)tracker->config.black_adc[index] -
                             (int32_t)tracker->config.white_adc[index];
        const int32_t numerator = (int32_t)input->raw_adc[index] -
                                  (int32_t)tracker->config.white_adc[index];
        const uint16_t strength = (uint16_t)clampf(
            ((float)numerator * 1000.0f) / (float)span, 0.0f, 1000.0f);
        const uint8_t bit = (uint8_t)(1U << index);
        bool black = (previous_black_mask & bit) != 0U;

        if (black) {
            black = strength > tracker->config.black_off_strength;
        } else {
            black = strength >= tracker->config.black_on_strength;
        }
        tracker->output.strength[index] = strength;
        if (black) {
            tracker->output.black_mask |= bit;
            weighted_sum += tracker->config.x_mm[index] * (float)strength;
            strength_sum += (float)strength;
        }
    }
    tracker->output.black_count = count_bits(tracker->output.black_mask);
    if (strength_sum > 0.0f) {
        const float half_span = tracker->config.x_mm[LINE_TRACKER_SENSOR_COUNT - 1U];

        tracker->output.error = clampf(
            (weighted_sum / strength_sum) / half_span, -1.0f, 1.0f);
    }
}

static void begin_run(line_tracker_t *tracker)
{
    tracker->output.state = LINE_TRACKER_RUN;
    tracker->output.lost_ticks = 0U;
    tracker->output.center_gap_mode = false;
    tracker->previous_error = 0.0f;
    tracker->last_seen_error = 0.0f;
    tracker->ramped_base_duty = 0.0f;
    tracker->center_gap_ticks = 0U;
    tracker->has_seen_track = false;
}

static void apply_search(line_tracker_t *tracker)
{
    if (tracker->last_seen_error < 0.0f) {
        tracker->output.left_duty = tracker->config.search_inner_duty;
        tracker->output.right_duty = tracker->config.search_outer_duty;
    } else if (tracker->last_seen_error > 0.0f) {
        tracker->output.left_duty = tracker->config.search_outer_duty;
        tracker->output.right_duty = tracker->config.search_inner_duty;
    } else {
        tracker->output.left_duty = tracker->config.search_outer_duty;
        tracker->output.right_duty = tracker->config.search_outer_duty;
    }
}

static void run_controller(line_tracker_t *tracker,
                           const line_tracker_input_t *input)
{
    float requested_base;
    float correction;

    if (input->line_valid &&
        (tracker->output.black_count > tracker->config.max_track_black_count)) {
        /* Four or more black elements is not a normal thin track line. */
        line_tracker_force_fault(tracker);
        return;
    }

    if (input->line_valid && (tracker->output.black_count > 0U)) {
        tracker->last_seen_error = tracker->output.error;
        tracker->has_seen_track = true;
        tracker->output.lost_ticks = 0U;
        tracker->output.center_gap_mode = false;
        tracker->center_gap_ticks = 0U;
    } else if (tracker->has_seen_track &&
               (absf(tracker->last_seen_error) <=
                tracker->config.center_gap_error_limit) &&
               (tracker->center_gap_ticks <
                tracker->config.center_gap_limit_ticks)) {
        ++tracker->center_gap_ticks;
        tracker->output.center_gap_mode = true;
        tracker->output.lost_ticks = 0U;
        tracker->output.error = 0.0f;
    } else {
        tracker->output.center_gap_mode = false;
        if (tracker->output.lost_ticks < UINT16_MAX) {
            ++tracker->output.lost_ticks;
        }
        if (tracker->output.lost_ticks >= tracker->config.lost_limit_ticks) {
            line_tracker_force_fault(tracker);
            return;
        }
        apply_search(tracker);
        return;
    }

    requested_base = (tracker->output.black_count >= 4U)
        ? tracker->config.wide_line_duty : tracker->config.base_duty;
    if (tracker->ramped_base_duty < requested_base) {
        tracker->ramped_base_duty += tracker->config.start_ramp_step;
        if (tracker->ramped_base_duty > requested_base) {
            tracker->ramped_base_duty = requested_base;
        }
    } else {
        tracker->ramped_base_duty = requested_base;
    }
    correction = (tracker->config.kp * tracker->output.error) +
                 (tracker->config.kd *
                  (tracker->output.error - tracker->previous_error));
    correction = clampf(correction, -tracker->config.correction_limit,
                         tracker->config.correction_limit);
    tracker->output.left_duty = clampf(tracker->ramped_base_duty + correction,
                                       0.0f, tracker->config.duty_limit);
    tracker->output.right_duty = clampf(tracker->ramped_base_duty - correction,
                                        0.0f, tracker->config.duty_limit);
    tracker->previous_error = tracker->output.error;
}

void line_tracker_step(line_tracker_t *tracker,
                       const line_tracker_input_t *input)
{
    if ((tracker == NULL) || (input == NULL)) {
        return;
    }
    update_line(tracker, input);
    if (input->stop_event) {
        line_tracker_reset(tracker);
        return;
    }
    if (tracker->output.state == LINE_TRACKER_WAIT) {
        stop_output(tracker);
        if (input->start_event) {
            begin_run(tracker);
        }
        return;
    }
    if (tracker->output.state == LINE_TRACKER_FAULT) {
        stop_output(tracker);
        if (input->start_event) {
            line_tracker_reset(tracker);
        }
        return;
    }
    run_controller(tracker, input);
}
