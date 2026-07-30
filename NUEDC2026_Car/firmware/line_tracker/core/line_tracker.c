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

static float maxf(float left, float right)
{
    return (left > right) ? left : right;
}

static float smoothstep(float value, float start, float end)
{
    float t;

    if (value <= start) {
        return 0.0f;
    }
    if (value >= end) {
        return 1.0f;
    }
    t = (value - start) / (end - start);
    return t * t * (3.0f - (2.0f * t));
}

static float ramp_toward(float current, float target, float accel_step,
                         float decel_step)
{
    if (target > current) {
        return (current + accel_step < target) ? current + accel_step : target;
    }
    return (current - decel_step > target) ? current - decel_step : target;
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
        (config->centroid_min_strength == 0U) ||
        (config->centroid_min_strength >= config->black_on_strength) ||
        (config->max_track_black_count == 0U) ||
        (config->max_track_black_count >= LINE_TRACKER_SENSOR_COUNT) ||
        (config->lost_limit_ticks == 0U) ||
        (config->lost_confirm_ticks == 0U) ||
        (config->cross_confirm_ticks == 0U) ||
        (config->duty_limit <= 0.0f) ||
        (config->speed_max_duty < config->speed_min_duty) ||
        (config->speed_start_duty <= 0.0f) ||
        (config->speed_start_duty > config->speed_min_duty) ||
        (config->speed_min_duty <= 0.0f) ||
        (config->speed_max_duty > config->duty_limit) ||
        (config->speed_filter_alpha <= 0.0f) ||
        (config->speed_filter_alpha > 1.0f) ||
        (config->speed_deadband_weight < 0.0f) ||
        (config->speed_full_slow_weight <=
         config->speed_deadband_weight) ||
        (config->speed_accel_step <= 0.0f) ||
        (config->speed_decel_step <= 0.0f) ||
        (config->error_filter_alpha <= 0.0f) ||
         (config->error_filter_alpha > 1.0f) ||
        (config->pid_d_filter_alpha <= 0.0f) ||
        (config->pid_d_filter_alpha > 1.0f) ||
        (config->yaw_gain_min <= 0.0f) ||
        (config->yaw_gain_min > 1.0f) ||
        (config->yaw_gain_full_weight <= config->yaw_gain_start_weight) ||
        (config->edge_blend_full_weight <=
         config->edge_blend_start_weight) ||
        (config->steering_polarity == 0.0f)) {
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
    tracker->filtered_error = 0.0f;
    tracker->filtered_derivative = 0.0f;
    tracker->integral_error = 0.0f;
    tracker->last_seen_error = 0.0f;
    tracker->ramped_base_duty = tracker->config.speed_start_duty;
    tracker->speed_weight_lpf = 0.0f;
    tracker->center_gap_ticks = 0U;
    tracker->lost_candidate_ticks = 0U;
    tracker->cross_candidate_ticks = 0U;
    tracker->has_seen_track = false;
    tracker->error_filter_seeded = false;
    tracker->derivative_seeded = false;
    tracker->speed_filter_seeded = false;
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
        if (strength > tracker->config.centroid_min_strength) {
            const float centroid_strength = (float)(strength -
                tracker->config.centroid_min_strength);

            weighted_sum += tracker->config.x_mm[index] * centroid_strength;
            strength_sum += centroid_strength;
        }
        if (black) {
            tracker->output.black_mask |= bit;
        }
    }
    tracker->output.black_count = count_bits(tracker->output.black_mask);
    if (strength_sum > 0.0f) {
        const float half_span = tracker->config.x_mm[LINE_TRACKER_SENSOR_COUNT - 1U];

        tracker->output.error = clampf(
            (weighted_sum / strength_sum) / half_span,
            -1.0f, 1.0f);
    }
}

static void begin_run(line_tracker_t *tracker)
{
    tracker->output.state = LINE_TRACKER_RUN;
    tracker->output.lost_ticks = 0U;
    tracker->output.center_gap_mode = false;
    tracker->previous_error = 0.0f;
    tracker->filtered_error = 0.0f;
    tracker->filtered_derivative = 0.0f;
    tracker->integral_error = 0.0f;
    tracker->last_seen_error = 0.0f;
    tracker->ramped_base_duty = tracker->config.speed_start_duty;
    tracker->speed_weight_lpf = 0.0f;
    tracker->center_gap_ticks = 0U;
    tracker->lost_candidate_ticks = 0U;
    tracker->cross_candidate_ticks = 0U;
    tracker->has_seen_track = false;
    tracker->error_filter_seeded = false;
    tracker->derivative_seeded = false;
    tracker->speed_filter_seeded = false;
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
    float yaw_error;
    float raw_yaw_error;
    float speed_sample;
    float speed_blend;
    float edge_weight;
    float gain_weight;
    float edge_blend;
    float raw_derivative;
    float yaw_duty;
    float unclamped_yaw_duty;
    float requested_base;

    if (input->line_valid &&
        (tracker->output.black_count > tracker->config.max_track_black_count)) {
        if (tracker->cross_candidate_ticks < UINT8_MAX) {
            ++tracker->cross_candidate_ticks;
        }
    } else {
        tracker->cross_candidate_ticks = 0U;
    }
    tracker->output.cross_mode = tracker->cross_candidate_ticks >=
        tracker->config.cross_confirm_ticks;
    if (tracker->output.cross_mode) {
        /* All-black/wide patterns are valid optics, not an ADC bus fault.
         * Cross them safely: low speed, straight ahead, no stored D kick
         * when the normal thin line reappears. */
        tracker->output.error = 0.0f;
        tracker->output.lost_ticks = 0U;
        tracker->output.center_gap_mode = false;
        tracker->filtered_error = 0.0f;
        tracker->previous_error = 0.0f;
        tracker->filtered_derivative = 0.0f;
        tracker->error_filter_seeded = false;
        tracker->derivative_seeded = false;
        tracker->speed_weight_lpf = 0.0f;
        tracker->speed_filter_seeded = false;
        tracker->lost_candidate_ticks = 0U;
        tracker->ramped_base_duty = tracker->config.speed_min_duty;
        tracker->output.left_duty = tracker->config.speed_min_duty;
        tracker->output.right_duty = tracker->config.speed_min_duty;
        return;
    }
    if (tracker->cross_candidate_ticks != 0U) {
        /* A single wide-black frame is commonly a reflection or sample
         * transient. Preserve the last valid command during confirmation. */
        tracker->output.lost_ticks = 0U;
        tracker->lost_candidate_ticks = 0U;
        return;
    }

    if (input->line_valid && (tracker->output.black_count > 0U)) {
        tracker->last_seen_error = tracker->output.error;
        tracker->has_seen_track = true;
        tracker->output.lost_ticks = 0U;
        tracker->lost_candidate_ticks = 0U;
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
        tracker->lost_candidate_ticks = 0U;
        tracker->output.error = 0.0f;
        /* A validated centre gap is a straight segment, not a decaying
         * remnant of the last side error.  Reacquisition seeds D again. */
        tracker->filtered_error = 0.0f;
        tracker->error_filter_seeded = true;
        tracker->previous_error = 0.0f;
        tracker->filtered_derivative = 0.0f;
        tracker->derivative_seeded = false;
        tracker->speed_weight_lpf = 0.0f;
        tracker->speed_filter_seeded = false;
    } else {
        if (tracker->lost_candidate_ticks < UINT8_MAX) {
            ++tracker->lost_candidate_ticks;
        }
        if (tracker->lost_candidate_ticks < tracker->config.lost_confirm_ticks) {
            tracker->output.center_gap_mode = false;
            tracker->output.lost_ticks = 0U;
            return;
        }
        tracker->output.center_gap_mode = false;
        tracker->error_filter_seeded = false;
        tracker->filtered_derivative = 0.0f;
        tracker->derivative_seeded = false;
        tracker->ramped_base_duty = tracker->config.speed_min_duty;
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

    /* The yaw path keeps the established filtered PD behaviour.  Speed uses
     * its own slower magnitude envelope, so one ADC/centroid frame cannot
     * pulse both wheel duties.  All quantities retain CarControl's -7..+7
     * weight convention. */
    raw_yaw_error = -7.0f * tracker->output.error;
    if (!tracker->error_filter_seeded) {
        tracker->filtered_error = tracker->output.error;
        tracker->error_filter_seeded = true;
    } else {
        tracker->filtered_error += tracker->config.error_filter_alpha *
            (tracker->output.error - tracker->filtered_error);
    }
    yaw_error = -7.0f * tracker->filtered_error;
    tracker->integral_error = clampf(tracker->integral_error + yaw_error,
                                     -tracker->config.pid_integral_limit,
                                     tracker->config.pid_integral_limit);
    /* D is measurement-derived because yaw_error is a fixed-zero-setpoint
     * transform of the line measurement.  Seeding suppresses the start and
     * reacquisition kick; the filtered D path comes from main's PID module. */
    if (!tracker->derivative_seeded) {
        tracker->previous_error = yaw_error;
        tracker->filtered_derivative = 0.0f;
        tracker->derivative_seeded = true;
    } else {
        raw_derivative = yaw_error - tracker->previous_error;
        tracker->filtered_derivative += tracker->config.pid_d_filter_alpha *
            (raw_derivative - tracker->filtered_derivative);
    }
    unclamped_yaw_duty = 0.01f *
        ((tracker->config.pid_p_yaw * yaw_error) +
         (tracker->config.pid_i_yaw * tracker->integral_error) +
         (tracker->config.pid_d_yaw * tracker->filtered_derivative));
    yaw_duty = unclamped_yaw_duty;
    yaw_duty = clampf(yaw_duty, -tracker->config.yaw_limit_duty,
                       tracker->config.yaw_limit_duty);
    if ((tracker->config.pid_i_yaw > 0.0f) &&
        (yaw_duty != unclamped_yaw_duty)) {
        /* Back-calculate the part of I that caused saturation. */
        tracker->integral_error += 0.5f *
            ((yaw_duty - unclamped_yaw_duty) /
             (0.01f * tracker->config.pid_i_yaw));
        tracker->integral_error = clampf(tracker->integral_error,
                                         -tracker->config.pid_integral_limit,
                                         tracker->config.pid_integral_limit);
    }
    gain_weight = absf(yaw_error);
    yaw_duty *= tracker->config.yaw_gain_min +
        ((1.0f - tracker->config.yaw_gain_min) *
         smoothstep(gain_weight, tracker->config.yaw_gain_start_weight,
                    tracker->config.yaw_gain_full_weight));
    speed_sample = absf(raw_yaw_error);
    if (!tracker->speed_filter_seeded) {
        tracker->speed_weight_lpf = speed_sample;
        tracker->speed_filter_seeded = true;
    } else {
        tracker->speed_weight_lpf += tracker->config.speed_filter_alpha *
            (speed_sample - tracker->speed_weight_lpf);
    }
    speed_blend = smoothstep(tracker->speed_weight_lpf,
                             tracker->config.speed_deadband_weight,
                             tracker->config.speed_full_slow_weight);
    requested_base = tracker->config.speed_max_duty +
        ((tracker->config.speed_min_duty -
          tracker->config.speed_max_duty) * speed_blend);
    tracker->ramped_base_duty = ramp_toward(
        tracker->ramped_base_duty, requested_base,
        tracker->config.speed_accel_step, tracker->config.speed_decel_step);
    requested_base = tracker->ramped_base_duty;
    edge_weight = maxf(absf(raw_yaw_error), absf(yaw_error));
    if ((tracker->output.black_mask & 0x01U) != 0U) {
        edge_blend = smoothstep(edge_weight,
                                tracker->config.edge_blend_start_weight,
                                tracker->config.edge_blend_full_weight);
        yaw_duty += edge_blend * (tracker->config.edge_yaw_duty - yaw_duty);
    } else if ((tracker->output.black_mask & 0x80U) != 0U) {
        edge_blend = smoothstep(edge_weight,
                                tracker->config.edge_blend_start_weight,
                                tracker->config.edge_blend_full_weight);
        yaw_duty += edge_blend * (-tracker->config.edge_yaw_duty - yaw_duty);
    }
    yaw_duty = clampf(yaw_duty, -tracker->config.yaw_limit_duty,
                       tracker->config.yaw_limit_duty);
    yaw_duty *= tracker->config.steering_polarity;
    tracker->output.left_duty = clampf(requested_base - yaw_duty,
                                       -tracker->config.duty_limit,
                                       tracker->config.duty_limit);
    tracker->output.right_duty = clampf(requested_base + yaw_duty,
                                        -tracker->config.duty_limit,
                                        tracker->config.duty_limit);
    tracker->previous_error = yaw_error;
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
    if (!input->line_valid) {
        line_tracker_force_fault(tracker);
        return;
    }
    run_controller(tracker, input);
}
