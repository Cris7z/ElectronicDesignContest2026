#include "h2026_q2.h"

#include <math.h>
#include <string.h>

static float clampf(float value, float lower, float upper)
{
    if (value < lower) {
        return lower;
    }
    if (value > upper) {
        return upper;
    }
    return value;
}

static float absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static uint32_t add_tick_saturating(uint32_t value)
{
    if (value > (UINT32_MAX - H2026_Q2_TICK_MS)) {
        return UINT32_MAX;
    }
    return value + H2026_Q2_TICK_MS;
}

static bool is_finite_positive(float value)
{
    return isfinite(value) && (value > 0.0f);
}

static bool is_finite_nonnegative(float value)
{
    return isfinite(value) && (value >= 0.0f);
}

static bool is_tick_multiple(uint32_t duration_ms)
{
    return (duration_ms % H2026_Q2_TICK_MS) == 0U;
}

static bool speed_pi_config_valid(const h2026_q2_speed_pi_config_t *config)
{
    return is_finite_nonnegative(config->kp) &&
           is_finite_nonnegative(config->ki) &&
           is_finite_nonnegative(config->ks) &&
           is_finite_nonnegative(config->kv) &&
           is_finite_nonnegative(config->integral_limit);
}

bool h2026_q2_config_validate(const h2026_q2_config_t *config)
{
    size_t index;
    float previous_end = 0.0f;

    if (config == NULL) {
        return false;
    }
    if ((config->wide_min_active < 2U) ||
        (config->wide_min_active > 7U) ||
        (config->marker_capture_min_active < 2U) ||
        (config->marker_capture_min_active > 7U) ||
        (config->marker_detect_min_active < 2U) ||
        (config->marker_detect_min_active >
         config->marker_capture_min_active) ||
        (config->marker_count_tolerance > 3U) ||
        !isfinite(config->marker_detect_ratio) ||
        (config->marker_detect_ratio <= 0.0f) ||
        (config->marker_detect_ratio > 1.0f) ||
        !isfinite(config->marker_center_limit_normalized) ||
        (config->marker_center_limit_normalized <= 0.0f) ||
        (config->marker_center_limit_normalized > 1.0f)) {
        return false;
    }
    if ((config->marker_release_ms == 0U) ||
        (config->marker_confirm_ms == 0U) ||
        (config->stop_hold_ms == 0U) ||
        !is_tick_multiple(config->marker_release_ms) ||
        !is_tick_multiple(config->marker_confirm_ms) ||
        !is_tick_multiple(config->stop_hold_ms) ||
        !is_tick_multiple(config->i2c_grace_ms) ||
        !is_tick_multiple(config->i2c_fault_ms) ||
        !is_tick_multiple(config->line_grace_ms) ||
        !is_tick_multiple(config->line_fault_ms) ||
        !is_tick_multiple(config->finish_gate_time_ms) ||
        !is_tick_multiple(config->mission_timeout_ms) ||
        !is_tick_multiple(config->stopping_timeout_ms) ||
        !is_tick_multiple(config->fault_coast_max_ms) ||
        (config->finish_gate_time_ms == 0U) ||
        (config->i2c_fault_ms == 0U) ||
        (config->line_fault_ms == 0U) ||
        (config->mission_timeout_ms == 0U) ||
        (config->stopping_timeout_ms == 0U) ||
        (config->fault_coast_max_ms == 0U) ||
        (config->i2c_grace_ms >= config->i2c_fault_ms) ||
        (config->line_grace_ms >= config->line_fault_ms) ||
        (config->finish_gate_time_ms >= config->mission_timeout_ms)) {
        return false;
    }
    if (!is_finite_nonnegative(config->start_clear_distance_m) ||
        !is_finite_positive(config->finish_gate_distance_m) ||
        (config->finish_gate_distance_m <=
         config->start_clear_distance_m) ||
        !is_finite_positive(config->stop_distance_from_marker_m) ||
        !is_finite_positive(config->stop_position_tolerance_m) ||
        !is_finite_positive(config->stop_speed_tolerance_mps) ||
        !is_finite_positive(config->left_meters_per_encoder_count) ||
        !is_finite_positive(config->right_meters_per_encoder_count) ||
        !is_finite_positive(config->track_width_m) ||
        ((config->left_encoder_sign != 1) &&
         (config->left_encoder_sign != -1)) ||
        ((config->right_encoder_sign != 1) &&
         (config->right_encoder_sign != -1))) {
        return false;
    }
    if (!is_finite_positive(config->cruise_speed_mps) ||
        !is_finite_nonnegative(config->minimum_tracking_speed_mps) ||
        !is_finite_nonnegative(config->degraded_i2c_speed_mps) ||
        !is_finite_nonnegative(config->degraded_line_speed_mps) ||
        !is_finite_positive(config->maximum_wheel_speed_mps) ||
        (config->minimum_tracking_speed_mps > config->cruise_speed_mps) ||
        (config->degraded_i2c_speed_mps > config->cruise_speed_mps) ||
        (config->degraded_line_speed_mps > config->cruise_speed_mps) ||
        (config->maximum_wheel_speed_mps < config->cruise_speed_mps) ||
        !is_finite_positive(config->acceleration_limit_mps2) ||
        !is_finite_positive(config->deceleration_limit_mps2) ||
        !is_finite_positive(config->stopping_deceleration_mps2) ||
        !is_finite_nonnegative(config->line_error_speed_reduction) ||
        !is_finite_nonnegative(config->curvature_speed_reduction_m)) {
        return false;
    }
    if (!isfinite(config->line_error_filter_alpha) ||
        (config->line_error_filter_alpha <= 0.0f) ||
        (config->line_error_filter_alpha > 1.0f) ||
        !isfinite(config->line_derivative_filter_alpha) ||
        (config->line_derivative_filter_alpha <= 0.0f) ||
        (config->line_derivative_filter_alpha > 1.0f) ||
        !is_finite_nonnegative(config->line_kp_center_mps) ||
        !is_finite_nonnegative(config->line_kp_edge_mps) ||
        !is_finite_nonnegative(config->line_kd_center_m) ||
        !is_finite_nonnegative(config->line_kd_edge_m) ||
        !is_finite_positive(config->line_correction_limit_mps) ||
        !is_finite_nonnegative(config->curvature_feedforward_gain) ||
        !is_finite_nonnegative(config->curve_transition_m)) {
        return false;
    }
    if ((config->curve_segment_count > H2026_Q2_MAX_CURVE_SEGMENTS) ||
        !speed_pi_config_valid(&config->left_speed_pi) ||
        !speed_pi_config_valid(&config->right_speed_pi) ||
        !isfinite(config->wheel_speed_filter_alpha) ||
        (config->wheel_speed_filter_alpha <= 0.0f) ||
        (config->wheel_speed_filter_alpha > 1.0f) ||
        !is_finite_positive(config->signed_duty_limit) ||
        (config->signed_duty_limit > 1.0f)) {
        return false;
    }
    for (index = 0U; index < config->curve_segment_count; ++index) {
        const h2026_q2_curve_segment_t *segment =
            &config->curve_segments[index];

        if (!is_finite_positive(segment->end_distance_m) ||
            !isfinite(segment->curvature_1pm) ||
            (segment->end_distance_m <= previous_end)) {
            return false;
        }
        previous_end = segment->end_distance_m;
    }
    return true;
}

static uint8_t reverse_bits(uint8_t value)
{
    value = (uint8_t)(((value & 0x55U) << 1U) |
                      ((value & 0xAAU) >> 1U));
    value = (uint8_t)(((value & 0x33U) << 2U) |
                      ((value & 0xCCU) >> 2U));
    value = (uint8_t)(((value & 0x0FU) << 4U) |
                      ((value & 0xF0U) >> 4U));
    return value;
}

void h2026_q2_line_decode(uint8_t raw_bits,
                          bool active_high,
                          bool bit0_is_left,
                          uint8_t wide_min_active,
                          h2026_q2_line_observation_t *observation)
{
    uint8_t normalized;
    uint8_t active_count = 0U;
    uint8_t block_count = 0U;
    uint8_t bit_index;
    int centroid_numerator = 0;
    bool previous_active = false;

    if (observation == NULL) {
        return;
    }
    if ((wide_min_active == 0U) || (wide_min_active > 8U)) {
        wide_min_active = 8U;
    }

    normalized = active_high ? raw_bits : (uint8_t)~raw_bits;
    if (!bit0_is_left) {
        normalized = reverse_bits(normalized);
    }

    memset(observation, 0, sizeof(*observation));
    observation->raw_bits = raw_bits;
    observation->normalized_bits = normalized;

    for (bit_index = 0U; bit_index < 8U; ++bit_index) {
        const bool active =
            (normalized & (uint8_t)(1U << bit_index)) != 0U;

        if (active) {
            ++active_count;
            centroid_numerator += ((int)bit_index * 2) - 7;
            if (!previous_active) {
                ++block_count;
            }
        }
        previous_active = active;
    }

    observation->active_count = active_count;
    observation->block_count = block_count;
    if (active_count == 0U) {
        observation->classification = H2026_Q2_LINE_LOST;
    } else if (active_count == 8U) {
        observation->classification = H2026_Q2_LINE_ALL;
    } else if (block_count > 1U) {
        observation->classification = H2026_Q2_LINE_MULTI;
    } else if (active_count >= wide_min_active) {
        observation->classification = H2026_Q2_LINE_WIDE;
    } else {
        observation->classification = H2026_Q2_LINE_NORMAL;
    }

    /*
     * Never invent a centroid for a split pattern. A single continuous block
     * uses equally-spaced sensor positions from -1 (left) to +1 (right).
     */
    observation->centroid_valid =
        observation->classification == H2026_Q2_LINE_NORMAL;
    if (observation->centroid_valid) {
        observation->centroid =
            (float)centroid_numerator / ((float)active_count * 7.0f);
    }
}

static bool state_is_running(h2026_q2_state_t state)
{
    return (state == H2026_Q2_STATE_CLEAR_START) ||
           (state == H2026_Q2_STATE_LAP) ||
           (state == H2026_Q2_STATE_FINISH_ARMED) ||
           (state == H2026_Q2_STATE_STOPPING);
}

static void reset_pi(h2026_q2_controller_t *controller)
{
    controller->left_pi.integral = 0.0f;
    controller->right_pi.integral = 0.0f;
}

static void transition_state(h2026_q2_controller_t *controller,
                             h2026_q2_state_t state)
{
    controller->state = state;
    controller->state_elapsed_ms = 0U;
}

void h2026_q2_reset(h2026_q2_controller_t *controller,
                    const h2026_q2_input_t *initial_input)
{
    h2026_q2_config_t saved_config;

    if (controller == NULL) {
        return;
    }
    saved_config = controller->config;
    memset(controller, 0, sizeof(*controller));
    controller->config = saved_config;
    controller->state = H2026_Q2_STATE_IDLE;
    controller->brake = true;
    if (initial_input != NULL) {
        controller->previous_left_count =
            initial_input->encoder_left_count;
        controller->previous_right_count =
            initial_input->encoder_right_count;
        h2026_q2_line_decode(initial_input->line_raw_reg5,
                             saved_config.sensor_active_high,
                             saved_config.sensor_bit0_is_left,
                             saved_config.wide_min_active,
                             &controller->line);
    }
}

bool h2026_q2_init(h2026_q2_controller_t *controller,
                   const h2026_q2_config_t *config,
                   const h2026_q2_input_t *initial_input)
{
    if (controller == NULL) {
        return false;
    }
    memset(controller, 0, sizeof(*controller));
    if (!h2026_q2_config_validate(config) || (initial_input == NULL)) {
        controller->state = H2026_Q2_STATE_FAULT;
        controller->fault = H2026_Q2_FAULT_CONFIG;
        controller->brake = true;
        return false;
    }
    controller->config = *config;
    h2026_q2_reset(controller, initial_input);
    return true;
}

static uint8_t popcount8(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U) {
        count = (uint8_t)(count + (value & 1U));
        value = (uint8_t)(value >> 1U);
    }
    return count;
}

static float pattern_center(uint8_t normalized_bits)
{
    int numerator = 0;
    uint8_t count = 0U;
    uint8_t bit_index;

    for (bit_index = 0U; bit_index < 8U; ++bit_index) {
        if ((normalized_bits &
             (uint8_t)(1U << bit_index)) != 0U) {
            numerator += ((int)bit_index * 2) - 7;
            ++count;
        }
    }
    if (count == 0U) {
        return 0.0f;
    }
    return (float)numerator / ((float)count * 7.0f);
}

static bool marker_candidate(h2026_q2_controller_t *controller,
                             bool line_i2c_valid)
{
    uint8_t expanded_reference;
    uint8_t maximum_active;

    controller->marker_overlap_count = 0U;
    controller->marker_pattern_center = 0.0f;
    if (!line_i2c_valid || !controller->marker_captured) {
        return false;
    }

    expanded_reference =
        (uint8_t)(controller->marker_reference_bits |
                  (uint8_t)(controller->marker_reference_bits << 1U) |
                  (uint8_t)(controller->marker_reference_bits >> 1U));
    controller->marker_overlap_count =
        popcount8((uint8_t)(controller->line.normalized_bits &
                            expanded_reference));
    controller->marker_pattern_center =
        pattern_center(controller->line.normalized_bits);
    maximum_active =
        (uint8_t)(controller->marker_reference_count +
                  controller->config.marker_count_tolerance);
    if (maximum_active > 8U) {
        maximum_active = 8U;
    }
    return (controller->line.classification == H2026_Q2_LINE_WIDE) &&
           (controller->line.block_count == 1U) &&
           (controller->line.active_count >=
            controller->marker_detection_threshold) &&
           (controller->line.active_count <= maximum_active) &&
           (controller->marker_overlap_count >=
            controller->marker_detection_threshold) &&
           (absf(controller->marker_pattern_center) <=
            controller->config.marker_center_limit_normalized);
}

static uint8_t marker_threshold(const h2026_q2_controller_t *controller)
{
    float scaled_count =
        (float)controller->marker_reference_count *
        controller->config.marker_detect_ratio;
    uint8_t threshold = (uint8_t)ceilf(scaled_count);

    if (threshold < controller->config.marker_detect_min_active) {
        threshold = controller->config.marker_detect_min_active;
    }
    if (threshold < controller->config.wide_min_active) {
        threshold = controller->config.wide_min_active;
    }
    if (threshold > 8U) {
        threshold = 8U;
    }
    return threshold;
}

static bool finish_gate_open(const h2026_q2_controller_t *controller)
{
    return controller->marker_captured &&
           (controller->elapsed_ms >=
            controller->config.finish_gate_time_ms) &&
           (controller->distance_m >=
            controller->config.finish_gate_distance_m);
}

static void trip_fault(h2026_q2_controller_t *controller,
                       h2026_q2_fault_t fault)
{
    controller->fault = fault;
    controller->center_speed_command_mps = 0.0f;
    controller->left_target_speed_mps = 0.0f;
    controller->right_target_speed_mps = 0.0f;
    controller->left_signed_duty = 0.0f;
    controller->right_signed_duty = 0.0f;
    controller->fault_coasting =
        (fault != H2026_Q2_FAULT_ESTOP) &&
        (fault != H2026_Q2_FAULT_CONFIG) &&
        ((absf(controller->left_measured_speed_mps) >
          controller->config.stop_speed_tolerance_mps) ||
         (absf(controller->right_measured_speed_mps) >
          controller->config.stop_speed_tolerance_mps));
    controller->fault_coast_ms = 0U;
    controller->brake = !controller->fault_coasting;
    reset_pi(controller);
    transition_state(controller, H2026_Q2_STATE_FAULT);
}

static void update_odometry(h2026_q2_controller_t *controller,
                            const h2026_q2_input_t *input,
                            bool accumulate_distance)
{
    const h2026_q2_config_t *config = &controller->config;
    float left_instantaneous;
    float right_instantaneous;

    controller->left_encoder_delta =
        input->encoder_left_count - controller->previous_left_count;
    controller->right_encoder_delta =
        input->encoder_right_count - controller->previous_right_count;
    controller->previous_left_count = input->encoder_left_count;
    controller->previous_right_count = input->encoder_right_count;

    left_instantaneous =
        ((float)controller->left_encoder_delta *
         (float)config->left_encoder_sign *
         config->left_meters_per_encoder_count) /
        H2026_Q2_TICK_S;
    right_instantaneous =
        ((float)controller->right_encoder_delta *
         (float)config->right_encoder_sign *
         config->right_meters_per_encoder_count) /
        H2026_Q2_TICK_S;

    controller->left_measured_speed_mps +=
        config->wheel_speed_filter_alpha *
        (left_instantaneous - controller->left_measured_speed_mps);
    controller->right_measured_speed_mps +=
        config->wheel_speed_filter_alpha *
        (right_instantaneous - controller->right_measured_speed_mps);

    if (accumulate_distance) {
        const float left_delta_m =
            (float)controller->left_encoder_delta *
            (float)config->left_encoder_sign *
            config->left_meters_per_encoder_count;
        const float right_delta_m =
            (float)controller->right_encoder_delta *
            (float)config->right_encoder_sign *
            config->right_meters_per_encoder_count;

        controller->distance_m += 0.5f * (left_delta_m + right_delta_m);
        if (controller->distance_m < 0.0f) {
            controller->distance_m = 0.0f;
        }
    }
}

static void update_line_filter(h2026_q2_controller_t *controller,
                               bool line_i2c_valid)
{
    const h2026_q2_config_t *config = &controller->config;

    if (line_i2c_valid &&
        (controller->line.classification == H2026_Q2_LINE_NORMAL) &&
        controller->line.centroid_valid) {
        controller->last_raw_line_error = controller->line.centroid;
        if (!controller->line_filter_initialized) {
            controller->filtered_line_error = controller->line.centroid;
            controller->previous_filtered_line_error =
                controller->line.centroid;
            controller->filtered_line_derivative_per_s = 0.0f;
            controller->line_filter_initialized = true;
        } else {
            float raw_derivative;

            controller->filtered_line_error +=
                config->line_error_filter_alpha *
                (controller->line.centroid -
                 controller->filtered_line_error);
            raw_derivative =
                (controller->filtered_line_error -
                 controller->previous_filtered_line_error) /
                H2026_Q2_TICK_S;
            controller->filtered_line_derivative_per_s +=
                config->line_derivative_filter_alpha *
                (raw_derivative -
                 controller->filtered_line_derivative_per_s);
            controller->previous_filtered_line_error =
                controller->filtered_line_error;
        }
    } else {
        controller->filtered_line_derivative_per_s *=
            1.0f - config->line_derivative_filter_alpha;
    }

    if (controller->line_filter_initialized) {
        const float magnitude =
            clampf(absf(controller->filtered_line_error), 0.0f, 1.0f);

        controller->active_line_kp_mps =
            config->line_kp_center_mps +
            magnitude *
                (config->line_kp_edge_mps -
                 config->line_kp_center_mps);
        controller->active_line_kd_m =
            config->line_kd_center_m +
            magnitude *
                (config->line_kd_edge_m -
                 config->line_kd_center_m);
        controller->line_pd_correction_mps =
            controller->active_line_kp_mps *
                controller->filtered_line_error +
            controller->active_line_kd_m *
                controller->filtered_line_derivative_per_s;
        controller->line_pd_correction_mps =
            clampf(controller->line_pd_correction_mps,
                   -config->line_correction_limit_mps,
                   config->line_correction_limit_mps);
    } else {
        controller->active_line_kp_mps =
            config->line_kp_center_mps;
        controller->active_line_kd_m =
            config->line_kd_center_m;
        controller->line_pd_correction_mps = 0.0f;
    }
}

static float smoothstep(float value)
{
    const float limited = clampf(value, 0.0f, 1.0f);
    return limited * limited * (3.0f - (2.0f * limited));
}

static float curvature_at_distance(const h2026_q2_config_t *config,
                                   float distance_m)
{
    size_t index;
    float curvature;

    if (config->curve_segment_count == 0U) {
        return 0.0f;
    }

    curvature = config->curve_segments[0].curvature_1pm;
    for (index = 0U; index + 1U < config->curve_segment_count; ++index) {
        const float boundary =
            config->curve_segments[index].end_distance_m;
        const float delta =
            config->curve_segments[index + 1U].curvature_1pm -
            config->curve_segments[index].curvature_1pm;
        float blend;

        if (config->curve_transition_m > 0.0f) {
            const float start =
                boundary - (0.5f * config->curve_transition_m);
            blend = smoothstep(
                (distance_m - start) / config->curve_transition_m);
        } else {
            blend = (distance_m >= boundary) ? 1.0f : 0.0f;
        }
        curvature += delta * blend;
    }
    return curvature;
}

static float scheduled_tracking_speed(
    const h2026_q2_controller_t *controller)
{
    const h2026_q2_config_t *config = &controller->config;
    const float reduction =
        config->line_error_speed_reduction *
            absf(controller->filtered_line_error) +
        config->curvature_speed_reduction_m *
            absf(controller->track_curvature_1pm);
    float target =
        config->cruise_speed_mps * (1.0f - clampf(reduction, 0.0f, 1.0f));

    if (target < config->minimum_tracking_speed_mps) {
        target = config->minimum_tracking_speed_mps;
    }
    if (controller->i2c_invalid_ms > config->i2c_grace_ms) {
        target = fminf(target, config->degraded_i2c_speed_mps);
    }
    if (controller->line_unusable_ms > config->line_grace_ms) {
        target = fminf(target, config->degraded_line_speed_mps);
    }
    return target;
}

static float slew_center_speed(const h2026_q2_config_t *config,
                               float current,
                               float target,
                               float falling_limit_mps2)
{
    const float rising_step =
        config->acceleration_limit_mps2 * H2026_Q2_TICK_S;
    const float falling_step =
        falling_limit_mps2 * H2026_Q2_TICK_S;

    if (target > current + rising_step) {
        return current + rising_step;
    }
    if (target < current - falling_step) {
        return current - falling_step;
    }
    return target;
}

static float update_speed_pi(const h2026_q2_speed_pi_config_t *config,
                             h2026_q2_pi_state_t *state,
                             float target,
                             float measured,
                             float duty_limit,
                             bool *saturated)
{
    const float error = target - measured;
    float candidate_integral =
        state->integral + config->ki * error * H2026_Q2_TICK_S;
    float static_feedforward = 0.0f;
    float unsaturated;
    float output;

    candidate_integral =
        clampf(candidate_integral,
               -config->integral_limit,
               config->integral_limit);
    if (target > 0.000001f) {
        static_feedforward = config->ks;
    } else if (target < -0.000001f) {
        static_feedforward = -config->ks;
    }

    unsaturated =
        static_feedforward + config->kv * target +
        config->kp * error + candidate_integral;
    output = clampf(unsaturated, -duty_limit, duty_limit);
    *saturated = output != unsaturated;

    /*
     * Conditional integration: reject only the integral step that would push
     * an already-saturated actuator farther into saturation.
     */
    if (!*saturated ||
        ((output >= duty_limit) && (error < 0.0f)) ||
        ((output <= -duty_limit) && (error > 0.0f))) {
        state->integral = candidate_integral;
    }

    unsaturated =
        static_feedforward + config->kv * target +
        config->kp * error + state->integral;
    output = clampf(unsaturated, -duty_limit, duty_limit);
    *saturated = output != unsaturated;
    return output;
}

static void run_motion_control(h2026_q2_controller_t *controller,
                               bool line_i2c_valid)
{
    const h2026_q2_config_t *config = &controller->config;
    float target_center;
    float total_turn;

    float falling_limit_mps2 = config->deceleration_limit_mps2;

    update_line_filter(controller, line_i2c_valid);
    controller->track_curvature_1pm =
        curvature_at_distance(config, controller->distance_m);
    target_center = scheduled_tracking_speed(controller);

    if (controller->state == H2026_Q2_STATE_STOPPING) {
        const float remaining =
            controller->stop_target_m - controller->distance_m;
        float distance_limited_speed = 0.0f;

        if (remaining > 0.0f) {
            distance_limited_speed =
                sqrtf(2.0f * config->stopping_deceleration_mps2 *
                      remaining);
        }
        target_center = fminf(target_center, distance_limited_speed);
        falling_limit_mps2 = config->stopping_deceleration_mps2;
    }

    controller->center_speed_command_mps =
        slew_center_speed(config,
                          controller->center_speed_command_mps,
                          target_center,
                          falling_limit_mps2);
    controller->curvature_feedforward_mps =
        config->curvature_feedforward_gain *
        0.5f * config->track_width_m *
        controller->center_speed_command_mps *
        controller->track_curvature_1pm;
    total_turn =
        controller->line_pd_correction_mps +
        controller->curvature_feedforward_mps;
    if (controller->state == H2026_Q2_STATE_STOPPING) {
        const float remaining =
            controller->stop_target_m - controller->distance_m;
        float turn_scale =
            controller->center_speed_command_mps /
            config->cruise_speed_mps;

        if (remaining <= config->stop_position_tolerance_m) {
            turn_scale = 0.0f;
        }
        total_turn *= clampf(turn_scale, 0.0f, 1.0f);
    }

    controller->left_target_speed_mps =
        controller->center_speed_command_mps + total_turn;
    controller->right_target_speed_mps =
        controller->center_speed_command_mps - total_turn;
    {
        const float peak =
            fmaxf(absf(controller->left_target_speed_mps),
                  absf(controller->right_target_speed_mps));

        if (peak > config->maximum_wheel_speed_mps) {
            const float scale =
                config->maximum_wheel_speed_mps / peak;
            controller->left_target_speed_mps *= scale;
            controller->right_target_speed_mps *= scale;
        }
    }

    controller->left_signed_duty =
        update_speed_pi(&config->left_speed_pi,
                        &controller->left_pi,
                        controller->left_target_speed_mps,
                        controller->left_measured_speed_mps,
                        config->signed_duty_limit,
                        &controller->left_duty_saturated);
    controller->right_signed_duty =
        update_speed_pi(&config->right_speed_pi,
                        &controller->right_pi,
                        controller->right_target_speed_mps,
                        controller->right_measured_speed_mps,
                        config->signed_duty_limit,
                        &controller->right_duty_saturated);
    controller->brake = false;
}

static void command_brake(h2026_q2_controller_t *controller)
{
    controller->center_speed_command_mps = 0.0f;
    controller->left_target_speed_mps = 0.0f;
    controller->right_target_speed_mps = 0.0f;
    controller->left_signed_duty = 0.0f;
    controller->right_signed_duty = 0.0f;
    controller->left_duty_saturated = false;
    controller->right_duty_saturated = false;
    controller->brake = true;
    reset_pi(controller);
}

static void command_fault_safe(h2026_q2_controller_t *controller)
{
    if (controller->fault_coasting &&
        (controller->fault_coast_ms <
         controller->config.fault_coast_max_ms) &&
        ((absf(controller->left_measured_speed_mps) >
          controller->config.stop_speed_tolerance_mps) ||
         (absf(controller->right_measured_speed_mps) >
          controller->config.stop_speed_tolerance_mps))) {
        controller->center_speed_command_mps = 0.0f;
        controller->left_target_speed_mps = 0.0f;
        controller->right_target_speed_mps = 0.0f;
        controller->left_signed_duty = 0.0f;
        controller->right_signed_duty = 0.0f;
        controller->left_duty_saturated = false;
        controller->right_duty_saturated = false;
        controller->brake = false;
        reset_pi(controller);
    } else {
        controller->fault_coasting = false;
        command_brake(controller);
    }
}

static void populate_output(h2026_q2_controller_t *controller,
                            const h2026_q2_input_t *input,
                            h2026_q2_output_t *output)
{
    const bool is_marker = marker_candidate(controller,
                                            input->line_i2c_valid);
    uint32_t flags = 0U;

    memset(output, 0, sizeof(*output));
    if (input->line_i2c_valid) {
        flags |= H2026_Q2_DIAG_LINE_SAMPLE_VALID;
    }
    if (input->line_i2c_valid && controller->line.centroid_valid) {
        flags |= H2026_Q2_DIAG_CENTROID_VALID;
    }
    if (controller->marker_captured) {
        flags |= H2026_Q2_DIAG_MARKER_CAPTURED;
    }
    if (is_marker) {
        flags |= H2026_Q2_DIAG_MARKER_CANDIDATE;
    }
    if (finish_gate_open(controller)) {
        flags |= H2026_Q2_DIAG_FINISH_GATE_OPEN;
    }
    if (controller->i2c_invalid_ms >
        controller->config.i2c_grace_ms) {
        flags |= H2026_Q2_DIAG_I2C_DEGRADED;
    }
    if (controller->line_unusable_ms >
        controller->config.line_grace_ms) {
        flags |= H2026_Q2_DIAG_LINE_DEGRADED;
    }
    if (controller->left_duty_saturated) {
        flags |= H2026_Q2_DIAG_LEFT_DUTY_SATURATED;
    }
    if (controller->right_duty_saturated) {
        flags |= H2026_Q2_DIAG_RIGHT_DUTY_SATURATED;
    }
    if (controller->fault_coasting) {
        flags |= H2026_Q2_DIAG_FAULT_COASTING;
    }

    output->left_signed_duty = controller->left_signed_duty;
    output->right_signed_duty = controller->right_signed_duty;
    output->brake = controller->brake;
    output->state = controller->state;
    output->fault = controller->fault;
    output->elapsed_ms = controller->elapsed_ms;
    output->state_elapsed_ms = controller->state_elapsed_ms;
    output->distance_m = controller->distance_m;
    output->stop_target_m = controller->stop_target_m;
    output->marker_edge_distance_m =
        controller->marker_edge_distance_m;
    output->stop_remaining_m =
        (controller->stop_target_m > controller->distance_m)
            ? controller->stop_target_m - controller->distance_m
            : 0.0f;
    output->center_speed_command_mps =
        controller->center_speed_command_mps;
    output->left_target_speed_mps =
        controller->left_target_speed_mps;
    output->right_target_speed_mps =
        controller->right_target_speed_mps;
    output->left_measured_speed_mps =
        controller->left_measured_speed_mps;
    output->right_measured_speed_mps =
        controller->right_measured_speed_mps;

    output->diagnostics.flags = flags;
    output->diagnostics.line = controller->line;
    output->diagnostics.marker_reference_bits =
        controller->marker_reference_bits;
    output->diagnostics.marker_reference_count =
        controller->marker_reference_count;
    output->diagnostics.marker_detection_threshold =
        controller->marker_detection_threshold;
    output->diagnostics.marker_overlap_count =
        controller->marker_overlap_count;
    output->diagnostics.marker_pattern_center =
        controller->marker_pattern_center;
    output->diagnostics.marker_edge_distance_m =
        controller->marker_edge_distance_m;
    output->diagnostics.marker_release_ms =
        controller->marker_release_ms;
    output->diagnostics.marker_confirm_ms =
        controller->marker_confirm_ms;
    output->diagnostics.i2c_invalid_ms =
        controller->i2c_invalid_ms;
    output->diagnostics.line_unusable_ms =
        controller->line_unusable_ms;
    output->diagnostics.rejected_start_count =
        controller->rejected_start_count;
    output->diagnostics.fault_coast_ms =
        controller->fault_coast_ms;
    output->diagnostics.left_encoder_delta =
        controller->left_encoder_delta;
    output->diagnostics.right_encoder_delta =
        controller->right_encoder_delta;
    output->diagnostics.raw_line_error =
        controller->last_raw_line_error;
    output->diagnostics.filtered_line_error =
        controller->filtered_line_error;
    output->diagnostics.filtered_line_derivative_per_s =
        controller->filtered_line_derivative_per_s;
    output->diagnostics.active_line_kp_mps =
        controller->active_line_kp_mps;
    output->diagnostics.active_line_kd_m =
        controller->active_line_kd_m;
    output->diagnostics.line_pd_correction_mps =
        controller->line_pd_correction_mps;
    output->diagnostics.track_curvature_1pm =
        controller->track_curvature_1pm;
    output->diagnostics.curvature_feedforward_mps =
        controller->curvature_feedforward_mps;
}

void h2026_q2_step(h2026_q2_controller_t *controller,
                   const h2026_q2_input_t *input,
                   h2026_q2_output_t *output)
{
    bool is_marker;
    bool gate_open;
    const bool was_running =
        (controller != NULL) && state_is_running(controller->state);

    if ((controller == NULL) || (input == NULL) || (output == NULL)) {
        return;
    }

    update_odometry(controller, input, was_running);
    if (input->line_i2c_valid) {
        h2026_q2_line_decode(input->line_raw_reg5,
                             controller->config.sensor_active_high,
                             controller->config.sensor_bit0_is_left,
                             controller->config.wide_min_active,
                             &controller->line);
    } else {
        /*
         * Preserve the last valid classification/centroid. The failed
         * transaction's byte is diagnostically visible but never feeds PD.
         */
        controller->line.raw_bits = input->line_raw_reg5;
    }

    if (was_running) {
        controller->elapsed_ms =
            add_tick_saturating(controller->elapsed_ms);
        controller->state_elapsed_ms =
            add_tick_saturating(controller->state_elapsed_ms);

        if (input->line_i2c_valid) {
            controller->i2c_invalid_ms = 0U;
        } else {
            controller->i2c_invalid_ms =
                add_tick_saturating(controller->i2c_invalid_ms);
        }

        is_marker = marker_candidate(controller,
                                     input->line_i2c_valid);
        if (input->line_i2c_valid) {
            const bool marker_is_safe_here =
                is_marker &&
                ((controller->state ==
                  H2026_Q2_STATE_CLEAR_START) ||
                 finish_gate_open(controller));

            if ((controller->line.classification ==
                 H2026_Q2_LINE_NORMAL) &&
                controller->line.centroid_valid) {
                controller->line_unusable_ms = 0U;
            } else if (marker_is_safe_here) {
                controller->line_unusable_ms = 0U;
            } else {
                controller->line_unusable_ms =
                    add_tick_saturating(
                        controller->line_unusable_ms);
            }
        }
    } else if (controller->state == H2026_Q2_STATE_IDLE) {
        controller->i2c_invalid_ms = 0U;
        controller->line_unusable_ms = 0U;
    } else if (controller->state == H2026_Q2_STATE_FAULT) {
        controller->state_elapsed_ms =
            add_tick_saturating(controller->state_elapsed_ms);
        if (controller->fault_coasting) {
            controller->fault_coast_ms =
                add_tick_saturating(controller->fault_coast_ms);
        }
    }

    if (input->estop_event) {
        trip_fault(controller, H2026_Q2_FAULT_ESTOP);
    } else if (state_is_running(controller->state) &&
               (controller->i2c_invalid_ms >=
                controller->config.i2c_fault_ms)) {
        trip_fault(controller, H2026_Q2_FAULT_I2C_TIMEOUT);
    } else if (state_is_running(controller->state) &&
               (controller->line_unusable_ms >=
                controller->config.line_fault_ms)) {
        trip_fault(controller, H2026_Q2_FAULT_LINE_TIMEOUT);
    } else if (state_is_running(controller->state) &&
               (controller->elapsed_ms >=
                controller->config.mission_timeout_ms)) {
        trip_fault(controller, H2026_Q2_FAULT_MISSION_TIMEOUT);
    } else if ((controller->state == H2026_Q2_STATE_STOPPING) &&
               (controller->state_elapsed_ms >=
                controller->config.stopping_timeout_ms)) {
        trip_fault(controller, H2026_Q2_FAULT_STOPPING_TIMEOUT);
    }

    is_marker = marker_candidate(controller, input->line_i2c_valid);
    gate_open = finish_gate_open(controller);

    switch (controller->state) {
    case H2026_Q2_STATE_IDLE:
        if (input->start_event) {
            if (input->line_i2c_valid &&
                (controller->line.classification ==
                 H2026_Q2_LINE_WIDE) &&
                (controller->line.block_count == 1U) &&
                (absf(pattern_center(
                     controller->line.normalized_bits)) <=
                 controller->config.marker_center_limit_normalized) &&
                (controller->line.active_count >=
                 controller->config.marker_capture_min_active)) {
                controller->marker_captured = true;
                controller->marker_reference_bits =
                    controller->line.normalized_bits;
                controller->marker_reference_count =
                    controller->line.active_count;
                controller->marker_detection_threshold =
                    marker_threshold(controller);
                controller->elapsed_ms = 0U;
                controller->distance_m = 0.0f;
                controller->stop_target_m = 0.0f;
                controller->marker_edge_distance_m = 0.0f;
                controller->marker_release_ms = 0U;
                controller->marker_confirm_ms = 0U;
                controller->stop_hold_ms = 0U;
                controller->fault = H2026_Q2_FAULT_NONE;
                controller->fault_coasting = false;
                controller->center_speed_command_mps = 0.0f;
                controller->line_filter_initialized = false;
                reset_pi(controller);
                transition_state(controller,
                                 H2026_Q2_STATE_CLEAR_START);
            } else {
                ++controller->rejected_start_count;
            }
        }
        break;

    case H2026_Q2_STATE_CLEAR_START:
        if (is_marker) {
            controller->marker_release_ms = 0U;
        } else if (input->line_i2c_valid &&
                   (controller->line.classification ==
                    H2026_Q2_LINE_NORMAL)) {
            controller->marker_release_ms =
                add_tick_saturating(controller->marker_release_ms);
        } else {
            controller->marker_release_ms = 0U;
        }
        if ((controller->marker_release_ms >=
             controller->config.marker_release_ms) &&
            (controller->distance_m >=
             controller->config.start_clear_distance_m)) {
            transition_state(controller, H2026_Q2_STATE_LAP);
        }
        break;

    case H2026_Q2_STATE_LAP:
        controller->marker_confirm_ms = 0U;
        if (gate_open && is_marker) {
            controller->marker_edge_distance_m =
                controller->distance_m;
            controller->marker_confirm_ms = H2026_Q2_TICK_MS;
            transition_state(controller,
                             H2026_Q2_STATE_FINISH_ARMED);
        }
        break;

    case H2026_Q2_STATE_FINISH_ARMED:
        if (!gate_open || !is_marker) {
            controller->marker_confirm_ms = 0U;
            controller->marker_edge_distance_m = 0.0f;
            transition_state(controller, H2026_Q2_STATE_LAP);
        } else {
            controller->marker_confirm_ms =
                add_tick_saturating(controller->marker_confirm_ms);
            if (controller->marker_confirm_ms >=
                controller->config.marker_confirm_ms) {
                controller->stop_target_m =
                    controller->marker_edge_distance_m +
                    controller->config.stop_distance_from_marker_m;
                controller->stop_hold_ms = 0U;
                transition_state(controller,
                                 H2026_Q2_STATE_STOPPING);
            }
        }
        break;

    case H2026_Q2_STATE_STOPPING:
    {
        const float position_error =
            controller->stop_target_m - controller->distance_m;

        if (position_error <
            -controller->config.stop_position_tolerance_m) {
            trip_fault(controller, H2026_Q2_FAULT_STOP_OVERSHOOT);
            break;
        }
        if ((absf(position_error) <=
             controller->config.stop_position_tolerance_m) &&
            (absf(controller->left_measured_speed_mps) <=
             controller->config.stop_speed_tolerance_mps) &&
            (absf(controller->right_measured_speed_mps) <=
             controller->config.stop_speed_tolerance_mps) &&
            (controller->center_speed_command_mps <=
             controller->config.stop_speed_tolerance_mps)) {
            controller->stop_hold_ms =
                add_tick_saturating(controller->stop_hold_ms);
        } else {
            controller->stop_hold_ms = 0U;
        }
        if (controller->stop_hold_ms >=
            controller->config.stop_hold_ms) {
            transition_state(controller, H2026_Q2_STATE_HOLD);
        }
        break;
    }

    case H2026_Q2_STATE_HOLD:
    case H2026_Q2_STATE_FAULT:
    default:
        break;
    }

    if (state_is_running(controller->state)) {
        run_motion_control(controller, input->line_i2c_valid);
    } else if (controller->state == H2026_Q2_STATE_FAULT) {
        command_fault_safe(controller);
    } else {
        command_brake(controller);
    }
    populate_output(controller, input, output);
}

const char *h2026_q2_state_name(h2026_q2_state_t state)
{
    switch (state) {
    case H2026_Q2_STATE_IDLE:
        return "IDLE";
    case H2026_Q2_STATE_CLEAR_START:
        return "CLEAR_START";
    case H2026_Q2_STATE_LAP:
        return "LAP";
    case H2026_Q2_STATE_FINISH_ARMED:
        return "FINISH_ARMED";
    case H2026_Q2_STATE_STOPPING:
        return "STOPPING";
    case H2026_Q2_STATE_HOLD:
        return "HOLD";
    case H2026_Q2_STATE_FAULT:
        return "FAULT";
    default:
        return "UNKNOWN";
    }
}

const char *h2026_q2_fault_name(h2026_q2_fault_t fault)
{
    switch (fault) {
    case H2026_Q2_FAULT_NONE:
        return "NONE";
    case H2026_Q2_FAULT_CONFIG:
        return "CONFIG";
    case H2026_Q2_FAULT_ESTOP:
        return "ESTOP";
    case H2026_Q2_FAULT_I2C_TIMEOUT:
        return "I2C_TIMEOUT";
    case H2026_Q2_FAULT_LINE_TIMEOUT:
        return "LINE_TIMEOUT";
    case H2026_Q2_FAULT_MISSION_TIMEOUT:
        return "MISSION_TIMEOUT";
    case H2026_Q2_FAULT_STOPPING_TIMEOUT:
        return "STOPPING_TIMEOUT";
    case H2026_Q2_FAULT_STOP_OVERSHOOT:
        return "STOP_OVERSHOOT";
    default:
        return "UNKNOWN";
    }
}
