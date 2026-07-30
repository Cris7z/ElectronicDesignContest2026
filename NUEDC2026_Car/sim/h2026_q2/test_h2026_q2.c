#include "h2026_q2.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

#define CHECK(condition)                                                     \
    do {                                                                     \
        if (!(condition)) {                                                  \
            (void)fprintf(stderr,                                            \
                          "%s:%d: CHECK failed: %s\n",                       \
                          __FILE__,                                          \
                          __LINE__,                                          \
                          #condition);                                       \
            return false;                                                    \
        }                                                                    \
    } while (0)

#define CHECK_NEAR(actual, expected, tolerance)                              \
    do {                                                                     \
        const double check_actual = (double)(actual);                        \
        const double check_expected = (double)(expected);                    \
        const double check_tolerance = (double)(tolerance);                  \
        if (fabs(check_actual - check_expected) > check_tolerance) {         \
            (void)fprintf(stderr,                                            \
                          "%s:%d: %.9g not within %.9g of %.9g\n",           \
                          __FILE__,                                          \
                          __LINE__,                                          \
                          check_actual,                                      \
                          check_tolerance,                                   \
                          check_expected);                                   \
            return false;                                                    \
        }                                                                    \
    } while (0)

typedef struct {
    h2026_q2_controller_t controller;
    h2026_q2_input_t input;
    h2026_q2_output_t output;
} fixture_t;

/*
 * These are deliberately synthetic host-fixture values, not measured
 * MG513XP28/chassis values and not production defaults.
 */
static h2026_q2_config_t test_config(void)
{
    h2026_q2_config_t config;

    memset(&config, 0, sizeof(config));
    config.wide_min_active = 5U;
    for (uint8_t index = 0U; index < H2026_Q2_LINE_SENSOR_COUNT; ++index) {
        config.sensor_x_mm[index] = ((float)index * 10.0f) - 35.0f;
    }
    config.marker_capture_min_active = 5U;
    config.marker_detect_min_active = 3U;
    config.marker_count_tolerance = 1U;
    config.marker_detect_ratio = 0.75f;
    config.marker_center_limit_normalized = 0.30f;
    config.marker_release_ms = 20U;
    config.marker_confirm_ms = 15U;
    config.start_clear_distance_m = 0.010f;
    config.start_acquire_timeout_ms = 100U;
    config.start_acquire_speed_mps = 0.05f;
    config.use_start_finish_marker = true;
    config.finish_gate_distance_m = 5.900f;
    config.finish_gate_time_ms = 1000U;
    config.distance_finish_approach_m = 0.10f;
    config.zero_offset_approach_speed_mps = 0.05f;
    config.stop_distance_from_marker_m = 0.200f;
    config.stop_position_tolerance_m = 0.0015f;
    config.stop_speed_tolerance_mps = 0.025f;
    config.stop_hold_ms = 20U;
    config.line_sensor_grace_ms = 10U;
    config.line_sensor_fault_ms = 30U;
    config.line_grace_ms = 10U;
    config.line_fault_ms = 40U;
    config.mission_timeout_ms = 30000U;
    config.stopping_timeout_ms = 5000U;
    config.fault_coast_max_ms = 100U;
    config.left_meters_per_encoder_count = 0.0001f;
    config.right_meters_per_encoder_count = 0.0001f;
    config.left_encoder_sign = 1;
    config.right_encoder_sign = 1;
    config.track_width_m = 0.24f;
    config.cruise_speed_mps = 0.50f;
    config.minimum_tracking_speed_mps = 0.20f;
    config.degraded_sensor_speed_mps = 0.10f;
    config.degraded_line_speed_mps = 0.08f;
    config.maximum_wheel_speed_mps = 1.20f;
    config.acceleration_limit_mps2 = 2.0f;
    config.deceleration_limit_mps2 = 2.0f;
    config.stopping_deceleration_mps2 = 2.0f;
    config.line_error_speed_reduction = 0.45f;
    config.curvature_speed_reduction_m = 0.08f;
    config.line_error_filter_alpha = 0.40f;
    config.line_derivative_filter_alpha = 0.25f;
    config.line_kp_center_mps = 0.16f;
    config.line_kp_edge_mps = 0.34f;
    config.line_kd_center_m = 0.004f;
    config.line_kd_edge_m = 0.010f;
    config.line_correction_limit_mps = 0.45f;
    config.curvature_feedforward_gain = 1.0f;
    config.curve_transition_m = 0.20f;
    config.curve_segment_count = 5U;
    config.curve_segments[0].end_distance_m = 1.0f;
    config.curve_segments[0].curvature_1pm = 0.0f;
    config.curve_segments[1].end_distance_m = 2.0f;
    config.curve_segments[1].curvature_1pm = 2.0f;
    config.curve_segments[2].end_distance_m = 3.0f;
    config.curve_segments[2].curvature_1pm = 0.0f;
    config.curve_segments[3].end_distance_m = 4.0f;
    config.curve_segments[3].curvature_1pm = -2.0f;
    config.curve_segments[4].end_distance_m = 6.5f;
    config.curve_segments[4].curvature_1pm = 0.0f;
    config.left_speed_pi.kp = 0.8f;
    config.left_speed_pi.ki = 0.6f;
    config.left_speed_pi.ks = 0.04f;
    config.left_speed_pi.kv = 0.8f;
    config.left_speed_pi.integral_limit = 0.30f;
    config.right_speed_pi = config.left_speed_pi;
    config.wheel_speed_filter_alpha = 1.0f;
    config.signed_duty_limit = 1.0f;
    return config;
}

static bool fixture_init(fixture_t *fixture,
                         const h2026_q2_config_t *config,
                         uint8_t initial_raw)
{
    memset(fixture, 0, sizeof(*fixture));
    fixture->input.line_frame.raw_bits = initial_raw;
    fixture->input.line_frame.valid = true;
    return h2026_q2_init(&fixture->controller,
                         config,
                         &fixture->input);
}

static void fixture_step(fixture_t *fixture,
                         uint8_t raw,
                         bool line_valid,
                         int64_t left_delta,
                         int64_t right_delta,
                         bool start,
                         bool estop)
{
    fixture->input.line_frame.raw_bits = raw;
    fixture->input.line_frame.valid = line_valid;
    fixture->input.encoder_left_count += left_delta;
    fixture->input.encoder_right_count += right_delta;
    fixture->input.start_event = start;
    fixture->input.estop_event = estop;
    h2026_q2_step(&fixture->controller,
                  &fixture->input,
                  &fixture->output);
    fixture->input.start_event = false;
    fixture->input.estop_event = false;
}

static bool start_and_clear(fixture_t *fixture, uint8_t start_marker)
{
    unsigned int guard;

    fixture_step(fixture, start_marker, true, 0, 0, true, false);
    CHECK(fixture->output.state == H2026_Q2_STATE_CLEAR_START);
    CHECK(!fixture->output.brake);
    for (guard = 0U;
         (guard < 100U) &&
         (fixture->output.state == H2026_Q2_STATE_CLEAR_START);
         ++guard) {
        fixture_step(fixture, 0x18U, true, 10, 10, false, false);
    }
    CHECK(fixture->output.state == H2026_Q2_STATE_LAP);
    return true;
}

static uint8_t reference_reverse8(uint8_t value)
{
    uint8_t result = 0U;
    unsigned int bit;

    for (bit = 0U; bit < 8U; ++bit) {
        if ((value & (uint8_t)(1U << bit)) != 0U) {
            result |= (uint8_t)(1U << (7U - bit));
        }
    }
    return result;
}

static uint8_t reference_popcount8(uint8_t value)
{
    uint8_t count = 0U;

    while (value != 0U) {
        count = (uint8_t)(count + (value & 1U));
        value = (uint8_t)(value >> 1U);
    }
    return count;
}

static uint8_t reference_block_count(uint8_t value)
{
    uint8_t blocks = 0U;
    bool previous = false;
    unsigned int bit;

    for (bit = 0U; bit < 8U; ++bit) {
        const bool active =
            (value & (uint8_t)(1U << bit)) != 0U;

        if (active && !previous) {
            ++blocks;
        }
        previous = active;
    }
    return blocks;
}

static h2026_q2_line_class_t reference_class(uint8_t bits)
{
    const uint8_t active = reference_popcount8(bits);
    const uint8_t blocks = reference_block_count(bits);

    if (active == 0U) {
        return H2026_Q2_LINE_LOST;
    }
    if (active == 8U) {
        return H2026_Q2_LINE_ALL;
    }
    if (blocks > 1U) {
        return H2026_Q2_LINE_MULTI;
    }
    if (active >= 5U) {
        return H2026_Q2_LINE_WIDE;
    }
    return H2026_Q2_LINE_NORMAL;
}

static float reference_centroid(uint8_t bits)
{
    int numerator = 0;
    uint8_t count = 0U;
    unsigned int bit;

    for (bit = 0U; bit < 8U; ++bit) {
        if ((bits & (uint8_t)(1U << bit)) != 0U) {
            numerator += ((int)bit * 2) - 7;
            ++count;
        }
    }
    return (float)numerator / ((float)count * 7.0f);
}

static bool test_all_256_line_patterns(void)
{
    unsigned int raw;

    for (raw = 0U; raw <= 0xFFU; ++raw) {
        const uint8_t bits = (uint8_t)raw;
        const h2026_q2_line_class_t expected = reference_class(bits);
        const bool expected_centroid =
            expected == H2026_Q2_LINE_NORMAL;
        h2026_q2_line_observation_t direct;
        h2026_q2_line_observation_t active_low;
        h2026_q2_line_observation_t reversed;

        h2026_q2_line_decode(bits, true, true, 5U, &direct);
        h2026_q2_line_decode((uint8_t)~bits,
                             false,
                             true,
                             5U,
                             &active_low);
        h2026_q2_line_decode(reference_reverse8(bits),
                             true,
                             false,
                             5U,
                             &reversed);

        CHECK(direct.normalized_bits == bits);
        CHECK(direct.active_count == reference_popcount8(bits));
        CHECK(direct.block_count == reference_block_count(bits));
        CHECK(direct.classification == expected);
        CHECK(direct.centroid_valid == expected_centroid);
        CHECK(active_low.normalized_bits == bits);
        CHECK(active_low.classification == expected);
        CHECK(active_low.centroid_valid == expected_centroid);
        CHECK(reversed.normalized_bits == bits);
        CHECK(reversed.classification == expected);
        CHECK(reversed.centroid_valid == expected_centroid);
        if (expected_centroid) {
            CHECK_NEAR(direct.centroid,
                       reference_centroid(bits),
                       0.000001f);
        }
    }
    return true;
}

static bool test_gray_calibration_crc_and_span(void)
{
    h2026_q2_line_calibration_t calibration;

    memset(&calibration, 0, sizeof(calibration));
    for (uint8_t index = 0U; index < H2026_Q2_LINE_SENSOR_COUNT; ++index) {
        calibration.white_adc[index] = 400U;
        calibration.black_adc[index] = 2600U;
        calibration.sensor_x_mm[index] = ((float)index * 10.0f) - 35.0f;
    }
    calibration.version = 1U;
    calibration.crc16 = h2026_q2_line_calibration_crc16(&calibration);
    CHECK(h2026_q2_line_calibration_valid(&calibration));
    ++calibration.black_adc[3];
    CHECK(!h2026_q2_line_calibration_valid(&calibration));
    --calibration.black_adc[3];
    calibration.crc16 = h2026_q2_line_calibration_crc16(&calibration);
    calibration.black_adc[5] = calibration.white_adc[5] + 409U;
    calibration.crc16 = h2026_q2_line_calibration_crc16(&calibration);
    CHECK(!h2026_q2_line_calibration_valid(&calibration));
    return true;
}

static bool test_gray_frame_weighted_centroid_and_binary_fallback(void)
{
    h2026_q2_config_t config = test_config();
    h2026_q2_line_sensor_frame_t frame;
    h2026_q2_line_observation_t observation;

    memset(&frame, 0, sizeof(frame));
    frame.valid = true;
    frame.raw_bits = 0x18U;
    frame.strength[3] = 200U;
    frame.strength[4] = 800U;
    h2026_q2_line_decode_frame(&frame, config.sensor_x_mm,
                               config.wide_min_active, &observation);
    CHECK(observation.classification == H2026_Q2_LINE_NORMAL);
    CHECK(observation.centroid_valid);
    CHECK_NEAR(observation.centroid, 0.08571429f, 0.00001f);

    frame.strength[3] = 0U;
    frame.strength[4] = 0U;
    h2026_q2_line_decode_frame(&frame, config.sensor_x_mm,
                               config.wide_min_active, &observation);
    CHECK_NEAR(observation.centroid, 0.0f, 0.00001f);

    frame.raw_bits = 0x24U;
    h2026_q2_line_decode_frame(&frame, config.sensor_x_mm,
                               config.wide_min_active, &observation);
    CHECK(observation.classification == H2026_Q2_LINE_MULTI);
    CHECK(!observation.centroid_valid);
    return true;
}

static bool test_config_requires_measured_values(void)
{
    h2026_q2_config_t config = test_config();
    h2026_q2_controller_t controller;
    h2026_q2_input_t input;

    memset(&input, 0, sizeof(input));
    input.line_frame.valid = true;
    CHECK(h2026_q2_config_validate(&config));

    config.left_meters_per_encoder_count = 0.0f;
    CHECK(!h2026_q2_config_validate(&config));
    CHECK(!h2026_q2_init(&controller, &config, &input));
    CHECK(controller.state == H2026_Q2_STATE_FAULT);
    CHECK(controller.fault == H2026_Q2_FAULT_CONFIG);
    CHECK(controller.brake);

    config = test_config();
    config.stop_distance_from_marker_m = 0.0f;
    CHECK(h2026_q2_config_validate(&config));

    config = test_config();
    config.left_encoder_sign = 0;
    CHECK(!h2026_q2_config_validate(&config));

    config = test_config();
    config.curve_segments[2].end_distance_m = 1.5f;
    CHECK(!h2026_q2_config_validate(&config));

    config = test_config();
    config.wide_min_active = 8U;
    CHECK(!h2026_q2_config_validate(&config));

    config = test_config();
    config.marker_capture_min_active = 8U;
    CHECK(!h2026_q2_config_validate(&config));
    return true;
}

static bool test_start_release_and_finish_gates(void)
{
    h2026_q2_config_t config = test_config();
    fixture_t fixture;
    unsigned int index;

    CHECK(fixture_init(&fixture, &config, 0x18U));

    /* A normal track line may be 10 cm before the physical start marker. */
    fixture_step(&fixture, 0x18U, true, 0, 0, true, false);
    CHECK(fixture.output.state == H2026_Q2_STATE_SEEK_START_MARKER);
    CHECK(!fixture.output.brake);
    fixture_step(&fixture, 0x3EU, true, 10, 10, false, false);
    CHECK(fixture.output.state == H2026_Q2_STATE_CLEAR_START);
    CHECK(fixture.output.diagnostics.marker_reference_bits == 0x3EU);

    /* Invalid start pictures remain rejected from a fresh idle state. */
    CHECK(fixture_init(&fixture, &config, 0x18U));

    fixture_step(&fixture, 0xF8U, true, 0, 0, true, false);
    CHECK(fixture.output.state == H2026_Q2_STATE_IDLE);
    CHECK(fixture.output.diagnostics.rejected_start_count == 1U);

    fixture_step(&fixture, 0xFFU, true, 0, 0, true, false);
    CHECK(fixture.output.state == H2026_Q2_STATE_IDLE);
    CHECK(fixture.output.diagnostics.rejected_start_count == 2U);

    fixture_step(&fixture, 0x3EU, true, 0, 0, true, false);
    CHECK(fixture.output.state == H2026_Q2_STATE_CLEAR_START);
    CHECK(fixture.output.diagnostics.marker_reference_bits == 0x3EU);
    CHECK(fixture.output.diagnostics.marker_reference_count == 5U);
    CHECK(fixture.output.diagnostics.marker_detection_threshold == 5U);

    for (index = 0U; index < 20U; ++index) {
        fixture_step(&fixture, 0x3EU, true, 10, 10, false, false);
        CHECK(fixture.output.state == H2026_Q2_STATE_CLEAR_START);
    }
    fixture_step(&fixture, 0x00U, true, 10, 10, false, false);
    CHECK(fixture.output.state == H2026_Q2_STATE_CLEAR_START);
    CHECK(fixture.output.diagnostics.marker_release_ms == 0U);
    fixture_step(&fixture, 0x81U, true, 10, 10, false, false);
    CHECK(fixture.output.state == H2026_Q2_STATE_CLEAR_START);
    CHECK(fixture.output.diagnostics.marker_release_ms == 0U);
    fixture_step(&fixture, 0x3EU, true, 10, 10, false, false);
    CHECK(fixture.output.state == H2026_Q2_STATE_CLEAR_START);
    CHECK(fixture.output.diagnostics.line_unusable_ms == 0U);

    for (index = 0U;
         (index < 50U) &&
         (fixture.output.state == H2026_Q2_STATE_CLEAR_START);
         ++index) {
        fixture_step(&fixture, 0x18U, true, 10, 10, false, false);
    }
    CHECK(fixture.output.state == H2026_Q2_STATE_LAP);

    /* Same active count but outside the expanded start reference: reject. */
    for (index = 0U; index < 3U; ++index) {
        fixture_step(&fixture, 0xF8U, true, 10, 10, false, false);
        CHECK(fixture.output.state == H2026_Q2_STATE_LAP);
        CHECK((fixture.output.diagnostics.flags &
               H2026_Q2_DIAG_MARKER_CANDIDATE) == 0U);
    }
    fixture_step(&fixture, 0xFFU, true, 10, 10, false, false);
    CHECK(fixture.output.state == H2026_Q2_STATE_LAP);
    CHECK((fixture.output.diagnostics.flags &
           H2026_Q2_DIAG_MARKER_CANDIDATE) == 0U);
    fixture_step(&fixture, 0x18U, true, 10, 10, false, false);
    CHECK(fixture.output.diagnostics.line_unusable_ms == 0U);

    /* A shifted start marker overlaps, but distance+time gate is still shut. */
    for (index = 0U; index < 3U; ++index) {
        fixture_step(&fixture, 0x7CU, true, 10, 10, false, false);
        CHECK(fixture.output.state == H2026_Q2_STATE_LAP);
        CHECK((fixture.output.diagnostics.flags &
               H2026_Q2_DIAG_MARKER_CANDIDATE) != 0U);
        CHECK((fixture.output.diagnostics.flags &
               H2026_Q2_DIAG_FINISH_GATE_OPEN) == 0U);
    }
    return true;
}

static bool test_odometry_only_start_and_finish(void)
{
    h2026_q2_config_t config = test_config();
    fixture_t fixture;

    config.use_start_finish_marker = false;
    config.finish_gate_distance_m = 0.030f;
    config.finish_gate_time_ms = 5U;
    config.distance_finish_approach_m = 0.010f;
    CHECK(fixture_init(&fixture, &config, 0x18U));

    /* Wheel centre is on the start datum; a normal line starts immediately. */
    fixture_step(&fixture, 0x18U, true, 0, 0, true, false);
    CHECK(fixture.output.state == H2026_Q2_STATE_START_ACQUIRE_LINE);
    CHECK(!fixture.output.brake);
    CHECK((fixture.output.diagnostics.flags &
           H2026_Q2_DIAG_MARKER_CAPTURED) == 0U);

    fixture_step(&fixture, 0x18U, true, 0, 0, false, false);
    CHECK(fixture.output.state == H2026_Q2_STATE_LAP);

    fixture_step(&fixture, 0x18U, true, 100, 100, false, false);
    CHECK(fixture.output.state == H2026_Q2_STATE_LAP);
    fixture_step(&fixture, 0x18U, true, 100, 100, false, false);
    CHECK(fixture.output.state == H2026_Q2_STATE_LAP);
    fixture_step(&fixture, 0x18U, true, 100, 100, false, false);
    CHECK(fixture.output.state == H2026_Q2_STATE_STOPPING);
    CHECK_NEAR(fixture.controller.stop_target_m, 0.030f, 0.000001f);
    return true;
}

static bool advance_to_distance(fixture_t *fixture,
                                float target_distance_m,
                                uint8_t line_bits)
{
    const h2026_q2_config_t *config = &fixture->controller.config;
    const int64_t target_count =
        (int64_t)llround((double)target_distance_m /
                         (double)config->left_meters_per_encoder_count);
    unsigned int guard = 0U;

    while ((fixture->input.encoder_left_count < target_count) &&
           (guard < 10000U)) {
        const int64_t remaining =
            target_count - fixture->input.encoder_left_count;
        const int64_t delta = (remaining > 25) ? 25 : remaining;

        fixture_step(fixture,
                     line_bits,
                     true,
                     delta,
                     delta,
                     false,
                     false);
        CHECK(fixture->output.state == H2026_Q2_STATE_LAP);
        ++guard;
    }
    CHECK(guard < 10000U);
    return true;
}

static bool test_open_gate_rejects_bad_marker_shapes(void)
{
    const uint8_t bad_markers[] = {0xFFU, 0x5DU, 0xF8U};
    h2026_q2_config_t config = test_config();
    fixture_t fixture;
    size_t index;

    CHECK(fixture_init(&fixture, &config, 0x18U));
    CHECK(start_and_clear(&fixture, 0x3EU));
    CHECK(advance_to_distance(&fixture, 6.1416f, 0x18U));
    CHECK((fixture.output.diagnostics.flags &
           H2026_Q2_DIAG_FINISH_GATE_OPEN) != 0U);

    for (index = 0U;
         index < sizeof(bad_markers) / sizeof(bad_markers[0]);
         ++index) {
        fixture_step(&fixture,
                     bad_markers[index],
                     true,
                     10,
                     10,
                     false,
                     false);
        CHECK(fixture.output.state == H2026_Q2_STATE_LAP);
        CHECK((fixture.output.diagnostics.flags &
               H2026_Q2_DIAG_MARKER_CANDIDATE) == 0U);
        fixture_step(&fixture, 0x18U, true, 10, 10, false, false);
        CHECK(fixture.output.diagnostics.line_unusable_ms == 0U);
    }
    CHECK(fixture.output.diagnostics.line.classification ==
          H2026_Q2_LINE_NORMAL);
    return true;
}

static bool enter_stopping_after_full_lap(fixture_t *fixture,
                                          h2026_q2_config_t *config)
{
    CHECK(fixture_init(fixture, config, 0x18U));
    CHECK(start_and_clear(fixture, 0x3EU));
    CHECK(advance_to_distance(fixture, 6.1416f, 0x18U));
    CHECK_NEAR(fixture->output.distance_m, 6.1416f, 0.001f);
    CHECK(fixture->output.elapsed_ms > config->finish_gate_time_ms);

    fixture_step(fixture, 0x7CU, true, 0, 0, false, false);
    CHECK(fixture->output.state == H2026_Q2_STATE_FINISH_ARMED);
    CHECK_NEAR(fixture->output.marker_edge_distance_m,
               6.1416f,
               0.001f);
    fixture_step(fixture, 0x7CU, true, 10, 10, false, false);
    CHECK(fixture->output.state == H2026_Q2_STATE_FINISH_ARMED);
    fixture_step(fixture, 0x7CU, true, 10, 10, false, false);
    CHECK(fixture->output.state == H2026_Q2_STATE_STOPPING);
    CHECK_NEAR(fixture->output.stop_target_m, 6.3416f, 0.001f);
    return true;
}

static bool test_full_6_1416m_lap_and_distance_stop(void)
{
    h2026_q2_config_t config = test_config();
    fixture_t fixture;
    double fractional_counts = 0.0;
    unsigned int guard;

    CHECK(enter_stopping_after_full_lap(&fixture, &config));
    for (guard = 0U;
         (guard < 3000U) &&
         (fixture.output.state == H2026_Q2_STATE_STOPPING);
         ++guard) {
        int64_t delta;

        fractional_counts +=
            ((double)fixture.output.center_speed_command_mps *
             (double)H2026_Q2_TICK_S) /
            (double)config.left_meters_per_encoder_count;
        delta = (int64_t)floor(fractional_counts);
        fractional_counts -= (double)delta;
        fixture_step(&fixture,
                     0x18U,
                     true,
                     delta,
                     delta,
                     false,
                     false);
    }

    CHECK(guard < 3000U);
    CHECK(fixture.output.state == H2026_Q2_STATE_HOLD);
    CHECK(fixture.output.brake);
    CHECK_NEAR(fixture.output.left_signed_duty, 0.0f, 0.000001f);
    CHECK_NEAR(fixture.output.right_signed_duty, 0.0f, 0.000001f);
    CHECK(fabsf(fixture.output.distance_m -
                fixture.output.stop_target_m) <=
          config.stop_position_tolerance_m +
              config.left_meters_per_encoder_count);
    return true;
}

static bool test_line_sensor_timeout_and_degraded_cap(void)
{
    h2026_q2_config_t config = test_config();
    fixture_t fixture;
    unsigned int tick;
    float previous_speed;

    CHECK(fixture_init(&fixture, &config, 0x18U));
    CHECK(start_and_clear(&fixture, 0x3EU));
    previous_speed = fixture.output.center_speed_command_mps;
    for (tick = 1U;
         tick <= config.line_sensor_fault_ms / H2026_Q2_TICK_MS;
         ++tick) {
        fixture_step(&fixture, 0xFFU, false, 25, 25, false, false);
        if ((tick * H2026_Q2_TICK_MS) > config.line_sensor_grace_ms &&
            fixture.output.state != H2026_Q2_STATE_FAULT) {
            CHECK((fixture.output.diagnostics.flags &
                   H2026_Q2_DIAG_LINE_SENSOR_DEGRADED) != 0U);
            CHECK(fixture.output.center_speed_command_mps <=
                  previous_speed + 0.000001f);
        }
        previous_speed = fixture.output.center_speed_command_mps;
    }
    CHECK(fixture.output.state == H2026_Q2_STATE_FAULT);
    CHECK(fixture.output.fault == H2026_Q2_FAULT_LINE_SENSOR_TIMEOUT);
    CHECK(!fixture.output.brake);
    CHECK((fixture.output.diagnostics.flags &
           H2026_Q2_DIAG_FAULT_COASTING) != 0U);
    CHECK_NEAR(fixture.output.left_signed_duty, 0.0f, 0.000001f);
    CHECK_NEAR(fixture.output.right_signed_duty, 0.0f, 0.000001f);
    fixture_step(&fixture, 0x18U, true, 0, 0, false, false);
    CHECK(fixture.output.brake);
    CHECK((fixture.output.diagnostics.flags &
           H2026_Q2_DIAG_FAULT_COASTING) == 0U);
    return true;
}

static bool test_invalid_and_non_normal_never_feed_pd(void)
{
    h2026_q2_config_t config = test_config();
    fixture_t fixture;
    float held_error;

    CHECK(fixture_init(&fixture, &config, 0x18U));
    CHECK(start_and_clear(&fixture, 0x3EU));
    fixture_step(&fixture, 0x80U, true, 0, 0, false, false);
    held_error = fixture.output.diagnostics.filtered_line_error;
    CHECK(held_error > 0.0f);

    fixture_step(&fixture, 0x01U, false, 0, 0, false, false);
    CHECK(fixture.output.diagnostics.line.raw_bits == 0x01U);
    CHECK(fixture.output.diagnostics.line.normalized_bits == 0x80U);
    CHECK_NEAR(fixture.output.diagnostics.filtered_line_error,
               held_error,
               0.000001f);

    fixture_step(&fixture, 0x7CU, true, 0, 0, false, false);
    CHECK(fixture.output.diagnostics.line.classification ==
          H2026_Q2_LINE_WIDE);
    CHECK(!fixture.output.diagnostics.line.centroid_valid);
    CHECK_NEAR(fixture.output.diagnostics.filtered_line_error,
               held_error,
               0.000001f);
    CHECK(fixture.output.diagnostics.line_unusable_ms ==
          H2026_Q2_TICK_MS);

    fixture_step(&fixture, 0x81U, true, 0, 0, false, false);
    CHECK(fixture.output.diagnostics.line.classification ==
          H2026_Q2_LINE_MULTI);
    CHECK(!fixture.output.diagnostics.line.centroid_valid);
    CHECK_NEAR(fixture.output.diagnostics.filtered_line_error,
               held_error,
               0.000001f);
    CHECK(fixture.output.diagnostics.line_unusable_ms ==
          2U * H2026_Q2_TICK_MS);
    return true;
}

static bool test_line_loss_recovery_and_timeout(void)
{
    h2026_q2_config_t config = test_config();
    fixture_t fixture;
    unsigned int tick;

    CHECK(fixture_init(&fixture, &config, 0x18U));
    CHECK(start_and_clear(&fixture, 0x3EU));

    for (tick = 0U; tick < 3U; ++tick) {
        fixture_step(&fixture, 0x00U, true, 0, 0, false, false);
    }
    CHECK(fixture.output.state == H2026_Q2_STATE_LAP);
    CHECK(fixture.output.diagnostics.line.classification ==
          H2026_Q2_LINE_LOST);
    CHECK((fixture.output.diagnostics.flags &
           H2026_Q2_DIAG_LINE_DEGRADED) != 0U);

    fixture_step(&fixture, 0x18U, true, 0, 0, false, false);
    CHECK(fixture.output.diagnostics.line_unusable_ms == 0U);

    for (tick = 1U;
         tick <= config.line_fault_ms / H2026_Q2_TICK_MS;
         ++tick) {
        fixture_step(&fixture, 0x00U, true, 0, 0, false, false);
    }
    CHECK(fixture.output.state == H2026_Q2_STATE_FAULT);
    CHECK(fixture.output.fault == H2026_Q2_FAULT_LINE_TIMEOUT);
    CHECK(fixture.output.brake);
    return true;
}

static bool test_mission_and_stopping_timeouts(void)
{
    h2026_q2_config_t mission_config = test_config();
    h2026_q2_config_t stopping_config = test_config();
    fixture_t fixture;
    unsigned int guard;

    mission_config.start_clear_distance_m = 0.001f;
    mission_config.finish_gate_distance_m = 100.0f;
    mission_config.finish_gate_time_ms = 50U;
    mission_config.mission_timeout_ms = 120U;
    CHECK(h2026_q2_config_validate(&mission_config));
    CHECK(fixture_init(&fixture, &mission_config, 0x18U));
    CHECK(start_and_clear(&fixture, 0x3EU));
    for (guard = 0U;
         (guard < 100U) &&
         (fixture.output.state != H2026_Q2_STATE_FAULT);
         ++guard) {
        fixture_step(&fixture, 0x18U, true, 0, 0, false, false);
    }
    CHECK(fixture.output.fault == H2026_Q2_FAULT_MISSION_TIMEOUT);

    stopping_config.stopping_timeout_ms = 50U;
    CHECK(enter_stopping_after_full_lap(&fixture, &stopping_config));
    for (guard = 0U;
         (guard < 100U) &&
         (fixture.output.state != H2026_Q2_STATE_FAULT);
         ++guard) {
        fixture_step(&fixture, 0x18U, true, 0, 0, false, false);
    }
    CHECK(fixture.output.fault == H2026_Q2_FAULT_STOPPING_TIMEOUT);
    CHECK(fixture.output.brake);
    return true;
}

static bool test_estop_is_immediate(void)
{
    h2026_q2_config_t config = test_config();
    fixture_t fixture;

    CHECK(fixture_init(&fixture, &config, 0x18U));
    CHECK(start_and_clear(&fixture, 0x3EU));
    fixture_step(&fixture, 0x18U, true, 0, 0, false, true);
    CHECK(fixture.output.state == H2026_Q2_STATE_FAULT);
    CHECK(fixture.output.fault == H2026_Q2_FAULT_ESTOP);
    CHECK(fixture.output.brake);
    return true;
}

static bool test_stop_overshoot_is_fault(void)
{
    h2026_q2_config_t config = test_config();
    fixture_t fixture;
    float excess_distance;
    int64_t excess_counts;

    CHECK(enter_stopping_after_full_lap(&fixture, &config));
    excess_distance =
        (fixture.output.stop_target_m - fixture.output.distance_m) +
        config.stop_position_tolerance_m + 0.010f;
    excess_counts =
        (int64_t)ceilf(excess_distance /
                       config.left_meters_per_encoder_count);
    fixture_step(&fixture,
                 0x18U,
                 true,
                 excess_counts,
                 excess_counts,
                 false,
                 false);
    CHECK(fixture.output.state == H2026_Q2_STATE_FAULT);
    CHECK(fixture.output.fault == H2026_Q2_FAULT_STOP_OVERSHOOT);
    CHECK(!fixture.output.brake);
    return true;
}

static bool test_stopping_turn_fades_to_zero(void)
{
    h2026_q2_config_t config = test_config();
    fixture_t fixture;
    float remaining;
    int64_t remaining_counts;
    unsigned int guard;

    CHECK(enter_stopping_after_full_lap(&fixture, &config));
    remaining =
        fixture.output.stop_target_m - fixture.output.distance_m;
    remaining_counts =
        (int64_t)llround((double)remaining /
                         (double)config.left_meters_per_encoder_count);
    fixture_step(&fixture,
                 0x80U,
                 true,
                 remaining_counts,
                 remaining_counts,
                 false,
                 false);
    CHECK(fixture.output.state == H2026_Q2_STATE_STOPPING);
    CHECK_NEAR(fixture.output.left_target_speed_mps,
               fixture.output.right_target_speed_mps,
               0.000001f);

    for (guard = 0U;
         (guard < 200U) &&
         (fixture.output.state == H2026_Q2_STATE_STOPPING);
         ++guard) {
        fixture_step(&fixture, 0x80U, true, 0, 0, false, false);
        if (fixture.output.state == H2026_Q2_STATE_STOPPING) {
            CHECK_NEAR(fixture.output.left_target_speed_mps,
                       fixture.output.right_target_speed_mps,
                       0.000001f);
        }
    }
    CHECK(guard < 200U);
    CHECK(fixture.output.state == H2026_Q2_STATE_HOLD);
    CHECK_NEAR(fixture.output.left_target_speed_mps,
               0.0f,
               0.000001f);
    CHECK_NEAR(fixture.output.right_target_speed_mps,
               0.0f,
               0.000001f);
    return true;
}

static bool test_fault_coast_has_hard_deadline(void)
{
    h2026_q2_config_t config = test_config();
    fixture_t fixture;
    unsigned int tick;

    config.fault_coast_max_ms = 20U;
    CHECK(fixture_init(&fixture, &config, 0x18U));
    CHECK(start_and_clear(&fixture, 0x3EU));
    for (tick = 0U;
         tick < config.line_sensor_fault_ms / H2026_Q2_TICK_MS;
         ++tick) {
        fixture_step(&fixture, 0xFFU, false, 25, 25, false, false);
    }
    CHECK(fixture.output.state == H2026_Q2_STATE_FAULT);
    CHECK(!fixture.output.brake);

    for (tick = H2026_Q2_TICK_MS;
         tick < config.fault_coast_max_ms;
         tick += H2026_Q2_TICK_MS) {
        fixture_step(&fixture, 0x18U, true, 25, 25, false, false);
        CHECK(!fixture.output.brake);
    }
    fixture_step(&fixture, 0x18U, true, 25, 25, false, false);
    CHECK(fixture.output.brake);
    CHECK(fixture.output.diagnostics.fault_coast_ms ==
          config.fault_coast_max_ms);
    return true;
}

static bool test_independent_wheel_distance_conversion(void)
{
    h2026_q2_config_t config = test_config();
    fixture_t fixture;

    config.left_meters_per_encoder_count = 0.0001f;
    config.right_meters_per_encoder_count = 0.0002f;
    CHECK(fixture_init(&fixture, &config, 0x3EU));
    fixture_step(&fixture, 0x3EU, true, 0, 0, true, false);
    fixture_step(&fixture, 0x18U, true, 10, 10, false, false);
    CHECK_NEAR(fixture.output.distance_m, 0.0015f, 0.000001f);
    CHECK_NEAR(fixture.output.left_measured_speed_mps,
               0.2f,
               0.00001f);
    CHECK_NEAR(fixture.output.right_measured_speed_mps,
               0.4f,
               0.00001f);
    return true;
}

static bool test_wheel_limit_preserves_turn_ratio(void)
{
    h2026_q2_config_t config = test_config();
    fixture_t fixture;
    float raw_left;
    float raw_right;
    float peak;
    float scale;

    config.maximum_wheel_speed_mps = config.cruise_speed_mps;
    config.line_kp_center_mps = 1.0f;
    config.line_kp_edge_mps = 1.0f;
    config.line_kd_center_m = 0.0f;
    config.line_kd_edge_m = 0.0f;
    config.line_correction_limit_mps = 0.45f;
    CHECK(fixture_init(&fixture, &config, 0x18U));
    CHECK(start_and_clear(&fixture, 0x3EU));
    fixture_step(&fixture, 0x80U, true, 0, 0, false, false);

    raw_left =
        fixture.output.center_speed_command_mps +
        fixture.output.diagnostics.line_pd_correction_mps +
        fixture.output.diagnostics.curvature_feedforward_mps;
    raw_right =
        fixture.output.center_speed_command_mps -
        fixture.output.diagnostics.line_pd_correction_mps -
        fixture.output.diagnostics.curvature_feedforward_mps;
    peak = fmaxf(fabsf(raw_left), fabsf(raw_right));
    CHECK(peak > config.maximum_wheel_speed_mps);
    scale = config.maximum_wheel_speed_mps / peak;
    CHECK_NEAR(fixture.output.left_target_speed_mps,
               raw_left * scale,
               0.000001f);
    CHECK_NEAR(fixture.output.right_target_speed_mps,
               raw_right * scale,
               0.000001f);
    return true;
}

static bool test_variable_pd_curvature_and_independent_pi(void)
{
    h2026_q2_config_t config = test_config();
    fixture_t fixture;
    float center_kp;

    config.right_speed_pi.kv = 0.5f;
    CHECK(fixture_init(&fixture, &config, 0x18U));
    CHECK(start_and_clear(&fixture, 0x3EU));

    fixture_step(&fixture, 0x18U, true, 0, 0, false, false);
    center_kp = fixture.output.diagnostics.active_line_kp_mps;
    fixture_step(&fixture, 0x80U, true, 0, 0, false, false);
    CHECK(fixture.output.diagnostics.filtered_line_error > 0.0f);
    CHECK(fixture.output.diagnostics.active_line_kp_mps > center_kp);
    CHECK(fixture.output.left_target_speed_mps >
          fixture.output.right_target_speed_mps);
    CHECK(fixture.output.left_signed_duty !=
          fixture.output.right_signed_duty);

    /*
     * Jumping odometry is acceptable in this pure-core unit test: it probes
     * distance-indexed smoothstep values, not a physical plant.
     */
    fixture_step(&fixture, 0x18U, true, 8890, 8890, false, false);
    CHECK_NEAR(fixture.output.distance_m, 0.900f, 0.002f);
    CHECK_NEAR(fixture.output.diagnostics.track_curvature_1pm,
               0.0f,
               0.01f);
    fixture_step(&fixture, 0x18U, true, 1000, 1000, false, false);
    CHECK_NEAR(fixture.output.distance_m, 1.000f, 0.002f);
    CHECK_NEAR(fixture.output.diagnostics.track_curvature_1pm,
               1.0f,
               0.02f);
    fixture_step(&fixture, 0x18U, true, 1000, 1000, false, false);
    CHECK_NEAR(fixture.output.distance_m, 1.100f, 0.002f);
    CHECK_NEAR(fixture.output.diagnostics.track_curvature_1pm,
               2.0f,
               0.02f);
    fixture_step(&fixture, 0x18U, true, 55000, 55000, false, false);
    CHECK(fixture.output.distance_m > 6.5f);
    CHECK_NEAR(fixture.output.diagnostics.track_curvature_1pm,
               0.0f,
               0.02f);
    return true;
}

static bool test_pi_anti_windup(void)
{
    h2026_q2_config_t config = test_config();
    fixture_t fixture;
    unsigned int tick;

    config.signed_duty_limit = 0.20f;
    config.left_speed_pi.kp = 4.0f;
    config.left_speed_pi.ki = 2.0f;
    config.left_speed_pi.ks = 0.0f;
    config.left_speed_pi.kv = 0.0f;
    config.right_speed_pi = config.left_speed_pi;
    CHECK(fixture_init(&fixture, &config, 0x18U));
    CHECK(start_and_clear(&fixture, 0x3EU));
    fixture.controller.left_pi.integral = 0.0f;
    fixture.controller.right_pi.integral = 0.0f;
    for (tick = 0U; tick < 50U; ++tick) {
        fixture_step(&fixture, 0x18U, true, 0, 0, false, false);
        CHECK(fixture.output.left_signed_duty <=
              config.signed_duty_limit);
        CHECK(fixture.output.right_signed_duty <=
              config.signed_duty_limit);
    }
    CHECK_NEAR(fixture.controller.left_pi.integral, 0.0f, 0.000001f);
    CHECK_NEAR(fixture.controller.right_pi.integral, 0.0f, 0.000001f);
    CHECK((fixture.output.diagnostics.flags &
           H2026_Q2_DIAG_LEFT_DUTY_SATURATED) != 0U);
    CHECK((fixture.output.diagnostics.flags &
           H2026_Q2_DIAG_RIGHT_DUTY_SATURATED) != 0U);
    fixture_step(&fixture, 0x18U, true, 27, 27, false, false);
    CHECK(fixture.output.left_signed_duty < 0.0f);
    CHECK(fixture.output.right_signed_duty < 0.0f);
    CHECK((fixture.output.diagnostics.flags &
           H2026_Q2_DIAG_LEFT_DUTY_SATURATED) == 0U);
    CHECK((fixture.output.diagnostics.flags &
           H2026_Q2_DIAG_RIGHT_DUTY_SATURATED) == 0U);
    CHECK(fixture.controller.left_pi.integral < 0.0f);
    CHECK(fixture.controller.right_pi.integral < 0.0f);
    return true;
}

typedef bool (*test_function_t)(void);

typedef struct {
    const char *name;
    test_function_t function;
} test_case_t;

int main(void)
{
    const test_case_t tests[] = {
        {"all 256 line patterns", test_all_256_line_patterns},
        {"gray calibration CRC and span", test_gray_calibration_crc_and_span},
        {"gray weighted centroid and binary fallback",
         test_gray_frame_weighted_centroid_and_binary_fallback},
        {"config requires measured values",
         test_config_requires_measured_values},
        {"start release and finish gates",
         test_start_release_and_finish_gates},
        {"odometry-only start and finish",
         test_odometry_only_start_and_finish},
        {"open gate rejects bad marker shapes",
         test_open_gate_rejects_bad_marker_shapes},
        {"full 6.1416 m lap and distance stop",
         test_full_6_1416m_lap_and_distance_stop},
        {"line-sensor timeout and degraded cap",
          test_line_sensor_timeout_and_degraded_cap},
        {"invalid and non-normal do not feed PD",
         test_invalid_and_non_normal_never_feed_pd},
        {"line loss recovery and timeout",
         test_line_loss_recovery_and_timeout},
        {"mission and stopping timeouts",
         test_mission_and_stopping_timeouts},
        {"immediate emergency stop", test_estop_is_immediate},
        {"stop overshoot is fault", test_stop_overshoot_is_fault},
        {"stopping turn fades to zero",
         test_stopping_turn_fades_to_zero},
        {"fault coast has hard deadline",
         test_fault_coast_has_hard_deadline},
        {"independent wheel distance conversion",
         test_independent_wheel_distance_conversion},
        {"wheel limiting preserves turn ratio",
         test_wheel_limit_preserves_turn_ratio},
        {"variable PD, curvature, independent PI",
         test_variable_pd_curvature_and_independent_pi},
        {"PI anti-windup", test_pi_anti_windup}
    };
    const size_t test_count = sizeof(tests) / sizeof(tests[0]);
    size_t index;

    for (index = 0U; index < test_count; ++index) {
        if (!tests[index].function()) {
            (void)fprintf(stderr, "[FAIL] %s\n", tests[index].name);
            return 1;
        }
        (void)printf("[PASS] %s\n", tests[index].name);
    }
    (void)printf("%zu tests passed\n", test_count);
    return 0;
}
