#ifndef H2026_Q2_CORE_H
#define H2026_Q2_CORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define H2026_Q2_TICK_MS 5U
#define H2026_Q2_TICK_S 0.005f
#define H2026_Q2_MAX_CURVE_SEGMENTS 16U

typedef enum {
    H2026_Q2_LINE_NORMAL = 0,
    H2026_Q2_LINE_LOST,
    H2026_Q2_LINE_WIDE,
    H2026_Q2_LINE_ALL,
    H2026_Q2_LINE_MULTI
} h2026_q2_line_class_t;

typedef enum {
    H2026_Q2_STATE_IDLE = 0,
    H2026_Q2_STATE_CLEAR_START,
    H2026_Q2_STATE_LAP,
    H2026_Q2_STATE_FINISH_ARMED,
    H2026_Q2_STATE_STOPPING,
    H2026_Q2_STATE_HOLD,
    H2026_Q2_STATE_FAULT
} h2026_q2_state_t;

typedef enum {
    H2026_Q2_FAULT_NONE = 0,
    H2026_Q2_FAULT_CONFIG,
    H2026_Q2_FAULT_ESTOP,
    H2026_Q2_FAULT_I2C_TIMEOUT,
    H2026_Q2_FAULT_LINE_TIMEOUT,
    H2026_Q2_FAULT_MISSION_TIMEOUT,
    H2026_Q2_FAULT_STOPPING_TIMEOUT,
    H2026_Q2_FAULT_STOP_OVERSHOOT
} h2026_q2_fault_t;

typedef struct {
    uint8_t raw_bits;
    uint8_t normalized_bits;
    uint8_t active_count;
    uint8_t block_count;
    h2026_q2_line_class_t classification;
    bool centroid_valid;
    float centroid;
} h2026_q2_line_observation_t;

typedef struct {
    float end_distance_m;
    /*
     * Positive curvature means a right turn. The core then asks the left
     * wheel to run faster than the right wheel.
     */
    float curvature_1pm;
} h2026_q2_curve_segment_t;

typedef struct {
    float kp;
    float ki;
    float ks;
    float kv;
    float integral_limit;
} h2026_q2_speed_pi_config_t;

typedef struct {
    /* HiWonder register-5 normalization; both facts must be measured. */
    bool sensor_active_high;
    bool sensor_bit0_is_left;
    uint8_t wide_min_active;

    /* Marker capture/detection is learned from the starting marker. */
    uint8_t marker_capture_min_active;
    uint8_t marker_detect_min_active;
    uint8_t marker_count_tolerance;
    float marker_detect_ratio;
    float marker_center_limit_normalized;
    uint32_t marker_release_ms;
    uint32_t marker_confirm_ms;
    float start_clear_distance_m;
    float finish_gate_distance_m;
    uint32_t finish_gate_time_ms;
    float stop_distance_from_marker_m;
    float stop_position_tolerance_m;
    float stop_speed_tolerance_mps;
    uint32_t stop_hold_ms;

    /* Safety timeouts. Every duration must be a multiple of 5 ms. */
    uint32_t i2c_grace_ms;
    uint32_t i2c_fault_ms;
    uint32_t line_grace_ms;
    uint32_t line_fault_ms;
    uint32_t mission_timeout_ms;
    uint32_t stopping_timeout_ms;
    uint32_t fault_coast_max_ms;

    /*
     * Physical conversion values. No production defaults are supplied:
     * encoder polarities and metres/count must come from chassis measurement.
     */
    float left_meters_per_encoder_count;
    float right_meters_per_encoder_count;
    int8_t left_encoder_sign;
    int8_t right_encoder_sign;
    float track_width_m;

    /* Centre-speed scheduling and bounded acceleration/deceleration. */
    float cruise_speed_mps;
    float minimum_tracking_speed_mps;
    float degraded_i2c_speed_mps;
    float degraded_line_speed_mps;
    float maximum_wheel_speed_mps;
    float acceleration_limit_mps2;
    float deceleration_limit_mps2;
    float stopping_deceleration_mps2;
    float line_error_speed_reduction;
    float curvature_speed_reduction_m;

    /* Filtered, variable-gain line PD; output is differential speed in m/s. */
    float line_error_filter_alpha;
    float line_derivative_filter_alpha;
    float line_kp_center_mps;
    float line_kp_edge_mps;
    float line_kd_center_m;
    float line_kd_edge_m;
    float line_correction_limit_mps;

    /* Distance-indexed, smoothly blended curvature feed-forward. */
    float curvature_feedforward_gain;
    float curve_transition_m;
    size_t curve_segment_count;
    h2026_q2_curve_segment_t curve_segments[H2026_Q2_MAX_CURVE_SEGMENTS];

    /* Independent wheel loops, including static and velocity feed-forward. */
    h2026_q2_speed_pi_config_t left_speed_pi;
    h2026_q2_speed_pi_config_t right_speed_pi;
    float wheel_speed_filter_alpha;
    float signed_duty_limit;
} h2026_q2_config_t;

typedef struct {
    uint8_t line_raw_reg5;
    bool line_i2c_valid;
    int64_t encoder_left_count;
    int64_t encoder_right_count;
    bool start_event;
    bool estop_event;
} h2026_q2_input_t;

enum {
    H2026_Q2_DIAG_LINE_SAMPLE_VALID = 1UL << 0,
    H2026_Q2_DIAG_CENTROID_VALID = 1UL << 1,
    H2026_Q2_DIAG_MARKER_CAPTURED = 1UL << 2,
    H2026_Q2_DIAG_MARKER_CANDIDATE = 1UL << 3,
    H2026_Q2_DIAG_FINISH_GATE_OPEN = 1UL << 4,
    H2026_Q2_DIAG_I2C_DEGRADED = 1UL << 5,
    H2026_Q2_DIAG_LINE_DEGRADED = 1UL << 6,
    H2026_Q2_DIAG_LEFT_DUTY_SATURATED = 1UL << 7,
    H2026_Q2_DIAG_RIGHT_DUTY_SATURATED = 1UL << 8,
    H2026_Q2_DIAG_FAULT_COASTING = 1UL << 9
};

typedef struct {
    uint32_t flags;
    h2026_q2_line_observation_t line;
    uint8_t marker_reference_bits;
    uint8_t marker_reference_count;
    uint8_t marker_detection_threshold;
    uint8_t marker_overlap_count;
    float marker_pattern_center;
    float marker_edge_distance_m;
    uint32_t marker_release_ms;
    uint32_t marker_confirm_ms;
    uint32_t i2c_invalid_ms;
    uint32_t line_unusable_ms;
    uint32_t rejected_start_count;
    uint32_t fault_coast_ms;
    int64_t left_encoder_delta;
    int64_t right_encoder_delta;
    float raw_line_error;
    float filtered_line_error;
    float filtered_line_derivative_per_s;
    float active_line_kp_mps;
    float active_line_kd_m;
    float line_pd_correction_mps;
    float track_curvature_1pm;
    float curvature_feedforward_mps;
} h2026_q2_diagnostics_t;

typedef struct {
    float left_signed_duty;
    float right_signed_duty;
    bool brake;
    h2026_q2_state_t state;
    h2026_q2_fault_t fault;
    uint32_t elapsed_ms;
    uint32_t state_elapsed_ms;
    float distance_m;
    float stop_target_m;
    float marker_edge_distance_m;
    float stop_remaining_m;
    float center_speed_command_mps;
    float left_target_speed_mps;
    float right_target_speed_mps;
    float left_measured_speed_mps;
    float right_measured_speed_mps;
    h2026_q2_diagnostics_t diagnostics;
} h2026_q2_output_t;

typedef struct {
    float integral;
} h2026_q2_pi_state_t;

/*
 * Public state is intentionally concrete so firmware can allocate it
 * statically. Application code must otherwise treat its fields as private.
 */
typedef struct {
    h2026_q2_config_t config;
    h2026_q2_state_t state;
    h2026_q2_fault_t fault;
    uint32_t elapsed_ms;
    uint32_t state_elapsed_ms;
    uint32_t i2c_invalid_ms;
    uint32_t line_unusable_ms;
    uint32_t marker_release_ms;
    uint32_t marker_confirm_ms;
    uint32_t stop_hold_ms;
    uint32_t rejected_start_count;
    uint32_t fault_coast_ms;
    int64_t previous_left_count;
    int64_t previous_right_count;
    int64_t left_encoder_delta;
    int64_t right_encoder_delta;
    float distance_m;
    float stop_target_m;
    float marker_edge_distance_m;
    float left_measured_speed_mps;
    float right_measured_speed_mps;
    float center_speed_command_mps;
    float filtered_line_error;
    float previous_filtered_line_error;
    float filtered_line_derivative_per_s;
    float last_raw_line_error;
    float active_line_kp_mps;
    float active_line_kd_m;
    float line_pd_correction_mps;
    float track_curvature_1pm;
    float curvature_feedforward_mps;
    bool line_filter_initialized;
    bool marker_captured;
    uint8_t marker_reference_bits;
    uint8_t marker_reference_count;
    uint8_t marker_detection_threshold;
    uint8_t marker_overlap_count;
    float marker_pattern_center;
    h2026_q2_line_observation_t line;
    h2026_q2_pi_state_t left_pi;
    h2026_q2_pi_state_t right_pi;
    float left_target_speed_mps;
    float right_target_speed_mps;
    float left_signed_duty;
    float right_signed_duty;
    bool left_duty_saturated;
    bool right_duty_saturated;
    bool fault_coasting;
    bool brake;
} h2026_q2_controller_t;

bool h2026_q2_config_validate(const h2026_q2_config_t *config);

void h2026_q2_line_decode(uint8_t raw_bits,
                          bool active_high,
                          bool bit0_is_left,
                          uint8_t wide_min_active,
                          h2026_q2_line_observation_t *observation);

bool h2026_q2_init(h2026_q2_controller_t *controller,
                   const h2026_q2_config_t *config,
                   const h2026_q2_input_t *initial_input);

void h2026_q2_reset(h2026_q2_controller_t *controller,
                    const h2026_q2_input_t *initial_input);

void h2026_q2_step(h2026_q2_controller_t *controller,
                   const h2026_q2_input_t *input,
                   h2026_q2_output_t *output);

const char *h2026_q2_state_name(h2026_q2_state_t state);
const char *h2026_q2_fault_name(h2026_q2_fault_t fault);

#ifdef __cplusplus
}
#endif

#endif
