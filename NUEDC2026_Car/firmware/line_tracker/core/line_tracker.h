#ifndef LINE_TRACKER_H
#define LINE_TRACKER_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_TRACKER_SENSOR_COUNT 8U

typedef enum {
    LINE_TRACKER_WAIT = 0,
    LINE_TRACKER_RUN,
    LINE_TRACKER_FAULT
} line_tracker_state_t;

typedef struct {
    uint16_t white_adc[LINE_TRACKER_SENSOR_COUNT];
    uint16_t black_adc[LINE_TRACKER_SENSOR_COUNT];
    float x_mm[LINE_TRACKER_SENSOR_COUNT];
    uint16_t black_on_strength;
    uint16_t black_off_strength;
    uint16_t centroid_min_strength;
    uint8_t max_track_black_count;
    uint16_t lost_limit_ticks;
    uint8_t lost_confirm_ticks;
    uint8_t cross_confirm_ticks;
    uint16_t center_gap_limit_ticks;
    float center_gap_error_limit;
    float speed_start_duty;
    float speed_max_duty;
    float speed_min_duty;
    /* Separate low-pass for speed scheduling; it must not change yaw PD. */
    float speed_filter_alpha;
    /* Keep maximum speed up to this CarControl-equivalent error weight. */
    float speed_deadband_weight;
    /* Reach minimum speed at or beyond this error weight. */
    float speed_full_slow_weight;
    float speed_accel_step;
    float speed_decel_step;
    float search_inner_duty;
    float search_outer_duty;
    float pid_p_yaw;
    float pid_i_yaw;
    float pid_d_yaw;
    float pid_integral_limit;
    /* First-order low-pass for weighted line position, 0 < alpha <= 1. */
    float error_filter_alpha;
    /* First-order low-pass for measurement derivative, 0 < alpha <= 1. */
    float pid_d_filter_alpha;
    /* Absolute D contribution after it is converted to signed PWM duty. */
    float pid_d_limit_duty;
    float yaw_limit_duty;
    float edge_yaw_duty;
    float yaw_gain_min;
    float yaw_gain_start_weight;
    float yaw_gain_full_weight;
    float edge_blend_start_weight;
    float edge_blend_full_weight;
    float steering_polarity;
    float duty_limit;
} line_tracker_config_t;

typedef struct {
    bool line_valid;
    uint16_t raw_adc[LINE_TRACKER_SENSOR_COUNT];
    bool start_event;
    bool stop_event;
} line_tracker_input_t;

typedef struct {
    line_tracker_state_t state;
    uint16_t strength[LINE_TRACKER_SENSOR_COUNT];
    uint8_t black_mask;
    uint8_t black_count;
    bool center_gap_mode;
    bool cross_mode;
    uint16_t lost_ticks;
    float error;
    /* CCS/host diagnostics: all yaw duties are before steering polarity. */
    float pid_p_duty;
    float pid_d_duty;
    bool pid_d_limited;
    float yaw_pd_duty;
    float yaw_final_duty;
    float base_duty;
    float edge_blend;
    float left_duty;
    float right_duty;
} line_tracker_output_t;

typedef struct {
    line_tracker_config_t config;
    line_tracker_output_t output;
    float previous_error;
    float filtered_error;
    float filtered_derivative;
    float integral_error;
    float last_seen_error;
    float ramped_base_duty;
    float speed_weight_lpf;
    uint16_t center_gap_ticks;
    uint8_t lost_candidate_ticks;
    uint8_t cross_candidate_ticks;
    bool has_seen_track;
    bool error_filter_seeded;
    bool derivative_seeded;
    bool speed_filter_seeded;
} line_tracker_t;

bool line_tracker_init(line_tracker_t *tracker,
                       const line_tracker_config_t *config);
void line_tracker_reset(line_tracker_t *tracker);
void line_tracker_force_fault(line_tracker_t *tracker);
void line_tracker_step(line_tracker_t *tracker,
                       const line_tracker_input_t *input);

#endif
