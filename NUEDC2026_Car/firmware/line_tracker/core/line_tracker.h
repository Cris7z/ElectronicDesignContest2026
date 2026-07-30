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
    uint8_t max_track_black_count;
    uint16_t lost_limit_ticks;
    uint16_t center_gap_limit_ticks;
    float center_gap_error_limit;
    float base_duty;
    float wide_line_duty;
    float search_inner_duty;
    float search_outer_duty;
    float start_ramp_step;
    float kp;
    float kd;
    float correction_limit;
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
    uint16_t lost_ticks;
    float error;
    float left_duty;
    float right_duty;
} line_tracker_output_t;

typedef struct {
    line_tracker_config_t config;
    line_tracker_output_t output;
    float previous_error;
    float last_seen_error;
    float ramped_base_duty;
    uint16_t center_gap_ticks;
    bool has_seen_track;
} line_tracker_t;

bool line_tracker_init(line_tracker_t *tracker,
                       const line_tracker_config_t *config);
void line_tracker_reset(line_tracker_t *tracker);
void line_tracker_force_fault(line_tracker_t *tracker);
void line_tracker_step(line_tracker_t *tracker,
                       const line_tracker_input_t *input);

#endif
