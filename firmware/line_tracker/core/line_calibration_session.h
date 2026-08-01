#ifndef LINE_CALIBRATION_SESSION_H
#define LINE_CALIBRATION_SESSION_H

#include "line_calibration.h"

#include <stdbool.h>
#include <stdint.h>

#define LINE_CALIBRATION_WHITE_TICKS 200U
#define LINE_CALIBRATION_BLACK_TICKS 1000U

typedef enum {
    LINE_CAL_IDLE = 0,
    LINE_CAL_WHITE,
    LINE_CAL_BLACK,
    LINE_CAL_READY_TO_SAVE,
    LINE_CAL_SAVED,
    LINE_CAL_FAILED
} line_calibration_state_t;

typedef enum {
    LINE_CAL_ERROR_NONE = 0,
    LINE_CAL_ERROR_ADC = 1,
    LINE_CAL_ERROR_SPAN = 2,
    LINE_CAL_ERROR_STORE_BASE = 16
} line_calibration_error_t;

typedef struct {
    line_calibration_state_t state;
    uint16_t ticks;
    uint32_t white_sum[LINE_TRACKER_SENSOR_COUNT];
    uint16_t white_adc[LINE_TRACKER_SENSOR_COUNT];
    uint16_t black_adc[LINE_TRACKER_SENSOR_COUNT];
    uint32_t black_delta[LINE_TRACKER_SENSOR_COUNT];
    uint8_t ok_bits;
    uint8_t error;
} line_calibration_session_t;

void line_calibration_session_begin(line_calibration_session_t *session);
void line_calibration_session_cancel(line_calibration_session_t *session);
void line_calibration_session_step(
    line_calibration_session_t *session,
    const uint16_t raw_adc[LINE_TRACKER_SENSOR_COUNT], bool frame_valid);
void line_calibration_session_mark_saved(line_calibration_session_t *session);
void line_calibration_session_mark_store_failed(
    line_calibration_session_t *session, uint8_t store_error);
bool line_calibration_session_active(
    const line_calibration_session_t *session);
uint8_t line_calibration_session_live_bits(
    const line_calibration_session_t *session,
    const uint16_t raw_adc[LINE_TRACKER_SENSOR_COUNT]);
uint8_t line_calibration_session_progress_percent(
    const line_calibration_session_t *session);

#endif
