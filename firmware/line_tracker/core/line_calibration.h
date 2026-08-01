#ifndef LINE_CALIBRATION_H
#define LINE_CALIBRATION_H

#include "line_tracker.h"

#include <stdbool.h>
#include <stdint.h>

#define LINE_CALIBRATION_VERSION 1U
#define LINE_CALIBRATION_MIN_SPAN 410U

typedef struct {
    uint16_t white_adc[LINE_TRACKER_SENSOR_COUNT];
    uint16_t black_adc[LINE_TRACKER_SENSOR_COUNT];
    float sensor_x_mm[LINE_TRACKER_SENSOR_COUNT];
    uint16_t version;
    uint16_t crc16;
} line_calibration_record_t;

uint16_t line_calibration_crc16(const line_calibration_record_t *record);
void line_calibration_finalize(line_calibration_record_t *record);
bool line_calibration_valid(const line_calibration_record_t *record);
uint8_t line_calibration_span_ok_bits(
    const uint16_t white_adc[LINE_TRACKER_SENSOR_COUNT],
    const uint16_t black_adc[LINE_TRACKER_SENSOR_COUNT]);

#endif
