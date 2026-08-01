#ifndef LINE_TRACKER_DISPLAY_H
#define LINE_TRACKER_DISPLAY_H

#include "line_tracker.h"

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    bool active;
    uint8_t state;
    uint8_t live_bits;
    uint8_t ok_bits;
    uint8_t progress_percent;
    uint8_t error;
    bool frame_valid;
    uint16_t raw_adc[LINE_TRACKER_SENSOR_COUNT];
} line_tracker_calibration_view_t;

void line_tracker_display_init(void);
void line_tracker_display_service(line_tracker_state_t state,
                                  uint32_t elapsed_ms,
                                  float distance_m,
                                  uint8_t mode,
                                  const line_tracker_calibration_view_t *calibration);

#endif
