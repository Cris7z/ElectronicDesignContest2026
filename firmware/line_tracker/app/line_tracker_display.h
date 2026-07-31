#ifndef LINE_TRACKER_DISPLAY_H
#define LINE_TRACKER_DISPLAY_H

#include "line_tracker.h"

#include <stdint.h>

void line_tracker_display_init(void);
void line_tracker_display_service(line_tracker_state_t state,
                                  uint32_t elapsed_ms);

#endif
