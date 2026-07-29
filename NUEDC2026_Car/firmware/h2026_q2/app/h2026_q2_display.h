#ifndef H2026_Q2_DISPLAY_H
#define H2026_Q2_DISPLAY_H

#include "../core/h2026_q2.h"

#include <stdbool.h>
#include <stdint.h>

void h2026_q2_display_init(void);

/*
 * Rebuild the 128x64 framebuffer. This does not touch GPIO and is safe to
 * call from the foreground immediately after one control iteration.
 */
void h2026_q2_display_render(const h2026_q2_output_t *output,
                             uint8_t line_raw,
                             bool line_valid,
                             uint32_t calibration_locks,
                             uint32_t app_fault_code);

/*
 * Push exactly one 128-byte page. Returns true after page 7 is sent and the
 * complete frame is therefore visible. The caller uses this to clear the
 * 100 ms display flag only after the full frame is complete.
 */
bool h2026_q2_display_flush_one_page(void);

/* Separate screen for the guarded, one-shot D157B motor bench test. */
void h2026_q2_display_render_motor_commission(uint8_t stage,
                                               int64_t left_delta,
                                               int64_t right_delta,
                                               uint32_t completed_tests);

#endif
