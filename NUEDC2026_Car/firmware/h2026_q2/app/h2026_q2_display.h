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
                             uint8_t line_bits,
                             bool line_valid,
                             uint32_t calibration_locks,
                             uint8_t calibration_state,
                             uint8_t calibration_failure,
                             const uint16_t raw_adc[H2026_Q2_LINE_SENSOR_COUNT],
                             uint32_t app_fault_code);

/*
 * Push exactly one 128-byte page. Returns true after page 7 is sent and the
 * complete frame is therefore visible. The caller uses this to clear the
 * 100 ms display flag only after the full frame is complete.
 */
bool h2026_q2_display_flush_one_page(void);

/* Separate screen for the guarded, one-shot TB6612 motor bench test. */
void h2026_q2_display_render_motor_commission(uint8_t stage,
                                               int64_t left_delta,
                                               int64_t left_test_right_delta,
                                               int64_t right_test_left_delta,
                                               int64_t right_delta,
                                               uint32_t left_invalid,
                                               uint32_t right_invalid,
                                               uint32_t left_events,
                                               uint32_t right_events,
                                               uint32_t completed_tests);

/* Ground distance-calibration page; its caller guarantees motors are safe. */
void h2026_q2_display_render_distance_calibration(uint8_t stage,
                                                   int64_t left_count,
                                                   int64_t right_count,
                                                   uint32_t left_invalid,
                                                   uint32_t right_invalid);

/* Read-only encoder bring-up page: it never enables either TB6612 channel. */
void h2026_q2_display_render_encoder_passive(
    int64_t left_count, int64_t right_count,
    uint32_t left_invalid, uint32_t right_invalid,
    uint8_t left_phase, uint8_t right_phase);

/* Read-only PA18/BLS page: it never enables either TB6612 channel. */
void h2026_q2_display_render_bls_passive(bool raw_level,
                                         uint32_t edge_count);

#endif
