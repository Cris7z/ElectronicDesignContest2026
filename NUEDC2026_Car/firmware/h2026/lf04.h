/** LF04 four independent digital infrared channel processing. */
#ifndef H2026_LF04_H
#define H2026_LF04_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float error;
    float error_filtered;
    uint8_t raw_bits;
    uint8_t dark_bits;
    uint8_t dark_count;
    bool line_valid;
    bool all_dark;
    bool active_low;
} lf04_t;

void lf04_init(lf04_t *sensor, bool active_low);

/**
 * raw_bits bit0..bit3 correspond to O1..O4 from left to right.
 * error is normalised to approximately [-1, 1].
 */
void lf04_update(lf04_t *sensor, uint8_t raw_bits);

#endif
