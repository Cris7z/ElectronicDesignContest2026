/**
 * YB-MVX05-V1.0 eight-channel digital line sensor processing.
 *
 * The sensor exposes three CD4051 address inputs (AD0..AD2) and one
 * multiplexed digital OUT signal. Bit 0 of an address is AD0; bit 2 is AD2.
 */
#ifndef H2026_GRAY8_MUX_H
#define H2026_GRAY8_MUX_H

#include <stdbool.h>
#include <stdint.h>

#define GRAY8_MUX_CHANNELS          8u
#define GRAY8_MUX_DEFAULT_SETTLE_US 100u

typedef struct {
    float error;
    float error_filtered;
    uint8_t raw_bits;
    uint8_t dark_bits;
    uint8_t dark_count;
    uint8_t scan_address;
    uint8_t scan_bits;
    bool line_valid;
    bool line_lost;
    bool all_dark;
    bool active_low;
} gray8_mux_t;

void gray8_mux_init(gray8_mux_t *sensor, bool active_low);

/**
 * Start a new X1..X8 scan.
 *
 * The caller should then repeatedly:
 *   1. obtain gray8_mux_current_address();
 *   2. drive AD0..AD2 from address bits 0..2;
 *   3. wait at least GRAY8_MUX_DEFAULT_SETTLE_US;
 *   4. read OUT and call gray8_mux_push_out_sample().
 */
void gray8_mux_begin_scan(gray8_mux_t *sensor);

/** Return the next CD4051 address in the range 0..7. */
uint8_t gray8_mux_current_address(const gray8_mux_t *sensor);

/**
 * Store OUT for the current address and advance to the next channel.
 *
 * Returns true after the eighth sample has been processed. A completed scan
 * automatically resets the scan state to address zero for the next scan.
 */
bool gray8_mux_push_out_sample(gray8_mux_t *sensor, bool out_high);

/**
 * Process a complete raw scan.
 *
 * raw_bits bit0..bit7 correspond to X1..X8 from left to right. The weighted
 * error is normalised to [-1, 1]. It is only updated for a usable line
 * pattern: neither all white/lost nor all dark.
 */
void gray8_mux_update(gray8_mux_t *sensor, uint8_t raw_bits);

#endif
