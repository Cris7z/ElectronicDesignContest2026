/**
 * HiWonder LineFollower_8CH v1.0 I2C driver and line-state processing.
 *
 * The transport callback must write the register-select byte and then read
 * the requested data using the 7-bit address supplied by this driver.
 */
#ifndef H2026_LINEFOLLOWER_8CH_H
#define H2026_LINEFOLLOWER_8CH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LINEFOLLOWER_8CH_CHANNELS          8u
#define LINEFOLLOWER_8CH_I2C_ADDR_7BIT     0x5Du
#define LINEFOLLOWER_8CH_REG_STATE         5u
#define LINEFOLLOWER_8CH_REG_ANALOG_CH1    6u
#define LINEFOLLOWER_8CH_REG_THRESHOLD_CH1 22u

typedef bool (*linefollower_8ch_i2c_read_fn)(
    void *context,
    uint8_t address_7bit,
    uint8_t register_address,
    uint8_t *data,
    size_t length);

typedef struct {
    float error;
    float error_filtered;
    uint16_t analog[LINEFOLLOWER_8CH_CHANNELS];
    uint16_t threshold[LINEFOLLOWER_8CH_CHANNELS];
    uint32_t i2c_errors;
    uint8_t state_bits;
    uint8_t line_bits;
    uint8_t line_count;
    bool line_valid;
    bool line_lost;
    bool all_line;
    bool state_active_high;
    bool state_valid;
    bool analog_valid;
    bool threshold_valid;
} linefollower_8ch_t;

/**
 * Initialise the processing state.
 *
 * HiWonder documents I2C/UART state bits as the inverse of the direct GPIO
 * outputs. For a learned target line, state_active_high should normally be
 * true; confirm this once on the real course before enabling the motors.
 */
void linefollower_8ch_init(linefollower_8ch_t *sensor,
                           bool state_active_high);

/** Process the one-byte value read from register 5. */
void linefollower_8ch_update_state(linefollower_8ch_t *sensor,
                                   uint8_t state_bits);

/**
 * Read register 5 and update the weighted line result.
 *
 * A failed state read clears state_valid and line_valid immediately. Slow
 * analog/threshold diagnostic reads never restore state_valid.
 */
bool linefollower_8ch_read_state(linefollower_8ch_t *sensor,
                                 linefollower_8ch_i2c_read_fn read,
                                 void *context);

/** Read registers 6..21 as eight little-endian 16-bit analog samples. */
bool linefollower_8ch_read_analog(linefollower_8ch_t *sensor,
                                  linefollower_8ch_i2c_read_fn read,
                                  void *context);

/** Read registers 22..37 as eight little-endian 16-bit learned thresholds. */
bool linefollower_8ch_read_thresholds(linefollower_8ch_t *sensor,
                                      linefollower_8ch_i2c_read_fn read,
                                      void *context);

#endif
