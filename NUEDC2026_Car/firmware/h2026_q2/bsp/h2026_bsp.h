/**
 * H2026 Q2 hardware abstraction for the frozen C07A/MSPM0G3507 vehicle.
 *
 * This interface intentionally exposes electrical motor polarity and raw
 * encoder phase polarity.  It does not assume which electrical direction is
 * vehicle-forward, and it does not contain CPR, wheel diameter, or track
 * width.  Those measured parameters belong to the Q2 controller config.
 */
#ifndef H2026_Q2_BSP_H
#define H2026_Q2_BSP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define H2026_BSP_CONTROL_PERIOD_MS       5u
#define H2026_BSP_DISPLAY_PERIOD_MS     100u
#define H2026_BSP_LINE_I2C_ADDRESS_7BIT   0x5Du
#define H2026_BSP_LINE_STATE_REGISTER     5u

typedef struct {
    int64_t left_count;
    int64_t right_count;
    uint32_t left_invalid_transitions;
    uint32_t right_invalid_transitions;
} h2026_bsp_encoder_snapshot_t;

typedef struct {
    uint32_t i2c_transactions;
    uint32_t i2c_timeouts;
    uint32_t i2c_bus_errors;
    uint32_t line_uart_requests;
    uint32_t line_uart_responses;
    uint32_t line_uart_timeouts;
    uint32_t control_tick_overruns;
} h2026_bsp_diagnostics_t;

/**
 * Initialise generated peripherals, force both TB6612 channels to coast,
 * seed encoder phase state, then enable encoder/control-tick interrupts.
 * Motor drive remains disarmed until h2026_bsp_motor_arm(true).
 */
bool h2026_bsp_init(void);

/**
 * Consume one pending 5 ms control tick.  The timer ISR never runs control
 * code; it only records the tick and an overrun if the previous tick was not
 * consumed.  If non-NULL, overrun_count receives the cumulative count.
 */
bool h2026_bsp_take_control_tick(uint32_t *overrun_count);
uint32_t h2026_bsp_millis(void);

/**
 * The 5 ms control-timer ISR raises this flag every 100 ms.  OLED bytes are
 * never sent from the ISR: the foreground consumes one page per control tick
 * and clears the request after the whole eight-page frame has been sent.
 */
bool h2026_bsp_display_refresh_pending(void);
void h2026_bsp_display_refresh_complete(void);
uint32_t h2026_bsp_display_counter(void);

/**
 * Arm/disarm motor drive.  Disarming immediately commands electrical coast
 * (IN1=0, IN2=0) on both motors.
 */
void h2026_bsp_motor_arm(bool armed);
bool h2026_bsp_motor_is_armed(void);

/**
 * Apply signed electrical duty in [-1, +1].
 *   duty > 0: IN1=PWM, IN2=0
 *   duty < 0: IN1=0,   IN2=PWM
 *   duty = 0: IN1=0,   IN2=0 (coast)
 *
 * This sign is not defined as vehicle-forward.  The controller must apply
 * independently measured left/right motor signs.
 *
 * Call this once per consumed 5 ms control tick.  Each wheel has an
 * independent reversal guard: changing directly between non-zero signs first
 * forces that wheel to 00 for one complete call interval.  The reverse PWM is
 * applied no earlier than a later control tick, and only if the same new sign
 * is requested again.  Disarm, explicit coast, and brake reset both guards.
 */
void h2026_bsp_motor_set_signed(float left_duty, float right_duty);
void h2026_bsp_motor_coast(void);
void h2026_bsp_motor_brake(void);

/** Snapshot raw four-edge quadrature counts and illegal two-bit jumps. */
void h2026_bsp_encoder_snapshot(h2026_bsp_encoder_snapshot_t *snapshot);

/**
 * Foreground-only bounded software-I2C transaction on U3 PB16/PB17:
 * write register selector 5, then read one state byte from address 0x5D.
 * It never runs from an ISR and returns false on timeout or bus error.
 */
bool h2026_bsp_line_read_reg5(uint8_t *state);

/**
 * Foreground-only native HiWonder UART state read on UART1 PB6/PB7, 115200
 * 8N1.  Initialisation places the module in manual mode (command 0); each
 * call requests command 1 when the previous response is complete.  A fresh
 * one-byte S1..S8 state response returns true.  The last response remains
 * valid for 30 ms so a single delayed byte does not make the motion loop
 * fault spuriously.
 */
bool h2026_bsp_line_uart_read_state(uint8_t *state);

/** Raw PA18 input level; active polarity and debounce belong to the app. */
bool h2026_bsp_start_level(void);
void h2026_bsp_led_set(bool on);

/**
 * Foreground-only four-wire OLED transport.  Keep refreshes low-rate; these
 * calls intentionally never execute in the 5 ms timer ISR.
 */
void h2026_bsp_oled_reset(void);
void h2026_bsp_oled_write_command(uint8_t command);
void h2026_bsp_oled_write_data(const uint8_t *data, size_t length);

/** Blocking debug-UART write; never call it from an ISR. */
void h2026_bsp_uart0_write(const uint8_t *data, size_t length);

/**
 * Put only bytes that fit in the debug-UART TX FIFO and return immediately.
 * This is the only UART API permitted in the 5 ms control foreground path:
 * a detached or stalled debug receiver must never stop OLED/control updates.
 */
size_t h2026_bsp_uart0_try_write(const uint8_t *data, size_t length);

void h2026_bsp_diagnostics_snapshot(h2026_bsp_diagnostics_t *diagnostics);

#endif
