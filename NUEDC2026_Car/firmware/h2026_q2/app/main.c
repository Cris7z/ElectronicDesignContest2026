#include "h2026_q2_app_config.h"
#include "h2026_q2_display.h"

#include "../bsp/h2026_bsp.h"
#include "../core/h2026_q2.h"

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define BUTTON_DEBOUNCE_TICKS       6U
#define BUTTON_ESTOP_HOLD_TICKS   200U
#define MARKER_READY_TICKS          10U
#define ENCODER_INVALID_WINDOW_TICKS 20U
#define ENCODER_INVALID_WINDOW_MAX    8U
#define FAULT_BRAKE_TICKS           20U
#define TELEMETRY_PERIOD_TICKS      20U
#define TELEMETRY_BYTES_PER_TICK     4U
#define ENCODER_SPEED_PLAUSIBILITY_FACTOR 2.0f
#define STALL_TARGET_MIN_MPS         0.12f
#define STALL_DUTY_MIN               0.25f
#define STALL_MEASURED_MAX_MPS       0.02f
#define STALL_CONFIRM_TICKS          60U

typedef enum {
    APP_FAULT_NONE = 0,
    APP_FAULT_BSP_INIT = 1,
    APP_FAULT_CORE_CONFIG = 2,
    APP_FAULT_TICK_OVERRUN = 3,
    APP_FAULT_ENCODER_SIGNAL = 4,
    APP_FAULT_ENCODER_STALL = 5,
    APP_FAULT_ENCODER_IMPLAUSIBLE = 6
} app_fault_t;

typedef struct {
    bool candidate_pressed;
    bool stable_pressed;
    uint16_t candidate_ticks;
    uint16_t held_ticks;
    bool estop_sent;
} button_filter_t;

typedef struct {
    char bytes[160];
    size_t length;
    size_t sent;
} telemetry_queue_t;

static h2026_q2_controller_t s_controller;
static h2026_q2_config_t s_config;
static h2026_q2_output_t s_output;
static h2026_q2_input_t s_input;
static button_filter_t s_button;
static telemetry_queue_t s_telemetry;

static bool s_controller_ready;
static app_fault_t s_app_fault;
static uint32_t s_calibration_locks;
static uint8_t s_line_raw;
static uint16_t s_marker_ready_ticks;
static uint32_t s_last_overrun_count;
static uint32_t s_left_invalid_previous;
static uint32_t s_right_invalid_previous;
static int64_t s_left_count_previous;
static int64_t s_right_count_previous;
static uint32_t s_encoder_invalid_window;
static uint16_t s_encoder_invalid_window_ticks;
static uint16_t s_left_stall_ticks;
static uint16_t s_right_stall_ticks;
static uint16_t s_fault_brake_ticks;
static uint32_t s_foreground_ticks;
static bool s_display_job_active;

static bool state_is_moving(h2026_q2_state_t state)
{
    return (state == H2026_Q2_STATE_CLEAR_START) ||
           (state == H2026_Q2_STATE_LAP) ||
           (state == H2026_Q2_STATE_FINISH_ARMED) ||
           (state == H2026_Q2_STATE_STOPPING);
}

static float line_pattern_center(uint8_t normalized_bits,
                                 uint8_t active_count)
{
    int numerator = 0;
    uint8_t bit;

    if (active_count == 0U) {
        return 0.0f;
    }
    for (bit = 0U; bit < 8U; ++bit) {
        if ((normalized_bits & (uint8_t)(1U << bit)) != 0U) {
            numerator += ((int)bit * 2) - 7;
        }
    }
    return (float)numerator / ((float)active_count * 7.0f);
}

static float absolute_float(float value)
{
    return (value < 0.0f) ? -value : value;
}

static bool button_update(bool raw_level,
                          bool *press_event,
                          bool *estop_event)
{
    const bool pressed = h2026_q2_app_start_pressed(raw_level);

    *press_event = false;
    *estop_event = false;
    if (pressed != s_button.candidate_pressed) {
        s_button.candidate_pressed = pressed;
        s_button.candidate_ticks = 1U;
    } else if (s_button.candidate_ticks < BUTTON_DEBOUNCE_TICKS) {
        ++s_button.candidate_ticks;
    }

    if ((s_button.candidate_ticks >= BUTTON_DEBOUNCE_TICKS) &&
        (s_button.stable_pressed != s_button.candidate_pressed)) {
        s_button.stable_pressed = s_button.candidate_pressed;
        if (s_button.stable_pressed) {
            s_button.held_ticks = 0U;
            s_button.estop_sent = false;
            *press_event = true;
        } else {
            s_button.held_ticks = 0U;
            s_button.estop_sent = false;
        }
    }

    if (s_button.stable_pressed) {
        if (s_button.held_ticks < UINT16_MAX) {
            ++s_button.held_ticks;
        }
        if (!s_button.estop_sent &&
            (s_button.held_ticks >= BUTTON_ESTOP_HOLD_TICKS)) {
            s_button.estop_sent = true;
            *estop_event = true;
        }
    }
    return s_button.stable_pressed;
}

static void update_marker_ready(void)
{
    h2026_q2_line_observation_t observation;

    if (!s_controller_ready || !s_input.line_i2c_valid) {
        s_marker_ready_ticks = 0U;
        return;
    }
    h2026_q2_line_decode(s_input.line_raw_reg5,
                         s_config.sensor_active_high,
                         s_config.sensor_bit0_is_left,
                         s_config.wide_min_active,
                         &observation);
    if ((observation.active_count >=
         s_config.marker_capture_min_active) &&
        (observation.active_count < 8U) &&
        (observation.classification == H2026_Q2_LINE_WIDE) &&
        (observation.block_count == 1U) &&
        (absolute_float(line_pattern_center(
             observation.normalized_bits,
             observation.active_count)) <=
         s_config.marker_center_limit_normalized)) {
        if (s_marker_ready_ticks < MARKER_READY_TICKS) {
            ++s_marker_ready_ticks;
        }
    } else {
        s_marker_ready_ticks = 0U;
    }
}

static bool app_fault_is_recoverable(app_fault_t fault)
{
    return (fault == APP_FAULT_TICK_OVERRUN) ||
           (fault == APP_FAULT_ENCODER_SIGNAL) ||
           (fault == APP_FAULT_ENCODER_STALL) ||
           (fault == APP_FAULT_ENCODER_IMPLAUSIBLE);
}

static void reset_for_retry(void)
{
    h2026_bsp_encoder_snapshot_t encoders;

    if (!s_controller_ready) {
        return;
    }
    h2026_bsp_motor_arm(false);
    h2026_bsp_encoder_snapshot(&encoders);
    s_input.encoder_left_count = encoders.left_count;
    s_input.encoder_right_count = encoders.right_count;
    s_input.start_event = false;
    s_input.estop_event = false;
    h2026_q2_reset(&s_controller, &s_input);
    h2026_q2_step(&s_controller, &s_input, &s_output);
    s_left_invalid_previous = encoders.left_invalid_transitions;
    s_right_invalid_previous = encoders.right_invalid_transitions;
    s_left_count_previous = encoders.left_count;
    s_right_count_previous = encoders.right_count;
    s_encoder_invalid_window = 0U;
    s_encoder_invalid_window_ticks = 0U;
    s_left_stall_ticks = 0U;
    s_right_stall_ticks = 0U;
    s_fault_brake_ticks = 0U;
    if (app_fault_is_recoverable(s_app_fault)) {
        s_app_fault = APP_FAULT_NONE;
    }
}

static void update_encoder_safety(
    const h2026_bsp_encoder_snapshot_t *encoders)
{
    const uint32_t left_delta =
        encoders->left_invalid_transitions - s_left_invalid_previous;
    const uint32_t right_delta =
        encoders->right_invalid_transitions - s_right_invalid_previous;
    const uint32_t new_invalid = left_delta + right_delta;
    const int64_t left_count_delta =
        encoders->left_count - s_left_count_previous;
    const int64_t right_count_delta =
        encoders->right_count - s_right_count_previous;
    const float left_delta_m =
        absolute_float((float)left_count_delta) *
        s_config.left_meters_per_encoder_count;
    const float right_delta_m =
        absolute_float((float)right_count_delta) *
        s_config.right_meters_per_encoder_count;
    const float plausible_delta_m =
        s_config.maximum_wheel_speed_mps *
        H2026_Q2_TICK_S *
        ENCODER_SPEED_PLAUSIBILITY_FACTOR;

    s_left_invalid_previous = encoders->left_invalid_transitions;
    s_right_invalid_previous = encoders->right_invalid_transitions;
    s_left_count_previous = encoders->left_count;
    s_right_count_previous = encoders->right_count;

    if (!state_is_moving(s_output.state)) {
        s_encoder_invalid_window = 0U;
        s_encoder_invalid_window_ticks = 0U;
        return;
    }
    if ((left_delta_m > plausible_delta_m) ||
        (right_delta_m > plausible_delta_m)) {
        s_app_fault = APP_FAULT_ENCODER_IMPLAUSIBLE;
    }
    if (new_invalid > UINT32_MAX - s_encoder_invalid_window) {
        s_encoder_invalid_window = UINT32_MAX;
    } else {
        s_encoder_invalid_window += new_invalid;
    }
    if (s_encoder_invalid_window_ticks < UINT16_MAX) {
        ++s_encoder_invalid_window_ticks;
    }
    if (s_encoder_invalid_window_ticks >=
        ENCODER_INVALID_WINDOW_TICKS) {
        if (s_encoder_invalid_window >
            ENCODER_INVALID_WINDOW_MAX) {
            s_app_fault = APP_FAULT_ENCODER_SIGNAL;
        }
        s_encoder_invalid_window = 0U;
        s_encoder_invalid_window_ticks = 0U;
    }
}

static void update_stall_safety(void)
{
    const bool left_expected =
        absolute_float(s_output.left_target_speed_mps) >=
            STALL_TARGET_MIN_MPS &&
        absolute_float(s_output.left_signed_duty) >= STALL_DUTY_MIN;
    const bool right_expected =
        absolute_float(s_output.right_target_speed_mps) >=
            STALL_TARGET_MIN_MPS &&
        absolute_float(s_output.right_signed_duty) >= STALL_DUTY_MIN;

    if (!state_is_moving(s_output.state)) {
        s_left_stall_ticks = 0U;
        s_right_stall_ticks = 0U;
        return;
    }

    if (left_expected &&
        (absolute_float(s_output.left_measured_speed_mps) <=
         STALL_MEASURED_MAX_MPS)) {
        if (s_left_stall_ticks < UINT16_MAX) {
            ++s_left_stall_ticks;
        }
    } else {
        s_left_stall_ticks = 0U;
    }
    if (right_expected &&
        (absolute_float(s_output.right_measured_speed_mps) <=
         STALL_MEASURED_MAX_MPS)) {
        if (s_right_stall_ticks < UINT16_MAX) {
            ++s_right_stall_ticks;
        }
    } else {
        s_right_stall_ticks = 0U;
    }
    if ((s_left_stall_ticks >= STALL_CONFIRM_TICKS) ||
        (s_right_stall_ticks >= STALL_CONFIRM_TICKS)) {
        s_app_fault = APP_FAULT_ENCODER_STALL;
    }
}

static void apply_motor_output(void)
{
    if (!s_controller_ready || (s_calibration_locks != 0U)) {
        h2026_bsp_motor_arm(false);
        return;
    }

    /*
     * A missed 5 ms deadline or invalid encoder signal makes closed-loop
     * braking untrustworthy, so an app-level fault goes straight to 00 coast.
     * It is deliberately not disguised as a button ESTOP in the core.
     */
    if (s_app_fault != APP_FAULT_NONE) {
        h2026_bsp_motor_arm(false);
        s_fault_brake_ticks = 0U;
        return;
    }

    if (s_output.state == H2026_Q2_STATE_FAULT) {
        if (s_output.brake) {
            if (s_fault_brake_ticks < FAULT_BRAKE_TICKS) {
                h2026_bsp_motor_arm(true);
                h2026_bsp_motor_brake();
                ++s_fault_brake_ticks;
            } else {
                h2026_bsp_motor_arm(false);
            }
        } else {
            /* Non-ESTOP core faults coast while wheel speed is still high. */
            h2026_bsp_motor_arm(false);
            s_fault_brake_ticks = 0U;
        }
        return;
    }

    s_fault_brake_ticks = 0U;
    if (state_is_moving(s_output.state)) {
        h2026_bsp_motor_arm(true);
        h2026_bsp_motor_set_signed(
            h2026_q2_app_left_motor_duty(s_output.left_signed_duty),
            h2026_q2_app_right_motor_duty(s_output.right_signed_duty));
    } else if (s_output.state == H2026_Q2_STATE_HOLD) {
        h2026_bsp_motor_arm(true);
        h2026_bsp_motor_brake();
    } else {
        h2026_bsp_motor_arm(false);
    }
}

static void telemetry_append_character(char character)
{
    if (s_telemetry.length < sizeof(s_telemetry.bytes)) {
        s_telemetry.bytes[s_telemetry.length] = character;
        ++s_telemetry.length;
    }
}

static void telemetry_append_text(const char *text)
{
    while ((text != NULL) && (*text != '\0')) {
        telemetry_append_character(*text);
        ++text;
    }
}

static void telemetry_append_u32(uint32_t value)
{
    char reverse[10];
    size_t count = 0U;

    do {
        reverse[count] = (char)('0' + (value % 10U));
        value /= 10U;
        ++count;
    } while ((value != 0U) && (count < sizeof(reverse)));
    while (count > 0U) {
        --count;
        telemetry_append_character(reverse[count]);
    }
}

static void telemetry_append_i32(int32_t value)
{
    uint32_t magnitude;
    if (value < 0) {
        telemetry_append_character('-');
        magnitude = (uint32_t)(-(value + 1)) + 1U;
    } else {
        magnitude = (uint32_t)value;
    }
    telemetry_append_u32(magnitude);
}

static void telemetry_append_hex8(uint8_t value)
{
    const char hex[] = "0123456789ABCDEF";
    telemetry_append_character(hex[(value >> 4U) & 0x0FU]);
    telemetry_append_character(hex[value & 0x0FU]);
}

static int32_t scaled_i32(float value, float scale)
{
    const float scaled = value * scale;
    if (scaled >= 0.0f) {
        return (int32_t)(scaled + 0.5f);
    }
    return (int32_t)(scaled - 0.5f);
}

static void telemetry_prepare(void)
{
    h2026_bsp_diagnostics_t diagnostics;

    if (s_telemetry.sent < s_telemetry.length) {
        return;
    }
    h2026_bsp_diagnostics_snapshot(&diagnostics);
    s_telemetry.length = 0U;
    s_telemetry.sent = 0U;

    telemetry_append_text("COUNT=");
    telemetry_append_u32(h2026_bsp_display_counter());
    telemetry_append_text(",Q2,s=");
    telemetry_append_u32((uint32_t)s_output.state);
    telemetry_append_text(",f=");
    telemetry_append_u32((uint32_t)s_output.fault);
    telemetry_append_text(",a=");
    telemetry_append_u32((uint32_t)s_app_fault);
    telemetry_append_text(",t=");
    telemetry_append_u32(s_output.elapsed_ms);
    telemetry_append_text(",dmm=");
    telemetry_append_i32(scaled_i32(s_output.distance_m, 1000.0f));
    telemetry_append_text(",raw=");
    telemetry_append_hex8(s_line_raw);
    telemetry_append_text(",uart_ok=");
    telemetry_append_u32(s_input.line_i2c_valid ? 1U : 0U);
    telemetry_append_text(",ur=");
    telemetry_append_u32(diagnostics.line_uart_requests);
    telemetry_append_text("/");
    telemetry_append_u32(diagnostics.line_uart_responses);
    telemetry_append_text("/");
    telemetry_append_u32(diagnostics.line_uart_timeouts);
    telemetry_append_text(",e=");
    telemetry_append_i32(
        scaled_i32(s_output.diagnostics.filtered_line_error, 1000.0f));
    telemetry_append_text(",v=");
    telemetry_append_i32(
        scaled_i32(s_output.center_speed_command_mps, 1000.0f));
    telemetry_append_text(",l=");
    telemetry_append_i32(
        scaled_i32(s_output.left_measured_speed_mps, 1000.0f));
    telemetry_append_text(",r=");
    telemetry_append_i32(
        scaled_i32(s_output.right_measured_speed_mps, 1000.0f));
    telemetry_append_text("\r\n");
}

static void telemetry_service(void)
{
    size_t remaining;
    size_t chunk;
    size_t sent;

    if (s_telemetry.sent >= s_telemetry.length) {
        return;
    }
    remaining = s_telemetry.length - s_telemetry.sent;
    chunk = (remaining < TELEMETRY_BYTES_PER_TICK)
                ? remaining
                : TELEMETRY_BYTES_PER_TICK;
    sent = h2026_bsp_uart0_try_write(
        (const uint8_t *)&s_telemetry.bytes[s_telemetry.sent], chunk);
    s_telemetry.sent += sent;
}

static void update_led(void)
{
    const uint32_t milliseconds = h2026_bsp_millis();
    bool on;

    if ((s_app_fault != APP_FAULT_NONE) ||
        (s_output.state == H2026_Q2_STATE_FAULT)) {
        on = ((milliseconds / 100U) & 1U) != 0U;
    } else if (state_is_moving(s_output.state)) {
        on = true;
    } else if (s_output.state == H2026_Q2_STATE_HOLD) {
        on = ((milliseconds / 250U) & 1U) != 0U;
    } else {
        on = ((milliseconds / 500U) & 1U) != 0U;
    }
    h2026_bsp_led_set(on);
}

/*
 * TIMG7 raises the display flag every 100 ms.  Keep the actual OLED transport
 * in the foreground and emit only one 128-byte page per 5 ms control turn.
 * Thus a full frame needs 40 ms, and only then is the timer's flag cleared.
 */
static void display_service(void)
{
    if (!h2026_bsp_display_refresh_pending()) {
        return;
    }

    if (!s_display_job_active) {
        h2026_q2_display_render(&s_output,
                                s_line_raw,
                                s_input.line_i2c_valid,
                                s_calibration_locks,
                                (uint32_t)s_app_fault);
        s_display_job_active = true;
    }

    if (h2026_q2_display_flush_one_page()) {
        s_display_job_active = false;
        h2026_bsp_display_refresh_complete();
    }
}

static void run_control_tick(uint32_t overrun_count)
{
    h2026_bsp_encoder_snapshot_t encoders;
    bool press_event;
    bool estop_event;

    ++s_foreground_ticks;
    s_input.line_i2c_valid = h2026_bsp_line_uart_read_state(&s_line_raw);
    if (s_input.line_i2c_valid) {
        s_input.line_raw_reg5 = s_line_raw;
    }
    h2026_bsp_encoder_snapshot(&encoders);
    s_input.encoder_left_count = encoders.left_count;
    s_input.encoder_right_count = encoders.right_count;

    (void)button_update(h2026_bsp_start_level(),
                        &press_event,
                        &estop_event);
    update_marker_ready();

    if (overrun_count != s_last_overrun_count) {
        s_app_fault = APP_FAULT_TICK_OVERRUN;
        s_last_overrun_count = overrun_count;
    }
    update_encoder_safety(&encoders);

    s_input.start_event = false;
    s_input.estop_event = estop_event;
    if (press_event && s_controller_ready) {
        if ((s_output.state == H2026_Q2_STATE_HOLD) ||
            (s_output.state == H2026_Q2_STATE_FAULT) ||
            app_fault_is_recoverable(s_app_fault)) {
            reset_for_retry();
        } else if ((s_output.state == H2026_Q2_STATE_IDLE) &&
                   (s_marker_ready_ticks >= MARKER_READY_TICKS) &&
                   (s_app_fault == APP_FAULT_NONE)) {
            s_input.start_event = true;
        }
    }
    if (s_controller_ready) {
        h2026_q2_step(&s_controller, &s_input, &s_output);
        update_stall_safety();
    }
    apply_motor_output();
    update_led();

    if ((s_foreground_ticks % TELEMETRY_PERIOD_TICKS) == 0U) {
        telemetry_prepare();
    }
    telemetry_service();
}

int main(void)
{
    h2026_bsp_encoder_snapshot_t encoders;
    uint32_t overrun_count = 0U;
    bool bsp_ok;

    memset(&s_output, 0, sizeof(s_output));
    memset(&s_input, 0, sizeof(s_input));
    s_output.state = H2026_Q2_STATE_IDLE;

    bsp_ok = h2026_bsp_init();
    h2026_bsp_motor_arm(false);
    h2026_q2_display_init();

    /*
     * OLED reset/initial clear occurs before real-time scheduling begins.
     * Consume any boot-time pending tick and use its overrun count as the
     * baseline; later changes are genuine foreground deadline failures.
     */
    (void)h2026_bsp_take_control_tick(&overrun_count);
    s_last_overrun_count = overrun_count;

    s_line_raw = 0U;
    s_input.line_i2c_valid = h2026_bsp_line_uart_read_state(&s_line_raw);
    s_input.line_raw_reg5 = s_line_raw;
    h2026_bsp_encoder_snapshot(&encoders);
    s_input.encoder_left_count = encoders.left_count;
    s_input.encoder_right_count = encoders.right_count;
    s_left_invalid_previous = encoders.left_invalid_transitions;
    s_right_invalid_previous = encoders.right_invalid_transitions;
    s_left_count_previous = encoders.left_count;
    s_right_count_previous = encoders.right_count;

    s_calibration_locks = h2026_q2_app_calibration_locks();
    s_controller_ready =
        bsp_ok && h2026_q2_app_build_config(&s_config);
    if (!bsp_ok) {
        s_app_fault = APP_FAULT_BSP_INIT;
    } else if ((s_calibration_locks == 0U) &&
               !h2026_q2_config_validate(&s_config)) {
        s_app_fault = APP_FAULT_CORE_CONFIG;
    }
    if (s_controller_ready &&
        !h2026_q2_init(&s_controller, &s_config, &s_input)) {
        s_controller_ready = false;
        s_app_fault = APP_FAULT_CORE_CONFIG;
    }

    h2026_q2_display_render(&s_output,
                            s_line_raw,
                            s_input.line_i2c_valid,
                            s_calibration_locks,
                            (uint32_t)s_app_fault);

    for (;;) {
        if (h2026_bsp_take_control_tick(&overrun_count)) {
            run_control_tick(overrun_count);
            display_service();
        } else {
            __WFE();
        }
    }
}
