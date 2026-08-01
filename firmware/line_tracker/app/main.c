/*
 * C07A line_tracker: the deliberately small first-stage vehicle program.
 *
 * Scope: two external task buttons -> eight-channel CD4051 scan -> weighted
 * PD -> TB6612, plus the encoder distance monitor and SSD1306 status display.
 * Not included: MPU6050, K230 or beam control.
 */
#include "lap_monitor.h"
#include "line_calibration.h"
#include "line_calibration_session.h"
#include "line_calibration_store.h"
#include "line_tracker.h"
#include "line_tracker_display.h"
#include "task_mode.h"
#include "wheel_speed_pi.h"

#include "h2026_bsp.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define BUTTON_DEBOUNCE_TICKS 6U
#define BUTTON_ESTOP_TICKS 200U
/* Raised-chassis encoder sweep, 2026-07-31: first useful duty L=6 %, R=8 %. */
#define LEFT_FORWARD_DUTY_FLOOR 0.060f
#define RIGHT_FORWARD_DUTY_FLOOR 0.080f
#define WHEEL_SHADOW_COUNTS_PER_COMMAND_WINDOW 163.256f
#define WHEEL_SHADOW_MEASUREMENT_FILTER_ALPHA 0.50f
#define WHEEL_SHADOW_KP 0.30f
#define WHEEL_SHADOW_KI_PER_SECOND 0.0f
#define WHEEL_SHADOW_INTEGRAL_LIMIT 0.040f
#define WHEEL_SHADOW_CORRECTION_LIMIT 0.030f
#define WHEEL_SHADOW_ENABLE_THRESHOLD 0.100f
/* A/B: retain speed-P measurement/diagnostics, but do not inject it into PWM. */
#define WHEEL_SHADOW_APPLY_CORRECTION false

/* H-R02: first ground run stopped at the final semicircle apex with the
 * geometric 6.1416 m value; use the resulting 7.040 m encoder target. */
#define H_R02_LAP_DISTANCE_M 7.0400f
#define MISSION_APPROACH_DISTANCE_M 0.1500f
#define MISSION_APPROACH_DUTY_LIMIT 0.1200f
#define MISSION_TIMEOUT_MS 20000U
#define LEFT_METERS_PER_COUNT 0.0001561454f
#define RIGHT_METERS_PER_COUNT 0.0001659948f
#define LEFT_FORWARD_SIGN (-1)
#define RIGHT_FORWARD_SIGN 1

/* All values below are measured on the installed 2026-07-30 vehicle. */
static const line_tracker_config_t k_frozen_config = {
    .white_adc = {174U, 174U, 172U, 173U, 171U, 172U, 171U, 171U},
    .black_adc = {4095U, 4095U, 4095U, 4095U,
                  4095U, 4095U, 4095U, 4095U},
    .x_mm = {-35.0f, -25.0f, -15.0f, -5.0f,
              5.0f, 15.0f, 25.0f, 35.0f},
    .black_on_strength = 600U,
    .black_off_strength = 400U,
    .centroid_min_strength = 150U,
    .max_track_black_count = 3U,
    .lost_limit_ticks = 60U,       /* 300 ms at the fixed 5 ms tick */
    .lost_confirm_ticks = 3U,      /* Ignore up to 10 ms of valid optical loss. */
    .cross_confirm_ticks = 2U,     /* Require two valid wide-black frames. */
    .center_gap_limit_ticks = 0U,  /* Disabled until the real track proves a centre gap. */
    .center_gap_error_limit = 0.20f,
    /* Phase A: a filtered, deadbanded speed envelope keeps normal centre-line
     * motion at steady throttle while preserving low speed in real turns. */
    .speed_start_duty = 0.0800f,
    /* Reserve the final 3 % of PWM for each wheel's active speed PI. */
    .speed_max_duty = 0.4875f,
    .speed_min_duty = 0.2300f,      /* Previous 20 %, raised 15 %. */
    .speed_filter_alpha = 0.20f,
    .speed_deadband_weight = 1.0f,
    .speed_full_slow_weight = 4.0f,
    .speed_accel_step = 0.0050f,
    .speed_decel_step = 0.0080f,
    .search_inner_duty = 0.1150f,
    .search_outer_duty = 0.2300f,
    .pid_p_yaw = 5.0f,
    .pid_i_yaw = 0.0f,
    .pid_d_yaw = 125.0f,
    .pid_integral_limit = 5.0f,
    .error_filter_alpha = 0.60f,   /* main's line-position low-pass. */
    .pid_d_filter_alpha = 0.35f,   /* Same D low-pass structure as main. */
    .yaw_limit_duty = 0.450f,
    .yaw_slew_step = 0.080f,
    .edge_yaw_duty = 0.300f,        /* edge yaw: 60 % -> 30 % */
    .yaw_gain_min = 0.90f,
    .yaw_gain_start_weight = 1.0f,
    .yaw_gain_full_weight = 4.0f,
    .edge_blend_start_weight = 5.0f,
    .edge_blend_full_weight = 7.0f,
    /* Ground test: black line offset must produce a turn back toward the line. */
    .steering_polarity = 1.0f,
    .duty_limit = 0.5175f
};

/* Phase D: each wheel gets a separate, bounded P-only speed correction. */
static const wheel_speed_pi_config_t k_wheel_speed_shadow_config = {
    .counts_per_command_window = WHEEL_SHADOW_COUNTS_PER_COMMAND_WINDOW,
    .measurement_filter_alpha = WHEEL_SHADOW_MEASUREMENT_FILTER_ALPHA,
    .kp = WHEEL_SHADOW_KP,
    .ki_per_second = WHEEL_SHADOW_KI_PER_SECOND,
    .integral_limit = WHEEL_SHADOW_INTEGRAL_LIMIT,
    .correction_limit = WHEEL_SHADOW_CORRECTION_LIMIT,
    .enable_threshold = WHEEL_SHADOW_ENABLE_THRESHOLD
};

static const lap_monitor_config_t k_lap_monitor_template = {
    .target_distance_m = H_R02_LAP_DISTANCE_M,
    .approach_distance_m = MISSION_APPROACH_DISTANCE_M,
    .left_meters_per_count = LEFT_METERS_PER_COUNT,
    .right_meters_per_count = RIGHT_METERS_PER_COUNT,
    .left_forward_sign = LEFT_FORWARD_SIGN,
    .right_forward_sign = RIGHT_FORWARD_SIGN,
    .timeout_ms = MISSION_TIMEOUT_MS
};

typedef struct {
    bool stable_pressed;
    bool candidate_pressed;
    uint8_t candidate_ticks;
    uint16_t active_ticks;
    bool hold_sent;
} app_button_t;

/* Intentionally visible in CCS/XDS110 Expressions during track tuning. */
volatile line_tracker_output_t g_line_tracker_output;
volatile uint16_t g_line_tracker_raw_adc[LINE_TRACKER_SENSOR_COUNT];
volatile uint32_t g_line_tracker_tick_overruns;
volatile bool g_line_tracker_bsp_ready;
volatile float g_line_tracker_applied_left_duty;
volatile float g_line_tracker_applied_right_duty;
volatile float g_line_tracker_debug_p_duty;
volatile float g_line_tracker_debug_d_duty;
volatile float g_line_tracker_debug_pd_target_yaw;
volatile float g_line_tracker_debug_final_yaw;
volatile float g_line_tracker_debug_base_duty;
volatile float g_line_tracker_debug_edge_blend;
volatile float g_line_tracker_shadow_left_target;
volatile float g_line_tracker_shadow_right_target;
volatile float g_line_tracker_shadow_left_measured;
volatile float g_line_tracker_shadow_right_measured;
volatile float g_line_tracker_shadow_left_correction;
volatile float g_line_tracker_shadow_right_correction;
volatile bool g_line_tracker_shadow_updated;
volatile uint32_t g_line_tracker_elapsed_ms;
volatile float g_line_tracker_distance_m;
volatile bool g_line_tracker_lap_approach_active;
volatile uint8_t g_line_tracker_mode;
volatile bool g_line_tracker_start_button_pressed;
volatile bool g_line_tracker_mode_button_pressed;
volatile uint8_t g_line_tracker_calibration_state;
volatile uint8_t g_line_tracker_calibration_bits;
volatile uint8_t g_line_tracker_calibration_ok_bits;
volatile uint8_t g_line_tracker_calibration_error;
volatile bool g_line_tracker_calibration_loaded;

static line_tracker_t s_tracker;
static lap_monitor_t s_lap_monitor;
static uint32_t s_tick_overrun_baseline;
static wheel_speed_pi_t s_wheel_speed_shadow;
static line_tracker_config_t s_active_config;
static lap_monitor_config_t s_active_lap_config;
static task_mode_t s_selected_mode;
static app_button_t s_start_button;
static app_button_t s_mode_button;
static line_calibration_record_t s_saved_calibration;
static bool s_calibration_loaded;
static line_calibration_session_t s_calibration_session;
static line_tracker_calibration_view_t s_calibration_view;

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static void clear_wheel_speed_shadow_output(void)
{
    g_line_tracker_shadow_left_target = 0.0f;
    g_line_tracker_shadow_right_target = 0.0f;
    g_line_tracker_shadow_left_measured = 0.0f;
    g_line_tracker_shadow_right_measured = 0.0f;
    g_line_tracker_shadow_left_correction = 0.0f;
    g_line_tracker_shadow_right_correction = 0.0f;
    g_line_tracker_shadow_updated = false;
}

static void update_lap_debug(void)
{
    g_line_tracker_elapsed_ms = s_lap_monitor.output.elapsed_ms;
    g_line_tracker_distance_m = s_lap_monitor.output.distance_m;
    g_line_tracker_lap_approach_active =
        s_lap_monitor.output.approach_active;
}

static void apply_saved_calibration(line_tracker_config_t *config)
{
    if ((config == NULL) || !s_calibration_loaded) {
        return;
    }
    memcpy(config->white_adc, s_saved_calibration.white_adc,
           sizeof(config->white_adc));
    memcpy(config->black_adc, s_saved_calibration.black_adc,
           sizeof(config->black_adc));
    memcpy(config->x_mm, s_saved_calibration.sensor_x_mm,
           sizeof(config->x_mm));
}

static bool configure_selected_mode(void)
{
    bool tracker_ready;
    bool lap_ready;
    float steering_scale;
    float derivative_scale;

    if (!task_mode_is_valid(s_selected_mode)) {
        return false;
    }
    s_active_config = k_frozen_config;
    apply_saved_calibration(&s_active_config);
    s_active_config.speed_max_duty =
        task_mode_speed_max_duty(s_selected_mode,
                                 k_frozen_config.speed_max_duty,
                                 k_frozen_config.speed_min_duty);
    steering_scale = task_mode_steering_scale(
        s_selected_mode, k_frozen_config.speed_max_duty,
        k_frozen_config.speed_min_duty);
    derivative_scale = task_mode_derivative_scale(
        s_selected_mode, k_frozen_config.speed_max_duty,
        k_frozen_config.speed_min_duty);
    s_active_config.pid_p_yaw *= steering_scale;
    s_active_config.pid_d_yaw *= derivative_scale;
    s_active_config.yaw_limit_duty *= steering_scale;
    s_active_config.yaw_slew_step *= steering_scale;
    s_active_config.edge_yaw_duty *= steering_scale;
    s_active_lap_config = k_lap_monitor_template;
    s_active_lap_config.target_distance_m =
        task_mode_target_distance_m(s_selected_mode);
    s_active_lap_config.timeout_ms = task_mode_timeout_ms(
        s_selected_mode, k_lap_monitor_template.timeout_ms);
    tracker_ready = line_tracker_init(&s_tracker, &s_active_config);
    lap_ready = lap_monitor_init(&s_lap_monitor, &s_active_lap_config);
    wheel_speed_pi_reset(&s_wheel_speed_shadow);
    clear_wheel_speed_shadow_output();
    update_lap_debug();
    g_line_tracker_mode = (uint8_t)s_selected_mode;
    g_line_tracker_output = s_tracker.output;
    return tracker_ready && lap_ready;
}

/* Uses physical-forward encoder signs. This has no actuator side effect. */
static void update_wheel_speed_shadow(float left_target, float right_target)
{
    h2026_bsp_encoder_snapshot_t snapshot;
    const wheel_speed_pi_output_t *output;

    h2026_bsp_encoder_snapshot(&snapshot);
    wheel_speed_pi_step(&s_wheel_speed_shadow, -snapshot.left_count,
                        snapshot.right_count, left_target, right_target);
    output = &s_wheel_speed_shadow.output;
    g_line_tracker_shadow_left_target = output->left_target;
    g_line_tracker_shadow_right_target = output->right_target;
    g_line_tracker_shadow_left_measured = output->left_measured;
    g_line_tracker_shadow_right_measured = output->right_measured;
    g_line_tracker_shadow_left_correction = output->left_correction;
    g_line_tracker_shadow_right_correction = output->right_correction;
    g_line_tracker_shadow_updated = output->updated;
}

static void app_button_init(app_button_t *button, bool pressed)
{
    memset(button, 0, sizeof(*button));
    button->stable_pressed = pressed;
    button->candidate_pressed = pressed;
}

static void app_button_step(app_button_t *button, bool pressed,
                            bool *short_event, bool *hold_event)
{
    *short_event = false;
    *hold_event = false;
    if (pressed != button->candidate_pressed) {
        button->candidate_pressed = pressed;
        button->candidate_ticks = 1U;
        return;
    }
    if (button->candidate_ticks < BUTTON_DEBOUNCE_TICKS) {
        ++button->candidate_ticks;
        return;
    }
    if (button->stable_pressed != button->candidate_pressed) {
        const bool was_pressed = button->stable_pressed;

        button->stable_pressed = button->candidate_pressed;
        if (button->stable_pressed) {
            button->active_ticks = 0U;
            button->hold_sent = false;
        } else if (was_pressed && !button->hold_sent) {
            *short_event = true;
        }
    }
    if (button->stable_pressed && !button->hold_sent) {
        if (button->active_ticks < UINT16_MAX) {
            ++button->active_ticks;
        }
        if (button->active_ticks >= BUTTON_ESTOP_TICKS) {
            button->hold_sent = true;
            *hold_event = true;
        }
    }
}

static uint8_t raw_bits_for_config(
    const uint16_t raw_adc[LINE_TRACKER_SENSOR_COUNT],
    const line_tracker_config_t *config)
{
    uint8_t bits = 0U;
    uint8_t index;

    if ((raw_adc == NULL) || (config == NULL)) {
        return 0U;
    }
    for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
        const uint32_t span = (uint32_t)config->black_adc[index] -
                              config->white_adc[index];
        const uint32_t threshold = config->white_adc[index] +
            ((span * config->black_on_strength) / 1000U);

        if (raw_adc[index] >= threshold) {
            bits |= (uint8_t)(1U << index);
        }
    }
    return bits;
}

static void calibration_begin(void)
{
    lap_monitor_cancel(&s_lap_monitor);
    line_tracker_reset(&s_tracker);
    g_line_tracker_output = s_tracker.output;
    h2026_bsp_motor_coast();
    h2026_bsp_motor_arm(false);
    line_calibration_session_begin(&s_calibration_session);
}

static void calibration_save(void)
{
    line_calibration_record_t record;
    uint8_t index;

    memset(&record, 0, sizeof(record));
    memcpy(record.white_adc, s_calibration_session.white_adc,
           sizeof(record.white_adc));
    memcpy(record.black_adc, s_calibration_session.black_adc,
           sizeof(record.black_adc));
    for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
        record.sensor_x_mm[index] = k_frozen_config.x_mm[index];
    }
    line_calibration_finalize(&record);
    if (!line_calibration_store_save(&record)) {
        line_calibration_session_mark_store_failed(
            &s_calibration_session, line_calibration_store_error());
        return;
    }
    s_saved_calibration = record;
    s_calibration_loaded = true;
    g_line_tracker_calibration_loaded = true;
    if (!configure_selected_mode()) {
        line_calibration_session_mark_store_failed(
            &s_calibration_session, 0xEFU);
        return;
    }
    line_calibration_session_mark_saved(&s_calibration_session);
    /* Flash commands briefly mask interrupts. Consume any resulting pending
     * control tick and rebase the overrun counter while motors are disarmed. */
    (void)h2026_bsp_take_control_tick(&s_tick_overrun_baseline);
    g_line_tracker_tick_overruns = s_tick_overrun_baseline;
}

static void update_calibration_view(const h2026_bsp_line_sample_t *frame)
{
    memset(&s_calibration_view, 0, sizeof(s_calibration_view));
    s_calibration_view.active =
        line_calibration_session_active(&s_calibration_session);
    if ((frame == NULL) || !s_calibration_view.active) {
        return;
    }
    s_calibration_view.state = (uint8_t)s_calibration_session.state;
    s_calibration_view.ok_bits = s_calibration_session.ok_bits;
    s_calibration_view.progress_percent =
        line_calibration_session_progress_percent(&s_calibration_session);
    s_calibration_view.error = s_calibration_session.error;
    s_calibration_view.frame_valid = frame->valid;
    memcpy(s_calibration_view.raw_adc, frame->raw_adc,
           sizeof(s_calibration_view.raw_adc));
    if (s_calibration_session.state == LINE_CAL_WHITE) {
        s_calibration_view.live_bits = raw_bits_for_config(
            frame->raw_adc, &s_active_config);
    } else {
        s_calibration_view.live_bits =
            line_calibration_session_live_bits(
                &s_calibration_session, frame->raw_adc);
    }
    g_line_tracker_calibration_state = s_calibration_view.state;
    g_line_tracker_calibration_bits = s_calibration_view.live_bits;
    g_line_tracker_calibration_ok_bits = s_calibration_view.ok_bits;
    g_line_tracker_calibration_error = s_calibration_view.error;
}

static bool calibration_service(const h2026_bsp_line_sample_t *frame,
                                bool mode_event, bool mode_hold_event)
{
    if (!line_calibration_session_active(&s_calibration_session)) {
        if (!mode_hold_event ||
            (s_tracker.output.state != LINE_TRACKER_WAIT)) {
            return false;
        }
        calibration_begin();
    } else if (mode_event) {
        line_calibration_session_cancel(&s_calibration_session);
        memset(&s_calibration_view, 0, sizeof(s_calibration_view));
        (void)configure_selected_mode();
        g_line_tracker_calibration_state = LINE_CAL_IDLE;
        return true;
    } else if (mode_hold_event &&
               ((s_calibration_session.state == LINE_CAL_SAVED) ||
                (s_calibration_session.state == LINE_CAL_FAILED))) {
        calibration_begin();
    }

    if ((s_calibration_session.state == LINE_CAL_WHITE) ||
        (s_calibration_session.state == LINE_CAL_BLACK)) {
        line_calibration_session_step(&s_calibration_session,
                                      frame->raw_adc, frame->valid);
    }
    if (s_calibration_session.state == LINE_CAL_READY_TO_SAVE) {
        calibration_save();
    }
    h2026_bsp_motor_coast();
    h2026_bsp_motor_arm(false);
    g_line_tracker_output = s_tracker.output;
    update_calibration_view(frame);
    return true;
}

static void apply_output(void)
{
    float left_duty;
    float right_duty;

    if (g_line_tracker_output.state != LINE_TRACKER_RUN) {
        h2026_bsp_motor_coast();
        h2026_bsp_motor_arm(false);
        wheel_speed_pi_reset(&s_wheel_speed_shadow);
        clear_wheel_speed_shadow_output();
        g_line_tracker_applied_left_duty = 0.0f;
        g_line_tracker_applied_right_duty = 0.0f;
        return;
    }
    left_duty = g_line_tracker_output.left_duty;
    right_duty = g_line_tracker_output.right_duty;
    update_wheel_speed_shadow(left_duty, right_duty);
    /* The correction is separately bounded to +/-3 % before this final
     * actuator clamp.  It is held across the four 5 ms ticks of one speed
     * measurement window. */
#if WHEEL_SHADOW_APPLY_CORRECTION
    left_duty += g_line_tracker_shadow_left_correction;
    right_duty += g_line_tracker_shadow_right_correction;
#endif
    if (s_lap_monitor.output.approach_active) {
        if (left_duty > MISSION_APPROACH_DUTY_LIMIT) {
            left_duty = MISSION_APPROACH_DUTY_LIMIT;
        }
        if (right_duty > MISSION_APPROACH_DUTY_LIMIT) {
            right_duty = MISSION_APPROACH_DUTY_LIMIT;
        }
    }
    left_duty = clampf(left_duty, -s_active_config.duty_limit,
                       s_active_config.duty_limit);
    right_duty = clampf(right_duty, -s_active_config.duty_limit,
                        s_active_config.duty_limit);
    if (left_duty > 0.0f && left_duty < LEFT_FORWARD_DUTY_FLOOR) {
        left_duty = LEFT_FORWARD_DUTY_FLOOR;
    }
    if (right_duty > 0.0f && right_duty < RIGHT_FORWARD_DUTY_FLOOR) {
        right_duty = RIGHT_FORWARD_DUTY_FLOOR;
    }
    h2026_bsp_motor_arm(true);
    /* Positive left/right signed duty was bench-verified as physical forward. */
    h2026_bsp_motor_set_signed(left_duty, right_duty);
    g_line_tracker_applied_left_duty = left_duty;
    g_line_tracker_applied_right_duty = right_duty;
}

static void update_led(uint32_t tick)
{
    switch (g_line_tracker_output.state) {
    case LINE_TRACKER_RUN:
        h2026_bsp_led_set(true);
        break;
    case LINE_TRACKER_FAULT:
        h2026_bsp_led_set(((tick / 20U) & 1U) != 0U); /* 100 ms */
        break;
    case LINE_TRACKER_WAIT:
    default:
        h2026_bsp_led_set(((tick / 100U) & 1U) != 0U); /* 500 ms */
        break;
    }
}

int main(void)
{
    uint32_t overrun_count = 0U;
    uint32_t tick = 0U;
    h2026_bsp_encoder_snapshot_t encoder_snapshot;
    bool mode_ready;

    g_line_tracker_bsp_ready = h2026_bsp_init();
    h2026_bsp_motor_coast();
    h2026_bsp_motor_arm(false);
    (void)wheel_speed_pi_init(&s_wheel_speed_shadow,
                              &k_wheel_speed_shadow_config);
    s_calibration_loaded =
        line_calibration_store_load(&s_saved_calibration);
    g_line_tracker_calibration_loaded = s_calibration_loaded;
    s_selected_mode = TASK_MODE_H_R02;
    mode_ready = configure_selected_mode();
    line_tracker_display_init();
    app_button_init(&s_start_button,
                    h2026_bsp_start_button_pressed());
    app_button_init(&s_mode_button,
                    h2026_bsp_mode_button_pressed());
    (void)h2026_bsp_take_control_tick(&s_tick_overrun_baseline);
    g_line_tracker_tick_overruns = s_tick_overrun_baseline;
    if (!mode_ready) {
        line_tracker_force_fault(&s_tracker);
    }

    for (;;) {
        h2026_bsp_line_sample_t frame;
        line_tracker_input_t input;
        bool start_event;
        bool stop_event;
        bool mode_event;
        bool mode_hold_event;
        bool start_pressed;
        bool mode_pressed;
        bool lap_timeout;
        uint8_t index;

        if (!h2026_bsp_take_control_tick(&overrun_count)) {
            __WFE();
            continue;
        }
        ++tick;
        if (overrun_count != s_tick_overrun_baseline) {
            g_line_tracker_tick_overruns = overrun_count;
            line_tracker_force_fault(&s_tracker);
        }
        memset(&input, 0, sizeof(input));
        (void)h2026_bsp_line_scan(&frame);
        input.line_valid = frame.valid;
        for (index = 0U; index < LINE_TRACKER_SENSOR_COUNT; ++index) {
            input.raw_adc[index] = frame.raw_adc[index];
            g_line_tracker_raw_adc[index] = frame.raw_adc[index];
        }
        start_pressed = h2026_bsp_start_button_pressed();
        mode_pressed = h2026_bsp_mode_button_pressed();
        g_line_tracker_start_button_pressed = start_pressed;
        g_line_tracker_mode_button_pressed = mode_pressed;
        app_button_step(&s_start_button, start_pressed,
                        &start_event, &stop_event);
        app_button_step(&s_mode_button, mode_pressed,
                        &mode_event, &mode_hold_event);
        if (calibration_service(&frame, mode_event, mode_hold_event)) {
            update_lap_debug();
            apply_output();
            update_led(tick);
            line_tracker_display_service(
                s_tracker.output.state,
                s_lap_monitor.output.elapsed_ms,
                s_lap_monitor.output.distance_m,
                (uint8_t)s_selected_mode,
                &s_calibration_view);
            continue;
        }
        if (mode_event &&
            (s_tracker.output.state == LINE_TRACKER_WAIT)) {
            s_selected_mode = task_mode_next(s_selected_mode);
            if (!configure_selected_mode()) {
                line_tracker_force_fault(&s_tracker);
            }
        }
        if (start_event &&
            (s_tracker.output.state == LINE_TRACKER_FAULT)) {
            /* Acknowledge a previous distance timeout before the tracker
             * consumes START as FAULT -> WAIT. */
            lap_monitor_cancel(&s_lap_monitor);
        }
        h2026_bsp_encoder_snapshot(&encoder_snapshot);
        if (start_event && (s_tracker.output.state == LINE_TRACKER_WAIT)) {
            lap_monitor_start(&s_lap_monitor,
                              encoder_snapshot.left_count,
                              encoder_snapshot.right_count,
                              h2026_bsp_millis());
        }
        lap_monitor_step(&s_lap_monitor,
                         encoder_snapshot.left_count,
                         encoder_snapshot.right_count,
                         h2026_bsp_millis());
        lap_timeout = (s_lap_monitor.output.state == LAP_MONITOR_TIMEOUT);
        if (s_lap_monitor.output.state == LAP_MONITOR_COMPLETE) {
            stop_event = true;
        }
        if (stop_event) {
            lap_monitor_cancel(&s_lap_monitor);
        }
        input.start_event = start_event;
        input.stop_event = stop_event;
        line_tracker_step(&s_tracker, &input);
        if (lap_timeout) {
            line_tracker_force_fault(&s_tracker);
        }
        if ((s_tracker.output.state != LINE_TRACKER_RUN) &&
            (s_lap_monitor.output.state == LAP_MONITOR_RUNNING)) {
            lap_monitor_cancel(&s_lap_monitor);
        }
        update_lap_debug();
        g_line_tracker_output = s_tracker.output;
        g_line_tracker_debug_p_duty = s_tracker.output.debug_p_duty;
        g_line_tracker_debug_d_duty = s_tracker.output.debug_d_duty;
        g_line_tracker_debug_pd_target_yaw =
            s_tracker.output.debug_pd_target_yaw;
        g_line_tracker_debug_final_yaw = s_tracker.output.debug_final_yaw;
        g_line_tracker_debug_base_duty = s_tracker.output.debug_base_duty;
        g_line_tracker_debug_edge_blend = s_tracker.output.debug_edge_blend;
        apply_output();
        update_led(tick);
        line_tracker_display_service(s_tracker.output.state,
                                     s_lap_monitor.output.elapsed_ms,
                                     s_lap_monitor.output.distance_m,
                                     (uint8_t)s_selected_mode,
                                     &s_calibration_view);
    }
}
