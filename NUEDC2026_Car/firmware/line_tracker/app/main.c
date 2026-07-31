/*
 * C07A line_tracker: the deliberately small first-stage vehicle program.
 *
 * Scope: BLS -> eight-channel CD4051 scan -> weighted PD -> TB6612.
 * Not included: lap counting, finish-line logic, MPU6050, K230, beam control,
 * OLED refresh and automatic Flash calibration.
 */
#include "line_tracker.h"
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

/* All values below are measured on the installed 2026-07-30 vehicle. */
static const line_tracker_config_t k_config = {
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
    .edge_yaw_duty = 0.300f,        /* edge yaw: 60 % -> 30 % */
    .yaw_gain_min = 0.90f,
    .yaw_gain_start_weight = 1.0f,
    .yaw_gain_full_weight = 4.0f,
    .edge_blend_start_weight = 5.0f,
    .edge_blend_full_weight = 7.0f,
    .steering_polarity = -1.0f,     /* Reversed after the 2026-07-31 chassis test. */
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

/* Intentionally visible in CCS/XDS110 Expressions during track tuning. */
volatile line_tracker_output_t g_line_tracker_output;
volatile uint16_t g_line_tracker_raw_adc[LINE_TRACKER_SENSOR_COUNT];
volatile uint32_t g_line_tracker_tick_overruns;
volatile bool g_line_tracker_bsp_ready;
volatile float g_line_tracker_applied_left_duty;
volatile float g_line_tracker_applied_right_duty;
volatile float g_line_tracker_shadow_left_target;
volatile float g_line_tracker_shadow_right_target;
volatile float g_line_tracker_shadow_left_measured;
volatile float g_line_tracker_shadow_right_measured;
volatile float g_line_tracker_shadow_left_correction;
volatile float g_line_tracker_shadow_right_correction;
volatile bool g_line_tracker_shadow_updated;

static line_tracker_t s_tracker;
static uint32_t s_tick_overrun_baseline;
static bool s_button_stable;
static bool s_button_candidate;
static uint8_t s_button_candidate_ticks;
static uint16_t s_button_active_ticks;
static bool s_button_estop_sent;
static wheel_speed_pi_t s_wheel_speed_shadow;

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

static void update_button(bool *start_event, bool *stop_event)
{
    const bool level = h2026_bsp_start_level(); /* PA18: pressed == high */

    *start_event = false;
    *stop_event = false;
    if (level != s_button_candidate) {
        s_button_candidate = level;
        s_button_candidate_ticks = 1U;
        return;
    }
    if (s_button_candidate_ticks < BUTTON_DEBOUNCE_TICKS) {
        ++s_button_candidate_ticks;
        return;
    }
    if (s_button_stable != s_button_candidate) {
        const bool was_pressed = s_button_stable;

        s_button_stable = s_button_candidate;
        if (s_button_stable) {
            s_button_active_ticks = 0U;
            s_button_estop_sent = false;
        } else if (was_pressed && !s_button_estop_sent) {
            /* Short release starts from WAIT and resets after FAULT. */
            *start_event = true;
        }
    }
    if (s_button_stable && !s_button_estop_sent) {
        if (s_button_active_ticks < UINT16_MAX) {
            ++s_button_active_ticks;
        }
        if (s_button_active_ticks >= BUTTON_ESTOP_TICKS) {
            s_button_estop_sent = true;
            *stop_event = true;
        }
    }
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
    left_duty = clampf(left_duty, -k_config.duty_limit, k_config.duty_limit);
    right_duty = clampf(right_duty, -k_config.duty_limit, k_config.duty_limit);
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

    g_line_tracker_bsp_ready = h2026_bsp_init();
    h2026_bsp_motor_coast();
    h2026_bsp_motor_arm(false);
    (void)line_tracker_init(&s_tracker, &k_config);
    (void)wheel_speed_pi_init(&s_wheel_speed_shadow,
                              &k_wheel_speed_shadow_config);
    clear_wheel_speed_shadow_output();
    s_button_stable = h2026_bsp_start_level();
    s_button_candidate = s_button_stable;
    (void)h2026_bsp_take_control_tick(&s_tick_overrun_baseline);
    g_line_tracker_tick_overruns = s_tick_overrun_baseline;

    for (;;) {
        h2026_bsp_line_sample_t frame;
        line_tracker_input_t input;
        bool start_event;
        bool stop_event;
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
        update_button(&start_event, &stop_event);
        input.start_event = start_event;
        input.stop_event = stop_event;
        line_tracker_step(&s_tracker, &input);
        g_line_tracker_output = s_tracker.output;
        apply_output();
        update_led(tick);
    }
}
