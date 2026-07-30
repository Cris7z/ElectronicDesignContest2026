/*
 * C07A line_tracker: the deliberately small first-stage vehicle program.
 *
 * Scope: BLS -> eight-channel CD4051 scan -> weighted PD -> TB6612.
 * Not included: lap counting, finish-line logic, MPU6050, K230, beam control,
 * OLED refresh and automatic Flash calibration.
 */
#include "line_tracker.h"

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

/* All values below are measured on the installed 2026-07-30 vehicle. */
static const line_tracker_config_t k_config = {
    .white_adc = {174U, 174U, 172U, 173U, 171U, 172U, 171U, 171U},
    .black_adc = {4095U, 4095U, 4095U, 4095U,
                  4095U, 4095U, 4095U, 4095U},
    .x_mm = {-35.0f, -25.0f, -15.0f, -5.0f,
              5.0f, 15.0f, 25.0f, 35.0f},
    .black_on_strength = 600U,
    .black_off_strength = 400U,
    .max_track_black_count = 3U,
    .lost_limit_ticks = 60U,       /* 300 ms at the fixed 5 ms tick */
    .center_gap_limit_ticks = 0U,  /* Disabled until the real track proves a centre gap. */
    .center_gap_error_limit = 0.20f,
    /* PWM polarity is hardware-verified; 8.5 % is the lowest matched crawl. */
    .base_duty = 0.085f,
    .wide_line_duty = 0.080f,
    .search_inner_duty = 0.060f,
    .search_outer_duty = 0.100f,
    .start_ramp_step = 0.0010f,
    .kp = 0.040f,
    .kd = 0.008f,
    .correction_limit = 0.012f,
    .duty_limit = 0.110f
};

/* Intentionally visible in CCS/XDS110 Expressions during track tuning. */
volatile line_tracker_output_t g_line_tracker_output;
volatile uint16_t g_line_tracker_raw_adc[LINE_TRACKER_SENSOR_COUNT];
volatile uint32_t g_line_tracker_tick_overruns;
volatile bool g_line_tracker_bsp_ready;
volatile float g_line_tracker_applied_left_duty;
volatile float g_line_tracker_applied_right_duty;

static line_tracker_t s_tracker;
static uint32_t s_tick_overrun_baseline;
static bool s_button_stable;
static bool s_button_candidate;
static uint8_t s_button_candidate_ticks;
static uint16_t s_button_active_ticks;
static bool s_button_estop_sent;

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
        g_line_tracker_applied_left_duty = 0.0f;
        g_line_tracker_applied_right_duty = 0.0f;
        return;
    }
    left_duty = g_line_tracker_output.left_duty;
    right_duty = g_line_tracker_output.right_duty;
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
