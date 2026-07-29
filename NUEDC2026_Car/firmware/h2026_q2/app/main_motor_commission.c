/*
 * Deliberately separate, one-shot bench program for TB6612 motor commissioning.
 * It never starts on power-up: after a quiet period, each BLS tap runs exactly
 * one wheel at 35 % electrical duty for 2.0 s and then coasts.  The normal Q2
 * mission firmware and its calibration locks are not changed.
 */
#include "h2026_q2_display.h"

#include "../bsp/h2026_bsp.h"

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#ifndef H2026_MOTOR_TEST_AUTORUN
#define H2026_MOTOR_TEST_AUTORUN 0
#endif

#define CONTROL_PERIOD_MS       H2026_BSP_CONTROL_PERIOD_MS
#define BLS_STARTUP_QUIET_TICKS (100U)  /* 500 ms */
#define AUTO_STARTUP_QUIET_TICKS (3000U) /* 15 s */
#define BUTTON_DEBOUNCE_TICKS   (6U)    /* 30 ms */
#define BLS_DRIVE_TICKS         (400U)  /* 2 s */
#define AUTO_DRIVE_TICKS        (600U)  /* 3 s */
#define BLS_NEXT_TEST_GUARD_TICKS (200U) /* 1 s */
#define AUTO_NEXT_TEST_GUARD_TICKS (400U) /* 2 s */
#define MOTOR_TEST_DUTY         (0.35f)

#if H2026_MOTOR_TEST_AUTORUN
#define STARTUP_QUIET_TICKS AUTO_STARTUP_QUIET_TICKS
#define DRIVE_TICKS AUTO_DRIVE_TICKS
#define NEXT_TEST_GUARD_TICKS AUTO_NEXT_TEST_GUARD_TICKS
#else
#define STARTUP_QUIET_TICKS BLS_STARTUP_QUIET_TICKS
#define DRIVE_TICKS BLS_DRIVE_TICKS
#define NEXT_TEST_GUARD_TICKS BLS_NEXT_TEST_GUARD_TICKS
#endif

typedef enum {
    MOTOR_TEST_QUIET = 0,
    MOTOR_TEST_READY_LEFT,
    MOTOR_TEST_DRIVE_LEFT,
    MOTOR_TEST_READY_RIGHT,
    MOTOR_TEST_DRIVE_RIGHT,
    MOTOR_TEST_DONE
} motor_test_stage_t;

/* Volatile so a later SWD read can record direction/count signs. */
static volatile motor_test_stage_t s_stage;
static volatile int64_t s_left_encoder_delta;
static volatile int64_t s_right_encoder_delta;
static volatile uint32_t s_completed_tests;

static bool s_button_stable;
static bool s_button_candidate;
static uint16_t s_button_candidate_ticks;
static uint32_t s_ticks;
static uint32_t s_drive_start_tick;
static uint32_t s_next_test_allowed_tick;
static uint32_t s_overrun_baseline;
static int64_t s_left_encoder_start;
static int64_t s_right_encoder_start;
static bool s_display_job_active;

/* A debounced change is sufficient: the physical BLS active polarity is not
 * yet calibrated, and this test must not assume it. */
static bool button_event(void)
{
    const bool level = h2026_bsp_start_level();

    if (level != s_button_candidate) {
        s_button_candidate = level;
        s_button_candidate_ticks = 1U;
        return false;
    }
    if (s_button_candidate_ticks < BUTTON_DEBOUNCE_TICKS) {
        ++s_button_candidate_ticks;
        return false;
    }
    if (s_button_stable != s_button_candidate) {
        s_button_stable = s_button_candidate;
        return true;
    }
    return false;
}

/* One/two/three LED pulses distinguish ready-left/ready-right/done. */
static void update_led(void)
{
    const uint32_t phase = s_ticks % 200U; /* one-second pattern */
    uint32_t pulse_count = 0U;
    bool on = false;

    switch (s_stage) {
    case MOTOR_TEST_DRIVE_LEFT:
    case MOTOR_TEST_DRIVE_RIGHT:
        h2026_bsp_led_set(true);
        return;
    case MOTOR_TEST_READY_LEFT:
        pulse_count = 1U;
        break;
    case MOTOR_TEST_READY_RIGHT:
        pulse_count = 2U;
        break;
    case MOTOR_TEST_DONE:
        pulse_count = 3U;
        break;
    default:
        h2026_bsp_led_set(false);
        return;
    }
    for (uint32_t pulse = 0U; pulse < pulse_count; ++pulse) {
        const uint32_t start = pulse * 40U;
        if ((phase >= start) && (phase < (start + 20U))) {
            on = true;
        }
    }
    h2026_bsp_led_set(on);
}

static void begin_test(motor_test_stage_t stage)
{
    h2026_bsp_encoder_snapshot_t encoders;

    h2026_bsp_encoder_snapshot(&encoders);
    s_left_encoder_start = encoders.left_count;
    s_right_encoder_start = encoders.right_count;
    s_drive_start_tick = s_ticks;
    s_stage = stage;
    h2026_bsp_motor_arm(true);
    if (stage == MOTOR_TEST_DRIVE_LEFT) {
        h2026_bsp_motor_set_signed(MOTOR_TEST_DUTY, 0.0f);
    } else {
        h2026_bsp_motor_set_signed(0.0f, MOTOR_TEST_DUTY);
    }
}

static void finish_test(void)
{
    h2026_bsp_encoder_snapshot_t encoders;

    h2026_bsp_motor_coast();
    h2026_bsp_motor_arm(false);
    h2026_bsp_encoder_snapshot(&encoders);
    if (s_stage == MOTOR_TEST_DRIVE_LEFT) {
        s_left_encoder_delta = encoders.left_count - s_left_encoder_start;
        s_stage = MOTOR_TEST_READY_RIGHT;
    } else {
        s_right_encoder_delta = encoders.right_count - s_right_encoder_start;
        s_stage = MOTOR_TEST_DONE;
    }
    ++s_completed_tests;
    s_next_test_allowed_tick = s_ticks + NEXT_TEST_GUARD_TICKS;
}

static void display_service(void)
{
    if (!h2026_bsp_display_refresh_pending()) {
        return;
    }
    if (!s_display_job_active) {
        h2026_q2_display_render_motor_commission((uint8_t)s_stage,
                                                  s_left_encoder_delta,
                                                  s_right_encoder_delta,
                                                  s_completed_tests);
        s_display_job_active = true;
    }
    if (h2026_q2_display_flush_one_page()) {
        s_display_job_active = false;
        h2026_bsp_display_refresh_complete();
    }
}

int main(void)
{
    uint32_t overrun_count = 0U;

    if (!h2026_bsp_init()) {
        for (;;) {
            h2026_bsp_motor_arm(false);
            h2026_bsp_led_set(true);
            __WFE();
        }
    }

    h2026_bsp_motor_arm(false);
    h2026_q2_display_init();

    /* OLED reset/clear is intentionally foreground-blocking.  Discard its
     * boot-time scheduler debt; only overruns after the test is ready are a
     * reason to force the outputs safe. */
    (void)h2026_bsp_take_control_tick(&overrun_count);
    s_overrun_baseline = overrun_count;
    s_button_stable = h2026_bsp_start_level();
    s_button_candidate = s_button_stable;
    s_stage = MOTOR_TEST_QUIET;
    s_left_encoder_delta = 0;
    s_right_encoder_delta = 0;
    s_completed_tests = 0U;
    s_display_job_active = false;

    for (;;) {
        bool input_event;

        if (!h2026_bsp_take_control_tick(&overrun_count)) {
            __WFE();
            continue;
        }

        ++s_ticks;
        input_event = button_event();

        if ((s_stage == MOTOR_TEST_QUIET) &&
            (s_ticks >= STARTUP_QUIET_TICKS)) {
#if H2026_MOTOR_TEST_AUTORUN
            begin_test(MOTOR_TEST_DRIVE_LEFT);
#else
            s_stage = MOTOR_TEST_READY_LEFT;
            s_next_test_allowed_tick = s_ticks;
#endif
        }

        if ((s_stage == MOTOR_TEST_DRIVE_LEFT) ||
            (s_stage == MOTOR_TEST_DRIVE_RIGHT)) {
            if ((uint32_t)(s_ticks - s_drive_start_tick) >= DRIVE_TICKS) {
                finish_test();
            }
        } else if (((s_stage == MOTOR_TEST_READY_LEFT) ||
                    (s_stage == MOTOR_TEST_READY_RIGHT)) &&
                   (s_ticks >= s_next_test_allowed_tick) &&
#if H2026_MOTOR_TEST_AUTORUN
                   ((s_stage == MOTOR_TEST_READY_RIGHT) || input_event)) {
#else
                   input_event) {
#endif
            if (s_stage == MOTOR_TEST_READY_LEFT) {
                begin_test(MOTOR_TEST_DRIVE_LEFT);
            } else {
                begin_test(MOTOR_TEST_DRIVE_RIGHT);
            }
        }

        /* No scheduler overrun may leave motor outputs active. */
        if (overrun_count != s_overrun_baseline) {
            h2026_bsp_motor_coast();
            h2026_bsp_motor_arm(false);
            s_stage = MOTOR_TEST_DONE;
        }
        update_led();
        display_service();
    }
}
