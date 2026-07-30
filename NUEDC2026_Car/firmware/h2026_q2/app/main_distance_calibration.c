/*
 * Ground distance calibration for the bench-verified C07A/TB6612 mapping.
 * A rising BLS edge drives both wheels forward at a deliberately low duty for
 * a fixed short interval.  A later BLS press is an immediate abort.  The
 * operator measures the axle-centre displacement, then divides it by L/R.
 */
#include "h2026_q2_display.h"

#include "../bsp/h2026_bsp.h"

#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#define BUTTON_DEBOUNCE_TICKS  (6U)   /* 30 ms */
#define STARTUP_QUIET_TICKS   (100U)  /* 500 ms */
#define DRIVE_TICKS           (600U)  /* 3.0 s */
#define LEFT_DRIVE_DUTY      (0.25f)
/* Bench count rates were 908:1361 at equal duty, so start right at 2/3. */
#define RIGHT_DRIVE_DUTY     (0.1667f)
#define BALANCE_PERIOD_TICKS  (20U)   /* 100 ms */
#define BALANCE_GAIN       (0.00020f)
#define RIGHT_DUTY_MIN       (0.100f)
#define RIGHT_DUTY_MAX       (0.250f)

typedef enum {
    DISTANCE_CAL_QUIET = 0,
    DISTANCE_CAL_READY,
    DISTANCE_CAL_DRIVE,
    DISTANCE_CAL_DONE,
    DISTANCE_CAL_ABORT
} distance_cal_stage_t;

static distance_cal_stage_t s_stage;
static bool s_button_stable;
static bool s_button_candidate;
static uint16_t s_button_candidate_ticks;
static uint32_t s_ticks;
static uint32_t s_start_tick;
static uint32_t s_overrun_baseline;
static int64_t s_left_start;
static int64_t s_right_start;
static int64_t s_left_forward_count;
static int64_t s_right_forward_count;
static uint32_t s_left_invalid_start;
static uint32_t s_right_invalid_start;
static uint32_t s_left_invalid;
static uint32_t s_right_invalid;
static int64_t s_left_balance_count;
static int64_t s_right_balance_count;
static float s_right_duty;
static bool s_display_job_active;

/* Return only a debounced press, never the release edge. */
static bool button_pressed(void)
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
        return s_button_stable;
    }
    return false;
}

static void finish(distance_cal_stage_t final_stage)
{
    h2026_bsp_encoder_snapshot_t encoders;

    h2026_bsp_motor_coast();
    h2026_bsp_motor_arm(false);
    h2026_bsp_encoder_snapshot(&encoders);
    /* Normalise by the bench-verified physical-forward signs. */
    s_left_forward_count = -(encoders.left_count - s_left_start);
    s_right_forward_count = encoders.right_count - s_right_start;
    s_left_invalid = encoders.left_invalid_transitions - s_left_invalid_start;
    s_right_invalid = encoders.right_invalid_transitions - s_right_invalid_start;
    s_stage = final_stage;
}

static void begin(void)
{
    h2026_bsp_encoder_snapshot_t encoders;

    s_left_start = encoders.left_count;
    s_right_start = encoders.right_count;
    s_left_invalid_start = encoders.left_invalid_transitions;
    s_right_invalid_start = encoders.right_invalid_transitions;
    s_left_forward_count = 0;
    s_right_forward_count = 0;
    s_left_invalid = 0U;
    s_right_invalid = 0U;
    s_left_balance_count = encoders.left_count;
    s_right_balance_count = encoders.right_count;
    s_right_duty = RIGHT_DRIVE_DUTY;
    s_start_tick = s_ticks;
    h2026_bsp_motor_arm(true);
    h2026_bsp_motor_set_signed(LEFT_DRIVE_DUTY, s_right_duty);
    s_stage = DISTANCE_CAL_DRIVE;
}

/* Keep the two wheels at the same measured rotation rate during the run. */
static void balance_wheel_speeds(void)
{
    h2026_bsp_encoder_snapshot_t encoders;
    int64_t left_delta;
    int64_t right_delta;
    int64_t error;

    if (((uint32_t)(s_ticks - s_start_tick) == 0U) ||
        (((uint32_t)(s_ticks - s_start_tick) % BALANCE_PERIOD_TICKS) != 0U)) {
        return;
    }
    h2026_bsp_encoder_snapshot(&encoders);
    left_delta = -(encoders.left_count - s_left_balance_count);
    right_delta = encoders.right_count - s_right_balance_count;
    s_left_balance_count = encoders.left_count;
    s_right_balance_count = encoders.right_count;
    error = left_delta - right_delta;
    s_right_duty += (float)error * BALANCE_GAIN;
    if (s_right_duty < RIGHT_DUTY_MIN) {
        s_right_duty = RIGHT_DUTY_MIN;
    } else if (s_right_duty > RIGHT_DUTY_MAX) {
        s_right_duty = RIGHT_DUTY_MAX;
    }
    h2026_bsp_motor_set_signed(LEFT_DRIVE_DUTY, s_right_duty);
}

static void display_service(void)
{
    if (!h2026_bsp_display_refresh_pending()) {
        return;
    }
    if (!s_display_job_active) {
        h2026_q2_display_render_distance_calibration(
            (uint8_t)s_stage, s_left_forward_count, s_right_forward_count,
            s_left_invalid, s_right_invalid);
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
    (void)h2026_bsp_take_control_tick(&overrun_count);
    s_overrun_baseline = overrun_count;
    s_button_stable = h2026_bsp_start_level();
    s_button_candidate = s_button_stable;
    s_stage = DISTANCE_CAL_QUIET;

    for (;;) {
        bool press;

        if (!h2026_bsp_take_control_tick(&overrun_count)) {
            __WFE();
            continue;
        }
        ++s_ticks;
        press = button_pressed();

        if ((s_stage == DISTANCE_CAL_QUIET) &&
            (s_ticks >= STARTUP_QUIET_TICKS)) {
            s_stage = DISTANCE_CAL_READY;
        }
        if (s_stage == DISTANCE_CAL_READY) {
            if (press) {
                begin();
            }
        } else if (s_stage == DISTANCE_CAL_DRIVE) {
            if (press) {
                finish(DISTANCE_CAL_ABORT);
            } else if ((uint32_t)(s_ticks - s_start_tick) >= DRIVE_TICKS) {
                finish(DISTANCE_CAL_DONE);
            } else {
                balance_wheel_speeds();
            }
        }
        if (overrun_count != s_overrun_baseline) {
            finish(DISTANCE_CAL_ABORT);
        }
        h2026_bsp_led_set(s_stage == DISTANCE_CAL_DRIVE);
        display_service();
    }
}
