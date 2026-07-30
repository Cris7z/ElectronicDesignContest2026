/*
 * Raised-chassis wheel-speed-PI bench image.
 *
 * It intentionally bypasses CD4051 and line_tracker: a BLS short press
 * toggles equal fixed forward duty for both wheels.  The speed PI stays in
 * shadow mode, so it records the correction but never changes motor PWM.
 */
#include "wheel_speed_pi.h"

#include "h2026_bsp.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#define BUTTON_DEBOUNCE_TICKS 6U
#define BENCH_DUTY 0.350f
#define WHEEL_SHADOW_COUNTS_PER_COMMAND_WINDOW 163.256f

static const wheel_speed_pi_config_t k_wheel_speed_shadow_config = {
    .counts_per_command_window = WHEEL_SHADOW_COUNTS_PER_COMMAND_WINDOW,
    .measurement_filter_alpha = 0.50f,
    .kp = 0.30f,
    .ki_per_second = 0.0f,
    .integral_limit = 0.040f,
    .correction_limit = 0.030f,
    .enable_threshold = 0.100f
};

/* Names deliberately match the production monitor for one common capture
 * script.  All values are diagnostic; neither correction reaches PWM. */
volatile float g_line_tracker_applied_left_duty;
volatile float g_line_tracker_applied_right_duty;
volatile float g_line_tracker_wheel_balance_trim;
volatile float g_line_tracker_shadow_left_target;
volatile float g_line_tracker_shadow_right_target;
volatile float g_line_tracker_shadow_left_measured;
volatile float g_line_tracker_shadow_right_measured;
volatile float g_line_tracker_shadow_left_correction;
volatile float g_line_tracker_shadow_right_correction;
volatile bool g_line_tracker_shadow_updated;
volatile uint8_t g_wheel_pi_bench_state; /* 0=WAIT, 1=RUN */

static wheel_speed_pi_t s_wheel_speed_shadow;
static bool s_button_stable;
static bool s_button_candidate;
static uint8_t s_button_ticks;

static void clear_shadow_output(void)
{
    g_line_tracker_shadow_left_target = 0.0f;
    g_line_tracker_shadow_right_target = 0.0f;
    g_line_tracker_shadow_left_measured = 0.0f;
    g_line_tracker_shadow_right_measured = 0.0f;
    g_line_tracker_shadow_left_correction = 0.0f;
    g_line_tracker_shadow_right_correction = 0.0f;
    g_line_tracker_shadow_updated = false;
}

static bool button_released_event(void)
{
    const bool level = h2026_bsp_start_level();

    if (level != s_button_candidate) {
        s_button_candidate = level;
        s_button_ticks = 1U;
        return false;
    }
    if (s_button_ticks < BUTTON_DEBOUNCE_TICKS) {
        ++s_button_ticks;
        return false;
    }
    if (s_button_stable != s_button_candidate) {
        const bool was_pressed = s_button_stable;

        s_button_stable = s_button_candidate;
        return was_pressed && !s_button_stable;
    }
    return false;
}

static void update_shadow(void)
{
    h2026_bsp_encoder_snapshot_t snapshot;
    const wheel_speed_pi_output_t *output;

    h2026_bsp_encoder_snapshot(&snapshot);
    /* Bench-verified physical-forward signs: left=-, right=+. */
    wheel_speed_pi_step(&s_wheel_speed_shadow, -snapshot.left_count,
                        snapshot.right_count, BENCH_DUTY, BENCH_DUTY);
    output = &s_wheel_speed_shadow.output;
    g_line_tracker_shadow_left_target = output->left_target;
    g_line_tracker_shadow_right_target = output->right_target;
    g_line_tracker_shadow_left_measured = output->left_measured;
    g_line_tracker_shadow_right_measured = output->right_measured;
    g_line_tracker_shadow_left_correction = output->left_correction;
    g_line_tracker_shadow_right_correction = output->right_correction;
    g_line_tracker_shadow_updated = output->updated;
}

int main(void)
{
    uint32_t overrun_count = 0U;

    (void)h2026_bsp_init();
    h2026_bsp_motor_coast();
    h2026_bsp_motor_arm(false);
    (void)wheel_speed_pi_init(&s_wheel_speed_shadow,
                              &k_wheel_speed_shadow_config);
    clear_shadow_output();
    s_button_stable = h2026_bsp_start_level();
    s_button_candidate = s_button_stable;
    (void)h2026_bsp_take_control_tick(&overrun_count);

    for (;;) {
        if (!h2026_bsp_take_control_tick(&overrun_count)) {
            __WFE();
            continue;
        }
        if (button_released_event()) {
            g_wheel_pi_bench_state ^= 1U;
        }
        if (g_wheel_pi_bench_state != 0U) {
            update_shadow();
            h2026_bsp_motor_arm(true);
            h2026_bsp_motor_set_signed(BENCH_DUTY, BENCH_DUTY);
            g_line_tracker_applied_left_duty = BENCH_DUTY;
            g_line_tracker_applied_right_duty = BENCH_DUTY;
        } else {
            h2026_bsp_motor_coast();
            h2026_bsp_motor_arm(false);
            wheel_speed_pi_reset(&s_wheel_speed_shadow);
            clear_shadow_output();
            g_line_tracker_applied_left_duty = 0.0f;
            g_line_tracker_applied_right_duty = 0.0f;
        }
        h2026_bsp_led_set(g_wheel_pi_bench_state != 0U);
    }
}
