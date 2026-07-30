/* Raised-chassis only: measure the minimum useful PWM for each wheel. */
#include "h2026_bsp.h"
#include "ti_msp_dl_config.h"

#include <stdbool.h>
#include <stdint.h>

#define BUTTON_DEBOUNCE_TICKS 6U
#define STEP_TICKS 100U /* 500 ms at the fixed 5 ms control tick. */
#define STEP_COUNT 7U

static const float k_duty[STEP_COUNT] = {
    0.02f, 0.03f, 0.04f, 0.05f, 0.06f, 0.08f, 0.10f
};

/* Read these with capture_deadband_results.py after the automatic stop. */
volatile uint8_t g_deadband_stage;
volatile bool g_deadband_running;
volatile bool g_deadband_done;
volatile int64_t g_deadband_left_delta[STEP_COUNT];
volatile int64_t g_deadband_right_delta[STEP_COUNT];

static bool s_button_stable;
static bool s_button_candidate;
static uint8_t s_button_ticks;
static uint16_t s_step_ticks;
static int64_t s_left_start;
static int64_t s_right_start;

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

static void begin_stage(void)
{
    h2026_bsp_encoder_snapshot_t snapshot;

    h2026_bsp_encoder_snapshot(&snapshot);
    s_left_start = snapshot.left_count;
    s_right_start = snapshot.right_count;
    s_step_ticks = 0U;
    h2026_bsp_motor_arm(true);
    h2026_bsp_motor_set_signed(k_duty[g_deadband_stage],
                               k_duty[g_deadband_stage]);
}

static void finish_stage(void)
{
    h2026_bsp_encoder_snapshot_t snapshot;

    h2026_bsp_encoder_snapshot(&snapshot);
    g_deadband_left_delta[g_deadband_stage] = snapshot.left_count - s_left_start;
    g_deadband_right_delta[g_deadband_stage] = snapshot.right_count - s_right_start;
    ++g_deadband_stage;
    if (g_deadband_stage >= STEP_COUNT) {
        h2026_bsp_motor_coast();
        h2026_bsp_motor_arm(false);
        g_deadband_running = false;
        g_deadband_done = true;
    } else {
        begin_stage();
    }
}

int main(void)
{
    uint32_t overrun_count = 0U;

    (void)h2026_bsp_init();
    h2026_bsp_motor_coast();
    h2026_bsp_motor_arm(false);
    s_button_stable = h2026_bsp_start_level();
    s_button_candidate = s_button_stable;
    (void)h2026_bsp_take_control_tick(&overrun_count);

    for (;;) {
        if (!h2026_bsp_take_control_tick(&overrun_count)) {
            __WFE();
            continue;
        }
        if (g_deadband_running) {
            if (button_released_event()) {
                h2026_bsp_motor_coast();
                h2026_bsp_motor_arm(false);
                g_deadband_running = false;
                g_deadband_done = true;
            } else if (++s_step_ticks >= STEP_TICKS) {
                finish_stage();
            }
        } else {
            h2026_bsp_motor_coast();
            h2026_bsp_motor_arm(false);
            if (button_released_event()) {
                g_deadband_stage = 0U;
                g_deadband_done = false;
                g_deadband_running = true;
                begin_stage();
            }
        }
        h2026_bsp_led_set(g_deadband_running);
    }
}
