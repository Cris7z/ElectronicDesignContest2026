#include "wheel_speed_pi.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static const wheel_speed_pi_config_t k_config = {
    .counts_per_command_window = 160.0f,
    .measurement_filter_alpha = 1.0f,
    .kp = 0.30f,
    .ki_per_second = 0.50f,
    .integral_limit = 0.04f,
    .correction_limit = 0.06f,
    .enable_threshold = 0.10f
};

/* Must match the active wheel P-only correction in app/main.c. */
static const wheel_speed_pi_config_t k_live_config = {
    .counts_per_command_window = 163.256f,
    .measurement_filter_alpha = 0.50f,
    .kp = 0.30f,
    .ki_per_second = 0.0f,
    .integral_limit = 0.040f,
    .correction_limit = 0.030f,
    .enable_threshold = 0.100f
};

static void run_window(wheel_speed_pi_t *controller, int64_t *left_count,
                       int64_t *right_count, int64_t left_step,
                       int64_t right_step, float left_target,
                       float right_target)
{
    unsigned tick;

    for (tick = 0U; tick < WHEEL_SPEED_PI_WINDOW_TICKS; ++tick) {
        *left_count += left_step;
        *right_count += right_step;
        wheel_speed_pi_step(controller, *left_count, *right_count,
                            left_target, right_target);
    }
}

static void test_window_and_matching_speed(void)
{
    wheel_speed_pi_t controller;
    int64_t left_count = 0;
    int64_t right_count = 0;

    assert(wheel_speed_pi_init(&controller, &k_config));
    wheel_speed_pi_step(&controller, left_count, right_count, 0.45f, 0.45f);
    run_window(&controller, &left_count, &right_count, 18, 18, 0.45f, 0.45f);
    assert(controller.output.updated);
    assert(fabsf(controller.output.left_target - 0.45f) < 0.0001f);
    assert(fabsf(controller.output.left_measured - 0.45f) < 0.0001f);
    assert(fabsf(controller.output.left_correction) < 0.0001f);
    assert(fabsf(controller.output.right_correction) < 0.0001f);
}

static void test_independent_corrections(void)
{
    wheel_speed_pi_t controller;
    int64_t left_count = 0;
    int64_t right_count = 0;

    assert(wheel_speed_pi_init(&controller, &k_config));
    wheel_speed_pi_step(&controller, left_count, right_count, 0.50f, 0.50f);
    run_window(&controller, &left_count, &right_count, 16, 24, 0.50f, 0.50f);
    assert(controller.output.left_correction > 0.0f);
    assert(controller.output.right_correction < 0.0f);
}

static void test_saturation_and_zero_reset(void)
{
    wheel_speed_pi_t controller;
    int64_t left_count = 0;
    int64_t right_count = 0;

    assert(wheel_speed_pi_init(&controller, &k_config));
    wheel_speed_pi_step(&controller, left_count, right_count, 0.50f, 0.50f);
    run_window(&controller, &left_count, &right_count, 0, 0, 0.50f, 0.50f);
    assert(fabsf(controller.output.left_correction -
                 k_config.correction_limit) < 0.0001f);
    run_window(&controller, &left_count, &right_count, 0, 0, 0.0f, 0.0f);
    assert(controller.output.left_correction == 0.0f);
    assert(controller.output.right_correction == 0.0f);
}

static void test_live_p_only_config_remains_bounded(void)
{
    wheel_speed_pi_t controller;
    int64_t left_count = 0;
    int64_t right_count = 0;

    assert(wheel_speed_pi_init(&controller, &k_live_config));
    wheel_speed_pi_step(&controller, left_count, right_count, 0.4875f, 0.4875f);
    run_window(&controller, &left_count, &right_count, 0, 0, 0.4875f, 0.4875f);
    assert(controller.output.updated);
    assert(fabsf(controller.output.left_correction - 0.030f) < 0.0001f);
    assert(fabsf(controller.output.right_correction - 0.030f) < 0.0001f);
    assert(fabsf(controller.left_integral) < 0.0001f);
    assert(fabsf(controller.right_integral) < 0.0001f);
}

int main(void)
{
    test_window_and_matching_speed();
    test_independent_corrections();
    test_saturation_and_zero_reset();
    test_live_p_only_config_remains_bounded();
    puts("wheel_speed_pi host tests: PASS");
    return 0;
}
