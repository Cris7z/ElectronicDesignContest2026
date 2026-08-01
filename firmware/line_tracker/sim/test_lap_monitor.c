#include "lap_monitor.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static const lap_monitor_config_t k_config = {
    .target_distance_m = 1.0f,
    .approach_distance_m = 0.15f,
    .left_meters_per_count = 0.01f,
    .right_meters_per_count = 0.01f,
    .left_forward_sign = -1,
    .right_forward_sign = 1,
    .timeout_ms = 20000U
};

static void test_forward_distance_and_approach(void)
{
    lap_monitor_t monitor;

    assert(lap_monitor_init(&monitor, &k_config));
    lap_monitor_start(&monitor, 1000, -2000, 50U);
    lap_monitor_step(&monitor, 915, -1915, 500U);
    assert(monitor.output.state == LAP_MONITOR_RUNNING);
    assert(fabsf(monitor.output.distance_m - 0.85f) < 0.0001f);
    assert(monitor.output.approach_active);
    assert(monitor.output.elapsed_ms == 450U);
}

static void test_complete_at_target(void)
{
    lap_monitor_t monitor;

    assert(lap_monitor_init(&monitor, &k_config));
    lap_monitor_start(&monitor, 0, 0, 0U);
    lap_monitor_step(&monitor, -100, 100, 1500U);
    assert(monitor.output.state == LAP_MONITOR_COMPLETE);
    assert(fabsf(monitor.output.distance_m - 1.0f) < 0.0001f);
    assert(monitor.output.remaining_m == 0.0f);
}

static void test_timeout_precedes_late_completion(void)
{
    lap_monitor_t monitor;

    assert(lap_monitor_init(&monitor, &k_config));
    lap_monitor_start(&monitor, 0, 0, 0U);
    lap_monitor_step(&monitor, -100, 100, 20000U);
    assert(monitor.output.state == LAP_MONITOR_TIMEOUT);
}

static void test_cancel_acknowledges_terminal_state(void)
{
    lap_monitor_t monitor;

    assert(lap_monitor_init(&monitor, &k_config));
    lap_monitor_start(&monitor, 0, 0, 0U);
    lap_monitor_step(&monitor, -100, 100, 1500U);
    assert(monitor.output.state == LAP_MONITOR_COMPLETE);
    lap_monitor_cancel(&monitor);
    assert(monitor.output.state == LAP_MONITOR_WAIT);
    assert(fabsf(monitor.output.distance_m - 1.0f) < 0.0001f);

    lap_monitor_start(&monitor, 0, 0, 0U);
    lap_monitor_step(&monitor, 0, 0, 20000U);
    assert(monitor.output.state == LAP_MONITOR_TIMEOUT);
    lap_monitor_cancel(&monitor);
    assert(monitor.output.state == LAP_MONITOR_WAIT);
}

static void test_invalid_config_rejected(void)
{
    lap_monitor_t monitor;
    lap_monitor_config_t invalid = k_config;

    invalid.left_forward_sign = 0;
    assert(!lap_monitor_init(&monitor, &invalid));
}

int main(void)
{
    test_forward_distance_and_approach();
    test_complete_at_target();
    test_timeout_precedes_late_completion();
    test_cancel_acknowledges_terminal_state();
    test_invalid_config_rejected();
    puts("lap_monitor host tests: PASS");
    return 0;
}
