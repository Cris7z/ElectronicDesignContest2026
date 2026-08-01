#include "task_mode.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static void test_mode_cycle(void)
{
    assert(task_mode_next(TASK_MODE_H_R02) == TASK_MODE_CONSTANT_SPEED);
    assert(task_mode_next(TASK_MODE_CONSTANT_SPEED) == TASK_MODE_H_R02);
}

static void test_distance_profiles(void)
{
    assert(fabsf(task_mode_target_distance_m(TASK_MODE_H_R02) - 7.04f) <
           0.0001f);
    assert(fabsf(task_mode_target_distance_m(TASK_MODE_CONSTANT_SPEED) -
                 7.04f) < 0.0001f);
}

static void test_timeout_profiles(void)
{
    assert(task_mode_timeout_ms(TASK_MODE_H_R02, 20000U) == 20000U);
    assert(task_mode_timeout_ms(TASK_MODE_CONSTANT_SPEED, 20000U) ==
           60000U);
}

static void test_speed_profiles(void)
{
    assert(fabsf(task_mode_speed_max_duty(TASK_MODE_H_R02,
                                          0.4875f, 0.2300f) -
                 0.4875f) < 0.0001f);
    assert(fabsf(task_mode_speed_max_duty(TASK_MODE_CONSTANT_SPEED,
                                          0.4875f, 0.2300f) -
                 0.2300f) < 0.0001f);
}

static void test_steering_profiles(void)
{
    const float expected_scale = 0.2300f / 0.4875f;

    assert(fabsf(task_mode_steering_scale(TASK_MODE_H_R02,
                                          0.4875f, 0.2300f) -
                 1.0f) < 0.0001f);
    assert(fabsf(task_mode_steering_scale(TASK_MODE_CONSTANT_SPEED,
                                          0.4875f, 0.2300f) -
                 expected_scale) < 0.0001f);
    assert(fabsf(task_mode_derivative_scale(TASK_MODE_H_R02,
                                             0.4875f, 0.2300f) -
                 1.0f) < 0.0001f);
    assert(fabsf(task_mode_derivative_scale(TASK_MODE_CONSTANT_SPEED,
                                             0.4875f, 0.2300f) -
                 (expected_scale * expected_scale)) < 0.0001f);
}

int main(void)
{
    assert(task_mode_is_valid(TASK_MODE_H_R02));
    assert(task_mode_is_valid(TASK_MODE_CONSTANT_SPEED));
    assert(!task_mode_is_valid((task_mode_t)2));
    assert(!task_mode_is_valid((task_mode_t)0));
    test_mode_cycle();
    test_distance_profiles();
    test_timeout_profiles();
    test_speed_profiles();
    test_steering_profiles();
    puts("task_mode host tests: PASS");
    return 0;
}
