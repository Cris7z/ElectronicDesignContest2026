#include "task_mode.h"

#define H_R02_DISTANCE_M 7.0600f
#define CONSTANT_SPEED_DISTANCE_M 7.0400f
#define CONSTANT_SPEED_TIMEOUT_MS 60000U

bool task_mode_is_valid(task_mode_t mode)
{
    return (mode == TASK_MODE_H_R02) ||
           (mode == TASK_MODE_CONSTANT_SPEED);
}

task_mode_t task_mode_next(task_mode_t mode)
{
    switch (mode) {
    case TASK_MODE_H_R02:
        return TASK_MODE_CONSTANT_SPEED;
    case TASK_MODE_CONSTANT_SPEED:
    default:
        return TASK_MODE_H_R02;
    }
}

float task_mode_target_distance_m(task_mode_t mode)
{
    if (mode == TASK_MODE_CONSTANT_SPEED) {
        return CONSTANT_SPEED_DISTANCE_M;
    }
    return H_R02_DISTANCE_M;
}

uint32_t task_mode_timeout_ms(task_mode_t mode,
                              uint32_t frozen_timeout_ms)
{
    if (mode == TASK_MODE_CONSTANT_SPEED) {
        return CONSTANT_SPEED_TIMEOUT_MS;
    }
    return frozen_timeout_ms;
}

float task_mode_speed_max_duty(task_mode_t mode,
                               float frozen_max_duty,
                               float frozen_min_duty)
{
    if (mode == TASK_MODE_CONSTANT_SPEED) {
        return frozen_min_duty;
    }
    return frozen_max_duty;
}

float task_mode_steering_scale(task_mode_t mode,
                               float frozen_max_duty,
                               float frozen_min_duty)
{
    if ((mode == TASK_MODE_CONSTANT_SPEED) &&
        (frozen_max_duty > 0.0f) &&
        (frozen_min_duty > 0.0f)) {
        return frozen_min_duty / frozen_max_duty;
    }
    return 1.0f;
}

float task_mode_derivative_scale(task_mode_t mode,
                                 float frozen_max_duty,
                                 float frozen_min_duty)
{
    if (mode == TASK_MODE_CONSTANT_SPEED) {
        float low_noise_scale;

        if ((frozen_max_duty <= 0.0f) ||
            (frozen_min_duty <= 0.0f)) {
            return 1.0f;
        }
        low_noise_scale = frozen_min_duty / frozen_max_duty;
        /* A one-frame centroid jump is speed-independent sensor noise.  Give
         * the task-specific M3 profile the proven lower D gain. */
        return low_noise_scale * low_noise_scale;
    }
    return 1.0f;
}
