#ifndef TASK_MODE_H
#define TASK_MODE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    TASK_MODE_H_R02 = 1,
    TASK_MODE_CONSTANT_SPEED = 3
} task_mode_t;

bool task_mode_is_valid(task_mode_t mode);
task_mode_t task_mode_next(task_mode_t mode);
float task_mode_target_distance_m(task_mode_t mode);
uint32_t task_mode_timeout_ms(task_mode_t mode,
                              uint32_t frozen_timeout_ms);
float task_mode_speed_max_duty(task_mode_t mode,
                               float frozen_max_duty,
                               float frozen_min_duty);
float task_mode_steering_scale(task_mode_t mode,
                               float frozen_max_duty,
                               float frozen_min_duty);
float task_mode_derivative_scale(task_mode_t mode,
                                 float frozen_max_duty,
                                 float frozen_min_duty);

#endif
