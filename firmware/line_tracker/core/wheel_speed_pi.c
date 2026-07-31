#include "wheel_speed_pi.h"

#include <string.h>

#define WHEEL_SPEED_PI_UPDATE_SECONDS 0.020f

static float absf(float value)
{
    return (value < 0.0f) ? -value : value;
}

static float clampf(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

static bool config_valid(const wheel_speed_pi_config_t *config)
{
    return (config != NULL) &&
        (config->counts_per_command_window > 0.0f) &&
        (config->measurement_filter_alpha > 0.0f) &&
        (config->measurement_filter_alpha <= 1.0f) &&
        (config->kp >= 0.0f) &&
        (config->ki_per_second >= 0.0f) &&
        (config->integral_limit > 0.0f) &&
        (config->correction_limit > 0.0f) &&
        (config->enable_threshold >= 0.0f);
}

void wheel_speed_pi_reset(wheel_speed_pi_t *controller)
{
    if (controller == NULL) {
        return;
    }
    memset(&controller->output, 0, sizeof(controller->output));
    controller->left_count_sum = 0;
    controller->right_count_sum = 0;
    controller->left_target_sum = 0.0f;
    controller->right_target_sum = 0.0f;
    controller->left_measured_filtered = 0.0f;
    controller->right_measured_filtered = 0.0f;
    controller->left_integral = 0.0f;
    controller->right_integral = 0.0f;
    controller->left_target_previous = 0.0f;
    controller->right_target_previous = 0.0f;
    controller->window_ticks = 0U;
    controller->encoder_seeded = false;
    controller->measurement_seeded = false;
}

bool wheel_speed_pi_init(wheel_speed_pi_t *controller,
                         const wheel_speed_pi_config_t *config)
{
    if ((controller == NULL) || !config_valid(config)) {
        return false;
    }
    memset(controller, 0, sizeof(*controller));
    controller->config = *config;
    wheel_speed_pi_reset(controller);
    return true;
}

static float update_one_wheel(wheel_speed_pi_t *controller, float target,
                              float measured, float *integral,
                              float *previous_target)
{
    const float correction_limit = controller->config.correction_limit;
    float error;
    float integral_candidate;
    float raw_correction;
    float correction;

    if ((absf(target) < controller->config.enable_threshold) ||
        ((target * *previous_target) < 0.0f)) {
        *integral = 0.0f;
    }
    *previous_target = target;
    if (absf(target) < controller->config.enable_threshold) {
        return 0.0f;
    }

    error = target - measured;
    integral_candidate = clampf(
        *integral + (controller->config.ki_per_second * error *
                     WHEEL_SPEED_PI_UPDATE_SECONDS),
        -controller->config.integral_limit, controller->config.integral_limit);
    raw_correction = (controller->config.kp * error) + integral_candidate;
    correction = clampf(raw_correction, -correction_limit, correction_limit);

    /* Conditional integration: retain the old I term only when saturation
     * and error would push the correction farther outward. */
    if (!((raw_correction > correction_limit && error > 0.0f) ||
          (raw_correction < -correction_limit && error < 0.0f))) {
        *integral = integral_candidate;
    }
    return correction;
}

void wheel_speed_pi_step(wheel_speed_pi_t *controller,
                         int64_t left_forward_count,
                         int64_t right_forward_count,
                         float left_target,
                         float right_target)
{
    float left_measured_raw;
    float right_measured_raw;

    if (controller == NULL) {
        return;
    }
    controller->output.updated = false;
    if (!controller->encoder_seeded) {
        controller->left_count_previous = left_forward_count;
        controller->right_count_previous = right_forward_count;
        controller->encoder_seeded = true;
        return;
    }

    controller->left_count_sum += left_forward_count -
        controller->left_count_previous;
    controller->right_count_sum += right_forward_count -
        controller->right_count_previous;
    controller->left_count_previous = left_forward_count;
    controller->right_count_previous = right_forward_count;
    controller->left_target_sum += left_target;
    controller->right_target_sum += right_target;
    if (++controller->window_ticks < WHEEL_SPEED_PI_WINDOW_TICKS) {
        return;
    }

    controller->output.left_target = controller->left_target_sum /
        (float)WHEEL_SPEED_PI_WINDOW_TICKS;
    controller->output.right_target = controller->right_target_sum /
        (float)WHEEL_SPEED_PI_WINDOW_TICKS;
    left_measured_raw = (float)controller->left_count_sum /
        controller->config.counts_per_command_window;
    right_measured_raw = (float)controller->right_count_sum /
        controller->config.counts_per_command_window;
    if (!controller->measurement_seeded) {
        controller->left_measured_filtered = left_measured_raw;
        controller->right_measured_filtered = right_measured_raw;
        controller->measurement_seeded = true;
    } else {
        controller->left_measured_filtered +=
            controller->config.measurement_filter_alpha *
            (left_measured_raw - controller->left_measured_filtered);
        controller->right_measured_filtered +=
            controller->config.measurement_filter_alpha *
            (right_measured_raw - controller->right_measured_filtered);
    }
    controller->output.left_measured = controller->left_measured_filtered;
    controller->output.right_measured = controller->right_measured_filtered;
    controller->output.left_correction = update_one_wheel(
        controller, controller->output.left_target,
        controller->output.left_measured, &controller->left_integral,
        &controller->left_target_previous);
    controller->output.right_correction = update_one_wheel(
        controller, controller->output.right_target,
        controller->output.right_measured, &controller->right_integral,
        &controller->right_target_previous);
    controller->left_count_sum = 0;
    controller->right_count_sum = 0;
    controller->left_target_sum = 0.0f;
    controller->right_target_sum = 0.0f;
    controller->window_ticks = 0U;
    controller->output.updated = true;
}
