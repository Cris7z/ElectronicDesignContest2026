#include "ball_balance.h"

#define RAD_TO_DEG 57.2957795131f

static float clampf_local(float value, float low, float high)
{
    return value < low ? low : (value > high ? high : value);
}
static float slew(float current, float target, float step)
{
    if (target > current + step) {
        return current + step;
    }
    if (target < current - step) {
        return current - step;
    }
    return target;
}

void ball_balance_default_params(ball_balance_params_t *params)
{
    params->kp_rad_per_m = 0.75f;
    params->ki_rad_per_m_s = 0.0f;
    params->kd_rad_s_per_m = 0.45f;
    params->accel_feedforward_gain = 1.0f / 9.80665f;
    params->velocity_lpf_alpha = 0.30f;
    params->integral_limit_m_s = 0.03f;
    params->max_angle_deg = 4.0f;
    params->max_slew_deg_s = 80.0f;
}

void ball_balance_init(ball_balance_t *ctrl,
                       const ball_balance_params_t *params)
{
    ctrl->p = *params;
    ball_balance_reset(ctrl);
}

void ball_balance_reset(ball_balance_t *ctrl)
{
    ctrl->integral_m_s = 0.0f;
    ctrl->velocity_m_s = 0.0f;
    ctrl->command_deg = 0.0f;
    ctrl->measurement_valid = false;
}

float ball_balance_update(ball_balance_t *ctrl,
                          float target_mm,
                          float measured_mm,
                          float measured_v_mm_s,
                          float vehicle_accel_m_s2,
                          float dt_s,
                          bool valid)
{
    dt_s = clampf_local(dt_s, 0.001f, 0.100f);
    const float max_step = ctrl->p.max_slew_deg_s * dt_s;

    if (!valid) {
        ctrl->integral_m_s = 0.0f;
        ctrl->measurement_valid = false;
        ctrl->command_deg = slew(ctrl->command_deg, 0.0f, max_step);
        return ctrl->command_deg;
    }

    const float target_m = target_mm * 0.001f;
    const float measured_m = measured_mm * 0.001f;
    const float raw_velocity_m_s = measured_v_mm_s * 0.001f;
    if (!ctrl->measurement_valid) {
        ctrl->velocity_m_s = raw_velocity_m_s;
        ctrl->measurement_valid = true;
    } else {
        const float a = clampf_local(ctrl->p.velocity_lpf_alpha, 0.0f, 1.0f);
        ctrl->velocity_m_s += a * (raw_velocity_m_s - ctrl->velocity_m_s);
    }

    const float error_m = target_m - measured_m;
    ctrl->integral_m_s += error_m * dt_s;
    ctrl->integral_m_s = clampf_local(ctrl->integral_m_s,
                                      -ctrl->p.integral_limit_m_s,
                                      ctrl->p.integral_limit_m_s);

    const float command_rad =
        ctrl->p.kp_rad_per_m * error_m +
        ctrl->p.ki_rad_per_m_s * ctrl->integral_m_s -
        ctrl->p.kd_rad_s_per_m * ctrl->velocity_m_s +
        ctrl->p.accel_feedforward_gain * vehicle_accel_m_s2;

    float target_deg = command_rad * RAD_TO_DEG;
    target_deg = clampf_local(target_deg,
                              -ctrl->p.max_angle_deg,
                              ctrl->p.max_angle_deg);

    if ((target_deg >= ctrl->p.max_angle_deg && error_m > 0.0f) ||
        (target_deg <= -ctrl->p.max_angle_deg && error_m < 0.0f)) {
        ctrl->integral_m_s -= error_m * dt_s;
    }

    ctrl->command_deg = slew(ctrl->command_deg, target_deg, max_step);
    return ctrl->command_deg;
}

float ball_balance_smooth_target(float start_mm, float end_mm,
                                 float elapsed_s, float duration_s)
{
    if (duration_s <= 0.0f || elapsed_s >= duration_s) {
        return end_mm;
    }
    if (elapsed_s <= 0.0f) {
        return start_mm;
    }
    const float u = elapsed_s / duration_s;
    const float s = u * u * u * (10.0f + u * (-15.0f + 6.0f * u));
    return start_mm + (end_mm - start_mm) * s;
}
