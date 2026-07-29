#include "h2026_q2_app_config.h"

#include <string.h>

#define H2026_Q2_PI 3.14159265358979323846f

static bool is_binary_setting(int value)
{
    return (value == 0) || (value == 1);
}

static bool is_sign_setting(int value)
{
    return (value == -1) || (value == 1);
}

uint32_t h2026_q2_app_calibration_locks(void)
{
    uint32_t locks = 0U;

    if (H2026_Q2_COMMISSIONED != 1) {
        locks |= H2026_Q2_CAL_LOCK_NOT_COMMISSIONED;
    }
    if (!is_binary_setting(H2026_Q2_SENSOR_ACTIVE_HIGH)) {
        locks |= H2026_Q2_CAL_LOCK_LINE_POLARITY;
    }
    if (!is_binary_setting(H2026_Q2_SENSOR_BIT0_IS_LEFT)) {
        locks |= H2026_Q2_CAL_LOCK_LINE_ORDER;
    }
    if (!is_sign_setting(H2026_Q2_LEFT_ENCODER_SIGN) ||
        !is_sign_setting(H2026_Q2_RIGHT_ENCODER_SIGN)) {
        locks |= H2026_Q2_CAL_LOCK_ENCODER_SIGNS;
    }
    if (!is_sign_setting(H2026_Q2_LEFT_MOTOR_SIGN) ||
        !is_sign_setting(H2026_Q2_RIGHT_MOTOR_SIGN)) {
        locks |= H2026_Q2_CAL_LOCK_MOTOR_SIGNS;
    }
    if (!(H2026_Q2_LEFT_METERS_PER_ENCODER_COUNT > 0.0f) ||
        !(H2026_Q2_RIGHT_METERS_PER_ENCODER_COUNT > 0.0f)) {
        locks |= H2026_Q2_CAL_LOCK_ENCODER_SCALE;
    }
    if (!(H2026_Q2_TRACK_WIDTH_M > 0.0f)) {
        locks |= H2026_Q2_CAL_LOCK_TRACK_WIDTH;
    }
    if (!(H2026_Q2_STOP_DISTANCE_FROM_MARKER_M > 0.0f)) {
        locks |= H2026_Q2_CAL_LOCK_STOP_OFFSET;
    }
    if (!is_binary_setting(H2026_Q2_START_ACTIVE_LEVEL)) {
        locks |= H2026_Q2_CAL_LOCK_BUTTON_LEVEL;
    }
    return locks;
}

static void configure_stadium_feedforward(h2026_q2_config_t *config)
{
    const float straight_m = 1.5f;
    const float half_circle_m = H2026_Q2_PI * 0.5f;
    const float clockwise_curvature_1pm = 2.0f;
    const float lap_m = (2.0f * straight_m) + (2.0f * half_circle_m);

    if (H2026_Q2_STADIUM_FEEDFORWARD_VERIFIED != 1) {
        config->curve_segment_count = 0U;
        return;
    }

    /*
     * 正曲率在 core 中定义为右转。末尾额外放一个 0 曲率段，使过终点后的
     * 停车距离不会继续套用最后一个半圆的前馈。
     */
    config->curve_segment_count = 5U;
    config->curve_segments[0].end_distance_m = straight_m;
    config->curve_segments[0].curvature_1pm = 0.0f;
    config->curve_segments[1].end_distance_m = straight_m + half_circle_m;
    config->curve_segments[1].curvature_1pm = clockwise_curvature_1pm;
    config->curve_segments[2].end_distance_m =
        (2.0f * straight_m) + half_circle_m;
    config->curve_segments[2].curvature_1pm = 0.0f;
    config->curve_segments[3].end_distance_m = lap_m;
    config->curve_segments[3].curvature_1pm = clockwise_curvature_1pm;
    config->curve_segments[4].end_distance_m = lap_m + 1.0f;
    config->curve_segments[4].curvature_1pm = 0.0f;
}

bool h2026_q2_app_build_config(h2026_q2_config_t *config)
{
    if (config == NULL) {
        return false;
    }

    memset(config, 0, sizeof(*config));

    config->sensor_active_high =
        H2026_Q2_SENSOR_ACTIVE_HIGH == 1;
    config->sensor_bit0_is_left =
        H2026_Q2_SENSOR_BIT0_IS_LEFT == 1;
    config->wide_min_active = 3U;

    /*
     * 5 cm 横向起止线一般只覆盖八路板的一部分，绝不能写死 0xFF。
     * 按键起跑时采集实际图案，返程只接受足够宽且与该图案相交的候选。
     */
    config->marker_capture_min_active = 3U;
    config->marker_detect_min_active = 3U;
    config->marker_count_tolerance = 1U;
    config->marker_detect_ratio = 0.70f;
    config->marker_center_limit_normalized = 0.45f;
    config->marker_release_ms = 60U;
    config->marker_confirm_ms = 25U;
    config->start_clear_distance_m = 0.12f;
    config->finish_gate_distance_m = 5.50f;
    config->finish_gate_time_ms = 11000U;
    config->stop_distance_from_marker_m =
        H2026_Q2_STOP_DISTANCE_FROM_MARKER_M;
    config->stop_position_tolerance_m = 0.008f;
    config->stop_speed_tolerance_mps = 0.015f;
    config->stop_hold_ms = 250U;

    config->i2c_grace_ms = 20U;
    config->i2c_fault_ms = 150U;
    config->line_grace_ms = 40U;
    config->line_fault_ms = 300U;
    config->mission_timeout_ms = 25000U;
    config->stopping_timeout_ms = 4000U;
    config->fault_coast_max_ms = 150U;

    config->left_meters_per_encoder_count =
        H2026_Q2_LEFT_METERS_PER_ENCODER_COUNT;
    config->right_meters_per_encoder_count =
        H2026_Q2_RIGHT_METERS_PER_ENCODER_COUNT;
    config->left_encoder_sign =
        (int8_t)H2026_Q2_LEFT_ENCODER_SIGN;
    config->right_encoder_sign =
        (int8_t)H2026_Q2_RIGHT_ENCODER_SIGN;
    config->track_width_m = H2026_Q2_TRACK_WIDTH_M;

    /* 6.142 m 一圈：直线约 0.40 m/s，弯道目标约 0.35 m/s。 */
    config->cruise_speed_mps = 0.40f;
    config->minimum_tracking_speed_mps = 0.18f;
    config->degraded_i2c_speed_mps = 0.14f;
    config->degraded_line_speed_mps = 0.12f;
    config->maximum_wheel_speed_mps = 0.60f;
    config->acceleration_limit_mps2 = 0.70f;
    config->deceleration_limit_mps2 = 1.20f;
    config->stopping_deceleration_mps2 = 0.80f;
    config->line_error_speed_reduction = 0.15f;
    config->curvature_speed_reduction_m = 0.0625f;

    config->line_error_filter_alpha = 0.35f;
    config->line_derivative_filter_alpha = 0.20f;
    config->line_kp_center_mps = 0.14f;
    config->line_kp_edge_mps = 0.30f;
    config->line_kd_center_m = 0.005f;
    config->line_kd_edge_m = 0.012f;
    config->line_correction_limit_mps = 0.23f;

    config->curvature_feedforward_gain = 0.90f;
    config->curve_transition_m = 0.12f;
    configure_stadium_feedforward(config);

    /*
     * 这里只是台架辨识前的保守起调值。kS/kV 和 PI 必须用架空轮速阶跃日志
     * 重辨识；H2026_Q2_COMMISSIONED 门不会允许这些值带着未知机械量直接动车。
     */
    config->left_speed_pi.kp = 0.60f;
    config->left_speed_pi.ki = 3.00f;
    config->left_speed_pi.ks = 0.08f;
    config->left_speed_pi.kv = 1.80f;
    config->left_speed_pi.integral_limit = 0.25f;
    config->right_speed_pi = config->left_speed_pi;
    config->wheel_speed_filter_alpha = 0.35f;
    config->signed_duty_limit = 0.85f;

    return (h2026_q2_app_calibration_locks() == 0U) &&
           h2026_q2_config_validate(config);
}

bool h2026_q2_app_start_pressed(bool raw_level)
{
    if (!is_binary_setting(H2026_Q2_START_ACTIVE_LEVEL)) {
        return false;
    }
    return raw_level ==
           (H2026_Q2_START_ACTIVE_LEVEL == 1);
}

float h2026_q2_app_left_motor_duty(float controller_duty)
{
    return controller_duty * (float)H2026_Q2_LEFT_MOTOR_SIGN;
}

float h2026_q2_app_right_motor_duty(float controller_duty)
{
    return controller_duty * (float)H2026_Q2_RIGHT_MOTOR_SIGN;
}
