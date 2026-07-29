#ifndef H2026_Q2_APP_CONFIG_H
#define H2026_Q2_APP_CONFIG_H

#include "../core/h2026_q2.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 实车标定区
 * ----------
 * 默认值故意保持“未标定”，因此当前固件可以编译、读传感器和显示诊断，但绝不
 * 会给 TB6612 输出电机命令。完成 README.md 的架空验收后逐项填写，最后才把
 * H2026_Q2_COMMISSIONED 改成 1。
 */
#define H2026_Q2_COMMISSIONED                 0

/* 只能填 0 或 1；HiWonder 寄存器 5 的目标有效电平和 bit0 左右顺序均待实测。 */
#define H2026_Q2_SENSOR_ACTIVE_HIGH          (-1)
#define H2026_Q2_SENSOR_BIT0_IS_LEFT         (-1)

/* 只能填 -1 或 +1；“正”必须统一为小车物理前进。 */
#define H2026_Q2_LEFT_ENCODER_SIGN            (-1)
#define H2026_Q2_RIGHT_ENCODER_SIGN             1
#define H2026_Q2_LEFT_MOTOR_SIGN                1
#define H2026_Q2_RIGHT_MOTOR_SIGN               1

/* 四边沿软件正交计数对应的单个 count 行驶距离，单位 m/count。左右分别标定。 */
#define H2026_Q2_LEFT_METERS_PER_ENCODER_COUNT  0.0f
#define H2026_Q2_RIGHT_METERS_PER_ENCODER_COUNT 0.0f

/* 左右轮接地点中心距，单位 m。 */
#define H2026_Q2_TRACK_WIDTH_M               0.0f

/*
 * 车头传感器确认终点横线后，小车还要前进多少距离，才能让规定的车体参考点回到
 * A 点。必须在实车上卷尺标定，不能用固定延时替代。
 */
#define H2026_Q2_STOP_DISTANCE_FROM_MARKER_M 0.0f

/* PA18 按下后的原始电平：只能填 0 或 1。 */
#define H2026_Q2_START_ACTIVE_LEVEL          (-1)

/*
 * 仅当 A 点确实位于“1.5 m 直线入口”，并用里程日志核对两个切弯点后置 1。
 * 未核对时保持 0，循迹 PD 仍可独立完成整圈。
 */
#define H2026_Q2_STADIUM_FEEDFORWARD_VERIFIED 0

enum {
    H2026_Q2_CAL_LOCK_NOT_COMMISSIONED = 1UL << 0,
    H2026_Q2_CAL_LOCK_LINE_POLARITY    = 1UL << 1,
    H2026_Q2_CAL_LOCK_LINE_ORDER       = 1UL << 2,
    H2026_Q2_CAL_LOCK_ENCODER_SIGNS    = 1UL << 3,
    H2026_Q2_CAL_LOCK_MOTOR_SIGNS      = 1UL << 4,
    H2026_Q2_CAL_LOCK_ENCODER_SCALE    = 1UL << 5,
    H2026_Q2_CAL_LOCK_TRACK_WIDTH      = 1UL << 6,
    H2026_Q2_CAL_LOCK_STOP_OFFSET      = 1UL << 7,
    H2026_Q2_CAL_LOCK_BUTTON_LEVEL     = 1UL << 8
};

uint32_t h2026_q2_app_calibration_locks(void);
bool h2026_q2_app_build_config(h2026_q2_config_t *config);
bool h2026_q2_app_start_pressed(bool raw_level);
float h2026_q2_app_left_motor_duty(float controller_duty);
float h2026_q2_app_right_motor_duty(float controller_duty);

#endif
