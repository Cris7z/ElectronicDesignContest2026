#ifndef H2026_Q2_APP_CONFIG_H
#define H2026_Q2_APP_CONFIG_H

#include "../core/h2026_q2.h"

#include <stdbool.h>
#include <stdint.h>

/*
 * 实车标定区
 * ----------
 * 此处保存 2026-07-30 已实测的首轮参数。H2026_Q2_COMMISSIONED=1 只解除
 * 参数门，不代表赛道整圈验收已完成；换线、换电机或改机械后必须重新标定并锁回 0。
 */
#define H2026_Q2_COMMISSIONED                 1

/* CD4051 ADC 标定的实际横坐标必须按 CH1..CH8 左到右填写，单位 mm。 */
#define H2026_Q2_SENSOR_X0_MM               (-35.0f)
#define H2026_Q2_SENSOR_X1_MM               (-25.0f)
#define H2026_Q2_SENSOR_X2_MM               (-15.0f)
#define H2026_Q2_SENSOR_X3_MM                (-5.0f)
#define H2026_Q2_SENSOR_X4_MM                 5.0f
#define H2026_Q2_SENSOR_X5_MM                15.0f
#define H2026_Q2_SENSOR_X6_MM                25.0f
#define H2026_Q2_SENSOR_X7_MM                35.0f

/* CH1/R0 经逐路遮挡确认：行驶方向朝前时，它位于车体左侧。 */
#define H2026_Q2_SENSOR_CH1_IS_LEFT            1

/*
 * 只能填 -1 或 +1；“正”必须统一为小车物理前进。
 * 架空主动实测（10 %、2 s）：左轮仅左通道 -908、右轮仅右通道 +1361，
 * 交叉计数均为 0；故左乘 -1、右乘 +1 后，物理前进均为正。
 */
#define H2026_Q2_LEFT_ENCODER_SIGN            (-1)
#define H2026_Q2_RIGHT_ENCODER_SIGN             1
#define H2026_Q2_LEFT_MOTOR_SIGN                1
#define H2026_Q2_RIGHT_MOTOR_SIGN               1

/* 四边沿软件正交计数对应的单个 count 行驶距离，单位 m/count。左右分别标定。 */
/* Ground run: 0.700 m / (L=4483, R=4217), INVALID=0/0. */
#define H2026_Q2_LEFT_METERS_PER_ENCODER_COUNT  0.0001561454f
#define H2026_Q2_RIGHT_METERS_PER_ENCODER_COUNT 0.0001659948f

/* 机械图纸：轮外宽 214.2 mm、轮胎轴向宽 14.5 mm，轮心距初值 199.7 mm。 */
#define H2026_Q2_TRACK_WIDTH_M               0.1997f

/*
 * 本车以轮子压在起点为零点，终点完全按一圈里程闭环；灰度横线不参与启停。
 * 此旧字段仅供“横线模式”保留，当前不会使用。
 */
#define H2026_Q2_STOP_DISTANCE_FROM_MARKER_M 0.0f

/* C07A SW3/BLS：R8=47 kOhm 外部下拉，实测按下 PA18 为高电平。 */
#define H2026_Q2_START_ACTIVE_LEVEL          1

/*
 * 仅当 A 点确实位于“1.5 m 直线入口”，并用里程日志核对两个切弯点后置 1。
 * 未核对时保持 0，循迹 PD 仍可独立完成整圈。
 */
#define H2026_Q2_STADIUM_FEEDFORWARD_VERIFIED 0

enum {
    H2026_Q2_CAL_LOCK_NOT_COMMISSIONED = 1UL << 0,
    H2026_Q2_CAL_LOCK_LINE_CALIBRATION = 1UL << 1,
    H2026_Q2_CAL_LOCK_LINE_ORDER       = 1UL << 2,
    H2026_Q2_CAL_LOCK_ENCODER_SIGNS    = 1UL << 3,
    H2026_Q2_CAL_LOCK_MOTOR_SIGNS      = 1UL << 4,
    H2026_Q2_CAL_LOCK_ENCODER_SCALE    = 1UL << 5,
    H2026_Q2_CAL_LOCK_TRACK_WIDTH      = 1UL << 6,
    H2026_Q2_CAL_LOCK_STOP_OFFSET      = 1UL << 7,
    H2026_Q2_CAL_LOCK_BUTTON_LEVEL     = 1UL << 8
};

uint32_t h2026_q2_app_calibration_locks(void);
bool h2026_q2_app_line_calibration_get(
    h2026_q2_line_calibration_t *calibration);
bool h2026_q2_app_line_calibration_set(
    const h2026_q2_line_calibration_t *calibration);
/* 0 means the last store passed; nonzero identifies the failed store step. */
uint8_t h2026_q2_app_line_calibration_store_error(void);
bool h2026_q2_app_build_config(h2026_q2_config_t *config);
bool h2026_q2_app_start_pressed(bool raw_level);
float h2026_q2_app_left_motor_duty(float controller_duty);
float h2026_q2_app_right_motor_duty(float controller_duty);

#endif
