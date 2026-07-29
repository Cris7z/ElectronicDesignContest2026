/**
 * chassis.h — 差速底盘串级控制
 *  内环: 左右轮各一个增量式 PI 速度环(+速度前馈), 周期 = CTRL_DT(5ms)
 *  外环输入: 目标线速度 v (m/s) + 目标角速度差 steer(左右轮速差 m/s)
 *  附加: 目标速度斜坡(防打滑/翘头), 弯道自适应降速
 */
#ifndef CHASSIS_H
#define CHASSIS_H

#include "pid.h"
#include "../sensors/encoder.h"

#define CTRL_DT 0.005f   /* 速度环周期 5ms */

typedef struct {
    pid_t pid_l, pid_r;      /* 速度环 */
    encoder_t enc_l, enc_r;

    float v_target;          /* 外部设定线速度 m/s */
    float v_ramped;          /* 斜坡后的线速度 */
    float v_ramp_step;       /* 每周期最大加减速 (m/s)/tick */
    float steer;             /* 轮速差指令 m/s (左转/逆时针为正: 右快左慢) */

    float track_width_m;     /* 轮距(米), 里程计用 */
    float out_l, out_r;      /* PWM 输出 [-1,1] */
    float slow_k;            /* 弯道降速系数: v = v/(1+slow_k*|steer_norm|) */
    int   enabled;
} chassis_t;

void chassis_init(chassis_t *c, float counts_per_rev, float wheel_d_m,
                  float track_width_m);
void chassis_set(chassis_t *c, float v_mps, float steer_mps);
void chassis_enable(chassis_t *c, int en);
/* 5ms 调用一次; cnt_l/cnt_r 为编码器累计计数(左右都以前进为正!) */
void chassis_update(chassis_t *c, int32_t cnt_l, int32_t cnt_r);

/* 便捷: 车体线速度/角速度(rad/s, 由轮速差算, 陀螺仪不可用时的备份) */
float chassis_speed(const chassis_t *c);
float chassis_omega(const chassis_t *c);

#endif
