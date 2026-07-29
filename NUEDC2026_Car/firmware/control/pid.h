/**
 * pid.h — 通用 PID 模块 (位置式 + 增量式)
 * 特性: 微分先行(对测量值微分,避免设定值突变踢腿)、微分一阶低通、
 *       积分限幅 + 反饱和(back-calculation)、输出限幅、输出斜坡限制。
 */
#ifndef PID_H
#define PID_H

#include <stdint.h>

typedef struct {
    /* 参数(可被菜单/上位机在线修改) */
    float kp, ki, kd;
    float kf;            /* 前馈系数: out += kf * target (速度环用) */
    float ks;            /* 静摩擦/死区前馈: out += ks*sgn(target), 治低速爬行 */
    float out_min, out_max;
    float i_min, i_max;  /* 积分限幅 */
    float d_lpf_alpha;   /* 微分低通 0~1, 越小越平滑, 建议0.2~0.5 */
    float out_slew;      /* 每周期输出最大变化量, 0=不限制 */

    /* 状态 */
    float integ;
    float prev_meas;     /* 上次测量(微分先行用) */
    float prev_out;
    float d_filt;
    uint8_t first;
} pid_t;

void  pid_init(pid_t *p, float kp, float ki, float kd,
               float out_min, float out_max);
void  pid_reset(pid_t *p);
/* 位置式: 转向环/航向环/距离环用。err = target - meas 由调用者算好传入,
 * meas 单独传入用于微分先行 */
float pid_pos(pid_t *p, float err, float meas);
/* 增量式 PI(+前馈): 速度环用, 返回累加后的输出(内部保存) */
float pid_inc(pid_t *p, float target, float meas);

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

#endif
