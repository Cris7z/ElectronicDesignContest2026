#include "chassis.h"

void chassis_init(chassis_t *c, float counts_per_rev, float wheel_d_m,
                  float track_width_m)
{
    /* 速度环默认参数: 输出为 PWM 占空比 [-1,1]
     * kp/ki 与电机、电压强相关, 必须自己阶跃整定! 这里给的是常见量级 */
    pid_init(&c->pid_l, 0.8f, 0.12f, 0.0f, -1.0f, 1.0f);
    pid_init(&c->pid_r, 0.8f, 0.12f, 0.0f, -1.0f, 1.0f);
    c->pid_l.kf = 0.55f;   /* 前馈: 大致 = 满速时PWM / 满速m/s, 先估后调 */
    c->pid_r.kf = 0.55f;

    encoder_init(&c->enc_l, counts_per_rev, wheel_d_m);
    encoder_init(&c->enc_r, counts_per_rev, wheel_d_m);

    c->v_target = c->v_ramped = c->steer = 0.0f;
    c->v_ramp_step = 0.02f;      /* 每5ms最多变0.02m/s => 4m/s^2 */
    c->track_width_m = track_width_m;
    c->out_l = c->out_r = 0.0f;
    c->slow_k = 0.8f;
    c->enabled = 0;
}

void chassis_set(chassis_t *c, float v_mps, float steer_mps)
{
    c->v_target = v_mps;
    c->steer = steer_mps;
}

void chassis_enable(chassis_t *c, int en)
{
    c->enabled = en;
    if (!en) {
        pid_reset(&c->pid_l); pid_reset(&c->pid_r);
        c->pid_l.kf = c->pid_r.kf;   /* 保留参数 */
        c->v_ramped = 0.0f;
        c->out_l = c->out_r = 0.0f;
    }
}

void chassis_update(chassis_t *c, int32_t cnt_l, int32_t cnt_r)
{
    /* BSP 的 5ms 中断可能先于 chassis_init 触发(见 bsp_init 尾注),
     * 未初始化时 counts_per_rev=0 会除零产生 NaN, 这里直接跳过 */
    if (c->enc_l.counts_per_rev <= 0.0f) { c->out_l = c->out_r = 0.0f; return; }

    encoder_update(&c->enc_l, cnt_l, CTRL_DT);
    encoder_update(&c->enc_r, cnt_r, CTRL_DT);

    if (!c->enabled) { c->out_l = c->out_r = 0.0f; return; }

    /* 弯道自适应降速: 转向指令越大, 目标线速度越低 */
    float steer_norm = c->steer / (c->v_target > 0.1f ? c->v_target : 0.1f);
    if (steer_norm < 0) steer_norm = -steer_norm;
    float v_want = c->v_target / (1.0f + c->slow_k * steer_norm);

    /* 斜坡 */
    float dv = v_want - c->v_ramped;
    c->v_ramped += clampf(dv, -c->v_ramp_step, c->v_ramp_step);

    float tl = c->v_ramped - 0.5f * c->steer;
    float tr = c->v_ramped + 0.5f * c->steer;

    c->out_l = pid_inc(&c->pid_l, tl, c->enc_l.speed_mps);
    c->out_r = pid_inc(&c->pid_r, tr, c->enc_r.speed_mps);
}

float chassis_speed(const chassis_t *c)
{
    return 0.5f * (c->enc_l.speed_mps + c->enc_r.speed_mps);
}

float chassis_omega(const chassis_t *c)
{
    return (c->enc_r.speed_mps - c->enc_l.speed_mps) / c->track_width_m;
}
