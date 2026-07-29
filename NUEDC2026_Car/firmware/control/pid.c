#include "pid.h"

void pid_init(pid_t *p, float kp, float ki, float kd,
              float out_min, float out_max)
{
    p->kp = kp; p->ki = ki; p->kd = kd; p->kf = 0.0f; p->ks = 0.0f;
    p->out_min = out_min; p->out_max = out_max;
    p->i_min = out_min; p->i_max = out_max;
    p->d_lpf_alpha = 0.3f;
    p->out_slew = 0.0f;
    pid_reset(p);
}

void pid_reset(pid_t *p)
{
    p->integ = 0.0f; p->prev_meas = 0.0f;
    p->prev_out = 0.0f; p->d_filt = 0.0f; p->first = 1;
}

float pid_pos(pid_t *p, float err, float meas)
{
    if (p->first) { p->prev_meas = meas; p->first = 0; }

    p->integ += p->ki * err;
    p->integ = clampf(p->integ, p->i_min, p->i_max);

    /* 微分先行: 对测量值微分并取负, 目标突变时 D 项不冲击 */
    float d_raw = -(meas - p->prev_meas);
    p->prev_meas = meas;
    p->d_filt += p->d_lpf_alpha * (d_raw - p->d_filt);

    float out = p->kp * err + p->integ + p->kd * p->d_filt;
    float sat = clampf(out, p->out_min, p->out_max);

    /* back-calculation 反饱和: 输出饱和时往回吐积分 */
    if (p->ki > 1e-9f && sat != out)
        p->integ += 0.5f * (sat - out);

    if (p->out_slew > 0.0f)
        sat = clampf(sat, p->prev_out - p->out_slew, p->prev_out + p->out_slew);
    p->prev_out = sat;
    return sat;
}

float pid_inc(pid_t *p, float target, float meas)
{
    float err = target - meas;
    if (p->first) { p->prev_meas = err; p->d_filt = err; p->first = 0; }
    /* 增量式: Δu = Kp*(e[k]-e[k-1]) + Ki*e[k]  (D 项一般速度环不用)
     * p->prev_meas 复用为 e[k-1], p->d_filt 复用为 e[k-2] */
    float delta = p->kp * (err - p->prev_meas) + p->ki * err
                + p->kd * (err - 2.0f * p->prev_meas + p->d_filt);
    p->d_filt = p->prev_meas;
    p->prev_meas = err;

    /* prev_out 只累积反馈量; 前馈单独叠加, 不参与积分, 防止前馈误差被积死 */
    p->prev_out = clampf(p->prev_out + delta, p->out_min, p->out_max);
    float ff = p->kf * target;
    if      (target >  1e-3f) ff += p->ks;   /* 静摩擦前馈, 正反转分别标定后 */
    else if (target < -1e-3f) ff -= p->ks;   /* 可拆成两个符号不对称的值 */
    return clampf(p->prev_out + ff, p->out_min, p->out_max);
}
