#include "track_ctrl.h"

void track_init(track_t *t)
{
    /* 循迹 PD: kp 决定回线快慢, kd 压振荡。输出即轮速差(m/s)。
     * 起步参考: kp=1.2, kd=6.0 (err 是 [-1,1] 的小量, kd 偏大正常) */
    pid_init(&t->pid_line, 1.2f, 0.0f, 6.0f, -1.5f, 1.5f);
    t->pid_line.d_lpf_alpha = 0.35f;

    /* 航向 PD: 输入误差单位是度 */
    pid_init(&t->pid_yaw, 0.06f, 0.0f, 0.10f, -1.2f, 1.2f);
    t->pid_yaw.d_lpf_alpha = 0.4f;

    t->vision_angle_k = 0.010f;   /* 每度线倾角补多少 steer */
    t->src = SRC_GRAY;
    t->active = SRC_GRAY;
    t->yaw_target_deg = 0.0f;
    t->steer_limit = 1.2f;
}

void track_use(track_t *t, track_src_t src)
{
    if (t->src == src) return;
    t->src = src;
    pid_reset(&t->pid_line);
    pid_reset(&t->pid_yaw);
}

void track_set_yaw_target(track_t *t, float deg) { t->yaw_target_deg = deg; }

void track_reset_pids(track_t *t)
{
    pid_reset(&t->pid_line);
    pid_reset(&t->pid_yaw);
}

float track_update(track_t *t, const gray8_t *g, const imu_t *imu,
                   const openmv_t *mv, uint32_t now_ms)
{
    track_src_t use = t->src;

    /* --- 自动降级 --- */
    if (use == SRC_VISION && !openmv_alive(mv, now_ms))
        use = (g->state != GRAY_STATE_LOST) ? SRC_GRAY : SRC_YAW;
    if (use == SRC_GRAY && g->state == GRAY_STATE_LOST && imu_alive(imu, now_ms)) {
        /* 灰度丢线短时间内先靠记忆方向抢线(gray8 已给满偏差),
         * 这里不切 YAW, 由任务层决定; 保持 GRAY */
    }
    t->active = use;

    float steer = 0.0f;
    switch (use) {
    case SRC_GRAY:
        steer = pid_pos(&t->pid_line, -g->err_lpf, g->err_lpf);
        break;
    case SRC_VISION: {
        float e = mv->line_valid ? mv->line_err : 0.0f;
        steer = pid_pos(&t->pid_line, -e, e)
              - t->vision_angle_k * mv->line_angle_deg;  /* 角度预瞄前馈 */
        break;
    }
    case SRC_YAW: {
        /* 最短路径误差: 多圈任务后连续角可能累到 ±N*360°, wrap 后不会绕远路回转 */
        float err = angle_wrap_180(t->yaw_target_deg - imu_yaw(imu));
        steer = pid_pos(&t->pid_yaw, err, imu_yaw(imu));
        break;
    }
    }
    return clampf(steer, -t->steer_limit, t->steer_limit);
}
