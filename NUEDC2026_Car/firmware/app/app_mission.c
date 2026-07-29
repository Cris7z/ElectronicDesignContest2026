#include "app_mission.h"
#include "../bsp/bsp.h"
#include <math.h>

/* ================= 示例任务表: 2024H 风格 =================
 * 循迹到第1个十字 -> 无引导线直行1.0m -> 右转90° -> 循迹0.8m -> 缓停 -> 响3声
 * 拿到真题后: 改这张表 + 需要时在 mission_update 里加新原语, 其他文件基本不动 */
/* 字段: { op, p1, p2, p3(trim, 现场逐圈补偿), timeout_ms(0=默认15s) } */
const mi_step_t demo_mission[] = {
    { MI_SET_SRC,        SRC_GRAY, 0,    0, 0 },
    { MI_FOLLOW_TO_CROSS, 1,      0.8f,  0, 0 },
    { MI_DEAD_STRAIGHT,  1.0f,    0.7f,  0, 0 },
    { MI_TURN_TO,        -90.0f,  0.6f,  0, 0 },
    { MI_FOLLOW_DIST,    0.8f,    0.8f,  0, 0 },
    { MI_STOP_AT,        0.10f,   0,     0, 0 },
    { MI_BEEP,           3,       0,     0, 0 },
    { MI_END,            0,       0,     0, 0 },
};

#define MI_DEFAULT_TIMEOUT_MS  15000u   /* 每原语默认超时(可被 step.timeout_ms 覆盖) */

void mission_init(mission_t *m)
{
    m->prog = 0; m->step = 0; m->running = 0; m->beep_left = 0;
    m->settle_cnt = 0; m->timed_out = 0; m->chirp_until_ms = 0;
    /* 距离环: 输入剩余距离(m), 输出目标速度(m/s) — 临近减速, 走完停死 */
    pid_init(&m->pid_dist, 4.0f, 0.0f, 0.0f, -0.5f, 0.5f);
}

static void step_enter(mission_t *m, odom_t *od, gray8_t *g, imu_t *imu,
                       track_t *t, uint32_t now_ms)
{
    m->step_start_dist  = od->dist_m;
    m->step_start_cross = g->cross_events;
    m->step_start_ms    = now_ms;
    m->settle_cnt = 0;
    /* 统一清理: 上一段的微分/积分历史不许踢新段 */
    pid_reset(&m->pid_dist);
    track_reset_pids(t);
    const mi_step_t *s = &m->prog[m->step];
    if (s->op != MI_BEEP && s->op != MI_END)
        m->chirp_until_ms = now_ms + 40u;    /* 切态短鸣: 听声定位当前状态 */
    switch (s->op) {
    case MI_DEAD_STRAIGHT:
        /* 锁定当前目标航向直行(不是当前实际yaw, 避免误差累积) */
        track_use(t, SRC_YAW);
        break;
    case MI_TURN_TO:
        track_use(t, SRC_YAW);
        track_set_yaw_target(t, s->p1 + s->p3);
        break;
    case MI_FOLLOW_TO_CROSS:
    case MI_FOLLOW_DIST:
        track_use(t, SRC_GRAY);
        break;
    case MI_BEEP:
        m->beep_left = (int)s->p1;
        break;
    default: break;
    }
    (void)imu;
}

void mission_start(mission_t *m, const mi_step_t *prog, uint32_t now_ms,
                   odom_t *od, gray8_t *g, imu_t *imu, track_t *t)
{
    m->prog = prog; m->step = 0; m->running = 1; m->timed_out = 0;
    odom_reset(od);
    imu_zero_yaw(imu);
    track_set_yaw_target(t, 0.0f);
    g->cross_events = 0;
    step_enter(m, od, g, imu, t, now_ms);
}

void mission_abort(mission_t *m, chassis_t *c)
{
    m->running = 0;
    chassis_set(c, 0.0f, 0.0f);
    chassis_enable(c, 0);
    bsp_buzzer(0);
}

static void step_next(mission_t *m, odom_t *od, gray8_t *g, imu_t *imu,
                      track_t *t, uint32_t now_ms)
{
    m->step++;
    step_enter(m, od, g, imu, t, now_ms);
}

void mission_update(mission_t *m, chassis_t *c, track_t *t, odom_t *od,
                    gray8_t *g, imu_t *imu, openmv_t *mv, uint32_t now_ms)
{
    if (!m->running || !m->prog) return;

    const mi_step_t *s = &m->prog[m->step];
    float p1 = s->p1 + s->p3;                /* p3 = 现场 trim(逐圈补偿) */
    float walked = od->dist_m - m->step_start_dist;
    float steer = track_update(t, g, imu, mv, now_ms);

    /* 超时防御: 卡死在某状态 = 整问 0 分。WAIT/BEEP 自带时长, 不套默认超时 */
    if (s->op != MI_END && s->op != MI_WAIT_MS && s->op != MI_BEEP) {
        uint32_t tmo = (s->timeout_ms > 0.5f) ? (uint32_t)s->timeout_ms
                                              : MI_DEFAULT_TIMEOUT_MS;
        if (now_ms - m->step_start_ms > tmo) {
            m->timed_out = 1;
            mission_abort(m, c);
            return;
        }
    }
    /* 切态短鸣(非阻塞); MI_BEEP 自己管蜂鸣器 */
    if (s->op != MI_BEEP)
        bsp_buzzer(now_ms < m->chirp_until_ms);

    switch (s->op) {
    case MI_END:
        mission_abort(m, c);
        return;

    case MI_SET_SRC:
        track_use(t, (track_src_t)(int)s->p1);
        step_next(m, od, g, imu, t, now_ms);
        return;

    case MI_FOLLOW_TO_CROSS:
        chassis_set(c, s->p2, steer);
        if ((int)(g->cross_events - m->step_start_cross) >= (int)s->p1)
            step_next(m, od, g, imu, t, now_ms);
        break;

    case MI_FOLLOW_DIST:
        chassis_set(c, s->p2, steer);
        if (walked >= p1) step_next(m, od, g, imu, t, now_ms);
        break;

    case MI_DEAD_STRAIGHT:
        chassis_set(c, s->p2, steer);
        if (walked >= p1) step_next(m, od, g, imu, t, now_ms);
        break;

    case MI_TURN_TO: {
        /* 最短路径误差 + 连续 N 帧判稳(防惯性穿过目标时误判到位) */
        float err = angle_wrap_180(p1 - imu_yaw(imu));
        chassis_set(c, 0.0f, steer);   /* 原地转: 线速度0, 只给差速 */
        if (fabsf(err) < 2.0f && fabsf(chassis_omega(c)) < 0.3f) {
            if (++m->settle_cnt >= 10)     /* 10 帧 @100Hz = 100ms 稳定 */
                step_next(m, od, g, imu, t, now_ms);
        } else {
            m->settle_cnt = 0;
        }
        break;
    }

    case MI_STOP_AT: {
        float remain = p1 - walked;
        float v = pid_pos(&m->pid_dist, remain, walked);
        if (v < 0.0f) v = 0.0f;
        chassis_set(c, v, steer);
        if (remain < 0.02f && fabsf(chassis_speed(c)) < 0.03f)
            step_next(m, od, g, imu, t, now_ms);
        break;
    }

    case MI_WAIT_MS:
        chassis_set(c, 0.0f, 0.0f);
        if (now_ms - m->step_start_ms >= (uint32_t)s->p1)
            step_next(m, od, g, imu, t, now_ms);
        break;

    case MI_BEEP:
        chassis_set(c, 0.0f, 0.0f);
        /* 200ms 响 200ms 停 */
        if (m->beep_left > 0) {
            uint32_t el = now_ms - m->step_start_ms;
            uint32_t phase = el / 200u;
            bsp_buzzer(phase % 2u == 0u);
            if ((int)(phase / 2u) >= m->beep_left) {
                bsp_buzzer(0);
                step_next(m, od, g, imu, t, now_ms);
            }
        } else step_next(m, od, g, imu, t, now_ms);
        break;

    default:
        step_next(m, od, g, imu, t, now_ms);
        break;
    }
    (void)mv;
}
