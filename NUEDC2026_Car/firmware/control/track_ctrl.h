/**
 * track_ctrl.h — 循迹/航向统一转向控制器 (外环)
 * 三种误差源可切换/自动降级:
 *   SRC_GRAY   灰度循迹(默认, 最稳)
 *   SRC_VISION OpenMV 线性回归循迹(err + angle 双量 PD, 相当于预瞄)
 *   SRC_YAW    陀螺仪航向保持(无引导线路段/定角转弯)
 * 自动降级: VISION 失联 -> GRAY 有线则 GRAY, 否则 YAW 保持当前航向。
 * 输出: steer(轮速差 m/s) 直接喂 chassis_set()。
 */
#ifndef TRACK_CTRL_H
#define TRACK_CTRL_H

#include "pid.h"
#include "../sensors/gray8.h"
#include "../sensors/imu_jy61p.h"
#include "../sensors/openmv_link.h"

typedef enum { SRC_GRAY = 0, SRC_VISION, SRC_YAW } track_src_t;

typedef struct {
    pid_t pid_line;      /* 循迹 PD: 输入误差[-1,1], 输出 steer m/s */
    pid_t pid_yaw;       /* 航向 PD: 输入角误差(度), 输出 steer m/s */
    float vision_angle_k;/* 视觉角度前馈(预瞄)系数 */
    track_src_t src;     /* 期望误差源 */
    track_src_t active;  /* 实际生效的(含降级) */
    float yaw_target_deg;
    float steer_limit;   /* 输出限幅 m/s */
} track_t;

void  track_init(track_t *t);
void  track_use(track_t *t, track_src_t src);
void  track_set_yaw_target(track_t *t, float deg);
/* 任务原语切换时清 PID 历史(防上一段微分踢新段) */
void  track_reset_pids(track_t *t);

/* 角误差最短路径归一到 (-180,180]: 只对"误差"用, 不要对连续角状态量用。
 * 需要刻意转大圈(>180°)时把动作拆成多个 TURN_TO */
static inline float angle_wrap_180(float a)
{
    while (a >  180.0f) a -= 360.0f;
    while (a <= -180.0f) a += 360.0f;
    return a;
}
/* 100Hz 调用, 返回 steer 指令 */
float track_update(track_t *t, const gray8_t *g, const imu_t *imu,
                   const openmv_t *mv, uint32_t now_ms);

#endif
