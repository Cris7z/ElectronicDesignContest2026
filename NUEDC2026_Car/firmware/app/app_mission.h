/**
 * app_mission.h — 赛题任务状态机(魔改主战场!)
 * 内置一套"万能动作原语", 拿到题后用这些原语拼流程即可:
 *   MI_FOLLOW_TO_CROSS  循迹直到第 n 个十字
 *   MI_FOLLOW_DIST      循迹走指定距离
 *   MI_DEAD_STRAIGHT    无引导线: 航向保持直行指定距离
 *   MI_TURN_TO          原地/行进间转到指定航向(度)
 *   MI_STOP_AT          距离环精确停车(误差<2cm)
 *   MI_WAIT_MS          原地等待
 *   MI_BEEP             声光提示
 * 示例任务表 demo_mission[] 复现了 2024H 风格: 循迹->无线段直行->转弯->停车
 */
#ifndef APP_MISSION_H
#define APP_MISSION_H

#include <stdint.h>
#include <stdbool.h>
#include "../control/chassis.h"
#include "../control/track_ctrl.h"
#include "../control/odometry.h"
#include "../sensors/gray8.h"
#include "../sensors/imu_jy61p.h"
#include "../sensors/openmv_link.h"

typedef enum {
    MI_END = 0,
    MI_FOLLOW_TO_CROSS,   /* p1=第几个十字(相对本步开始) p2=速度 */
    MI_FOLLOW_DIST,       /* p1=距离m p2=速度 */
    MI_DEAD_STRAIGHT,     /* p1=距离m p2=速度 (航向=进入本步时的目标) */
    MI_TURN_TO,           /* p1=目标航向度(绝对, 相对起点) p2=原地转速差 */
    MI_STOP_AT,           /* p1=前方剩余距离m, 距离环缓停 */
    MI_WAIT_MS,           /* p1=毫秒 */
    MI_BEEP,              /* p1=次数 */
    MI_SET_SRC,           /* p1=track_src_t */
} mi_op_t;

/* p3: 现场修正量(trim), 运行时加到 p1 上(距离/角度原语用)。多圈任务把
 *     同一段复制 N 份、每份 p3 填该圈补偿值, 即"逐圈查表补偿"(24H 套路);
 * timeout_ms: 本步超时上限, 0=默认 15s。超时 -> 安全停车 + timed_out 置位。
 * 老写法 {op,p1,p2} 仍兼容(缺省字段=0)。 */
typedef struct { mi_op_t op; float p1, p2, p3, timeout_ms; } mi_step_t;

typedef struct {
    const mi_step_t *prog;
    int step;
    int running;
    /* 本步上下文 */
    float step_start_dist;
    uint16_t step_start_cross;
    uint32_t step_start_ms;
    int beep_left;
    int settle_cnt;      /* 转弯到位判稳: 连续 N 帧在窗口内 */
    uint8_t timed_out;   /* 原语超时被迫停车标志(遥测/OLED 可显示) */
    uint32_t chirp_until_ms; /* 切态短蜂鸣(听声定位卡在哪个状态) */
    pid_t pid_dist;      /* 精确停车距离环 */
} mission_t;

void mission_init(mission_t *m);
void mission_start(mission_t *m, const mi_step_t *prog, uint32_t now_ms,
                   odom_t *od, gray8_t *g, imu_t *imu, track_t *t);
void mission_abort(mission_t *m, chassis_t *c);
/* 100Hz 调用: 内部调 track_update + chassis_set */
void mission_update(mission_t *m, chassis_t *c, track_t *t, odom_t *od,
                    gray8_t *g, imu_t *imu, openmv_t *mv, uint32_t now_ms);

extern const mi_step_t demo_mission[];

#endif
