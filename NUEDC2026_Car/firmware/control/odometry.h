/**
 * odometry.h — 里程计 (编码器测距 + 陀螺仪测向的互补融合)
 * theta 优先用陀螺仪(不漂移的短时精度高), 陀螺仪失联时退化为纯编码器差分。
 */
#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float x_m, y_m;        /* 世界系坐标, 起点为原点, 起始朝向为 +x */
    float theta_rad;       /* 航向角 */
    float dist_m;          /* 累计路程(绝对值累计) */
    float _last_dl, _last_dr;
} odom_t;

void odom_init(odom_t *o);
void odom_reset(odom_t *o);
/* dl/dr: 左右轮累计里程(米); yaw_rad: 陀螺仪连续航向(弧度), 无效传 NAN */
void odom_update(odom_t *o, float dl, float dr, float track_width_m,
                 float yaw_rad, bool yaw_ok);

#endif
