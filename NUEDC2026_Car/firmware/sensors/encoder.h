/**
 * encoder.h — 正交编码器测速 + 里程
 * 由 BSP 提供硬件计数值(有符号累计), 本模块做差分测速 + 一阶低通。
 */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

typedef struct {
    /* 机械参数 — 按自己的车改! */
    float counts_per_rev;   /* 电机输出轴每转编码器计数(含减速比与4倍频) */
    float wheel_diameter_m; /* 轮径(米) */

    int32_t last_count;
    float speed_mps;        /* 低通后的轮速 m/s */
    float speed_raw;
    float dist_m;           /* 累计里程(米, 有符号) */
    float lpf_alpha;        /* 0.3~0.6 */
    uint8_t first;
} encoder_t;

void  encoder_init(encoder_t *e, float counts_per_rev, float wheel_diameter_m);
/* dt: 调用周期(秒), count: 当前累计计数(BSP读硬件) */
void  encoder_update(encoder_t *e, int32_t count, float dt);

#endif
