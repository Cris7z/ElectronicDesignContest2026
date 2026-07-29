/**
 * gray8.h — 8路灰度/红外循迹传感器处理
 * 支持: 数字量(8bit) 或 模拟量(8通道ADC, 带逐通道校准归一化)
 * 输出: 加权平均线位置误差 [-1, +1], 丢线记忆, 十字/全白事件检测
 */
#ifndef GRAY8_H
#define GRAY8_H

#include <stdint.h>
#include <stdbool.h>

#define GRAY_N 8

typedef enum {
    GRAY_STATE_ON_LINE = 0,  /* 正常在线上 */
    GRAY_STATE_LOST,         /* 丢线(全白), 用记忆方向 */
    GRAY_STATE_CROSS,        /* 十字/横线(全黑或≥6路黑) */
} gray_state_t;

typedef struct {
    /* 校准: 每通道白/黑参考值(模拟量用), 按键触发采集 */
    uint16_t cal_white[GRAY_N];
    uint16_t cal_black[GRAY_N];

    float weights[GRAY_N];   /* 默认 -7,-5,-3,-1,1,3,5,7 归一化 */
    float err;               /* 当前误差 [-1,1], 左负右正 */
    float err_lpf;           /* 低通后的误差 */
    float lpf_alpha;         /* 建议 0.5~0.8, 太小会引入相位滞后 */
    gray_state_t state;
    float last_dir;          /* 丢线记忆: 丢线前误差符号(+1/-1) */
    uint8_t black_cnt;       /* 当前黑线路数 */
    uint8_t cross_min;       /* ≥此路数判为十字, 默认6 */
    uint16_t cross_debounce; /* 十字去抖计数 */
    uint16_t cross_events;   /* 累计十字事件数(任务状态机用) */
    bool inverted;           /* true=白线黑底 */

    /* --- 信号前端(Pololu readLine 范式 + 智能车抗扰套路) --- */
    float noise_floor;       /* 归一化黑度低于此值的通道置零(防反光), 默认0.15 */
    bool  median_en;         /* 模拟量 3 点中位值滤波(去尖峰), 默认开 */
    float nl_k;              /* 非线性误差映射强度 0~1: err'=(1-k)err+k·err³,
                                大偏差增益增强(差比和差思想)。默认 0=线性,
                                上车调好线性参数后再开, 开后循迹 Kp 降到 0.7~0.8 倍 */
    uint16_t med_h1[GRAY_N]; /* 中位值滤波历史(上一拍/上上拍) */
    uint16_t med_h2[GRAY_N];
    bool  med_primed;
} gray8_t;

void gray8_init(gray8_t *g);
/* 模拟量入口: raw[8] 为 ADC 原始值; 内部先归一化再算误差 */
void gray8_update_analog(gray8_t *g, const uint16_t raw[GRAY_N]);
/* 数字量入口: bits 的 bit0..bit7 = 传感器1..8, 1=压线 */
void gray8_update_digital(gray8_t *g, uint8_t bits);
/* 校准: 全白地面上按一下 / 全黑线上按一下 */
void gray8_cal_white(gray8_t *g, const uint16_t raw[GRAY_N]);
void gray8_cal_black(gray8_t *g, const uint16_t raw[GRAY_N]);

#endif
