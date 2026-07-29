#include "gray8.h"

static inline float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

static const float k_def_w[GRAY_N] = {-7, -5, -3, -1, 1, 3, 5, 7};

void gray8_init(gray8_t *g)
{
    for (int i = 0; i < GRAY_N; i++) {
        g->weights[i]  = k_def_w[i] / 7.0f;   /* 归一化到 [-1,1] */
        g->cal_white[i] = 3500;               /* ADC 12bit 默认值, 现场必须重校 */
        g->cal_black[i] = 500;
    }
    g->err = g->err_lpf = 0.0f;
    g->lpf_alpha = 0.6f;
    g->state = GRAY_STATE_ON_LINE;
    g->last_dir = 0.0f;
    g->black_cnt = 0;
    g->cross_min = 6;
    g->cross_debounce = 0;
    g->cross_events = 0;
    g->inverted = false;
    g->noise_floor = 0.15f;
    g->median_en = true;
    g->nl_k = 0.0f;
    g->med_primed = false;
}

static inline uint16_t median3_u16(uint16_t a, uint16_t b, uint16_t c)
{
    if (a > b) { uint16_t t = a; a = b; b = t; }
    if (b > c) { b = c; }
    return a > b ? a : b;
}

/* 由归一化黑度 dark[i]∈[0,1] 计算误差与状态 */
static void gray8_core(gray8_t *g, const float dark[GRAY_N])
{
    float sum = 0.0f, wsum = 0.0f;
    uint8_t cnt = 0;
    for (int i = 0; i < GRAY_N; i++) {
        float d = dark[i];
        if (d > 0.5f) cnt++;
        sum  += d;
        wsum += d * g->weights[i];
    }
    g->black_cnt = cnt;

    if (cnt == 0) {
        /* 丢线: 保持记忆方向满打误差, 让车全力回线 */
        g->state = GRAY_STATE_LOST;
        g->err = (g->last_dir >= 0.0f) ? 1.0f : -1.0f;
    } else if (cnt >= g->cross_min) {
        /* 十字/横线: 误差按 0 处理直行冲过, 并产生事件(带 8 周期去抖) */
        g->state = GRAY_STATE_CROSS;
        g->err = 0.0f;
        if (g->cross_debounce == 0) g->cross_events++;
        g->cross_debounce = 8;
    } else {
        g->state = GRAY_STATE_ON_LINE;
        g->err = wsum / sum;                 /* 加权平均线位置 */
        if (g->nl_k > 0.0f) {                /* 大偏差增益增强(可选) */
            float e = g->err;
            g->err = (1.0f - g->nl_k) * e + g->nl_k * e * e * e;
        }
        if (g->err > 0.05f)  g->last_dir = 1.0f;
        if (g->err < -0.05f) g->last_dir = -1.0f;
    }
    if (g->cross_debounce) g->cross_debounce--;

    g->err_lpf += g->lpf_alpha * (g->err - g->err_lpf);
}

void gray8_update_analog(gray8_t *g, const uint16_t raw[GRAY_N])
{
    float dark[GRAY_N];
    if (!g->med_primed) {                    /* 首帧种入历史, 避免开机假黑 */
        for (int i = 0; i < GRAY_N; i++) { g->med_h1[i] = g->med_h2[i] = raw[i]; }
        g->med_primed = true;
    }
    for (int i = 0; i < GRAY_N; i++) {
        uint16_t r = raw[i];
        if (g->median_en) {                  /* 3 点中位值: 去地板反光尖峰 */
            uint16_t m = median3_u16(raw[i], g->med_h1[i], g->med_h2[i]);
            g->med_h2[i] = g->med_h1[i];
            g->med_h1[i] = raw[i];
            r = m;
        }
        float w = (float)g->cal_white[i], b = (float)g->cal_black[i];
        float span = w - b;
        float d = (span > 1.0f || span < -1.0f) ? (w - (float)r) / span : 0.0f;
        if (g->inverted) d = 1.0f - d;
        d = clamp01(d);
        if (d < g->noise_floor) d = 0.0f;    /* 噪声地板: 反光不参与加权 */
        dark[i] = d;
    }
    gray8_core(g, dark);
}

void gray8_update_digital(gray8_t *g, uint8_t bits)
{
    float dark[GRAY_N];
    for (int i = 0; i < GRAY_N; i++) {
        bool on = (bits >> i) & 1u;
        if (g->inverted) on = !on;
        dark[i] = on ? 1.0f : 0.0f;
    }
    gray8_core(g, dark);
}

void gray8_cal_white(gray8_t *g, const uint16_t raw[GRAY_N])
{
    for (int i = 0; i < GRAY_N; i++) g->cal_white[i] = raw[i];
}

void gray8_cal_black(gray8_t *g, const uint16_t raw[GRAY_N])
{
    for (int i = 0; i < GRAY_N; i++) g->cal_black[i] = raw[i];
}
