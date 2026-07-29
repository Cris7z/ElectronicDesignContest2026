#include "openmv_link.h"

void openmv_init(openmv_t *o)
{
    o->line_err = 0.0f; o->line_angle_deg = 0.0f;
    o->line_valid = false; o->see_cross = false;
    o->target_class = 0; o->target_cx = o->target_cy = 0;
    o->frames = 0; o->last_ms = 0; o->idx = 0;
}

void openmv_rx_byte(openmv_t *o, uint8_t b, uint32_t now_ms)
{
    /* 帧: [0]=0xAA [1]=0x55 [2]=type [3..8]=payload [9]=sum [10]=0x0D */
    if (o->idx == 0 && b != 0xAA) return;
    if (o->idx == 1 && b != 0x55) { o->idx = 0; return; }
    o->buf[o->idx++] = b;
    if (o->idx < 11) return;
    o->idx = 0;

    if (o->buf[10] != 0x0D) return;
    uint8_t sum = 0;
    for (int i = 2; i <= 8; i++) sum += o->buf[i];
    if (sum != o->buf[9]) return;

    const uint8_t *p = &o->buf[3];
    switch (o->buf[2]) {
    case 0x01: {
        int16_t e = (int16_t)(p[0] | (p[1] << 8));
        int16_t a = (int16_t)(p[2] | (p[3] << 8));
        uint8_t flags = p[4];
        o->line_err = e / 1000.0f;
        o->line_angle_deg = a / 10.0f;
        o->line_valid = flags & 0x01;
        o->see_cross  = (flags >> 1) & 0x01;
        if ((flags >> 2) & 0x01 && o->target_class == 0)
            o->target_class = 255;  /* 看到目标但未分类 */
        break;
    }
    case 0x02:
        o->target_class = p[0];
        o->target_cx = (int16_t)(p[1] | (p[2] << 8));
        o->target_cy = (int16_t)(p[3] | (p[4] << 8));
        break;
    default: return;
    }
    o->frames++;
    o->last_ms = now_ms;
}

bool openmv_alive(const openmv_t *o, uint32_t now_ms)
{
    return o->frames > 0 && (now_ms - o->last_ms) < 150u;
}
