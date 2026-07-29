#include "imu_jy61p.h"

void imu_init(imu_t *m)
{
    m->roll = m->pitch = m->yaw = 0.0f;
    m->yaw_cont = m->yaw_offset = 0.0f;
    m->gz_dps = 0.0f;
    m->frames = 0; m->last_ms = 0;
    m->idx = 0; m->_prev_yaw = 0.0f; m->_first = true;
}

static void imu_push_yaw(imu_t *m, float yaw, uint32_t now_ms)
{
    if (m->_first) {
        /* 首帧: 连续角直接对齐绝对角, 不能丢掉初始朝向! */
        m->_prev_yaw = yaw;
        m->yaw_cont = yaw;
        m->_first = false;
    }
    float d = yaw - m->_prev_yaw;
    if (d > 180.0f)  d -= 360.0f;   /* 跳变展开 */
    if (d < -180.0f) d += 360.0f;
    m->yaw_cont += d;
    m->_prev_yaw = yaw;
    m->yaw = yaw;
    m->frames++;
    m->last_ms = now_ms;
}

void imu_rx_byte(imu_t *m, uint8_t b, uint32_t now_ms)
{
    if (m->idx == 0 && b != 0x55) return;
    m->buf[m->idx++] = b;
    if (m->idx < 11) return;
    m->idx = 0;

    uint8_t sum = 0;
    for (int i = 0; i < 10; i++) sum += m->buf[i];
    if (sum != m->buf[10]) return;              /* 校验失败丢帧 */

    int16_t v[3];
    for (int i = 0; i < 3; i++)
        v[i] = (int16_t)((m->buf[3 + 2*i] << 8) | m->buf[2 + 2*i]);

    switch (m->buf[1]) {
    case 0x52:  /* 角速度帧: ±2000dps */
        m->gz_dps = v[2] / 32768.0f * 2000.0f;
        break;
    case 0x53:  /* 角度帧: ±180° */
        m->roll  = v[0] / 32768.0f * 180.0f;
        m->pitch = v[1] / 32768.0f * 180.0f;
        imu_push_yaw(m, v[2] / 32768.0f * 180.0f, now_ms);
        break;
    default: break;
    }
}

void imu_feed_yaw(imu_t *m, float yaw_deg, uint32_t now_ms)
{
    imu_push_yaw(m, yaw_deg, now_ms);
}

void imu_zero_yaw(imu_t *m) { m->yaw_offset = m->yaw_cont; }

float imu_yaw(const imu_t *m) { return m->yaw_cont - m->yaw_offset; }

bool imu_alive(const imu_t *m, uint32_t now_ms)
{
    return m->frames > 0 && (now_ms - m->last_ms) < 200u;
}
