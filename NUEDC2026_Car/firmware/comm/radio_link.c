#include "radio_link.h"
#include <string.h>

/* CRC8, 多项式 0x07, 初值 0x00, 覆盖 type..payload 共 12 字节 */
uint8_t rl_crc8(const uint8_t *d, uint16_t n)
{
    uint8_t crc = 0x00;
    for (uint16_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x80u) ? (uint8_t)((crc << 1) ^ 0x07u) : (uint8_t)(crc << 1);
    }
    return crc;
}

void rl_init(radio_link_t *l, rl_tx_fn tx)
{
    memset(l, 0, sizeof(*l));
    l->tx = tx;
}

bool rl_link_alive(const radio_link_t *l, uint32_t now_ms, uint32_t timeout_ms)
{
    return l->peer_valid && (now_ms - l->peer_ms) <= timeout_ms;
}

/* 组一帧: type + payload -> out[15] */
static void rl_build(radio_link_t *l, uint8_t type, const uint8_t pay[RL_PAYLOAD_N],
                     uint8_t out[RL_FRAME_LEN])
{
    out[0] = 0xAA; out[1] = 0x55;
    out[2] = type;
    out[3] = ++l->tx_seq;
    memcpy(&out[4], pay, RL_PAYLOAD_N);
    out[14] = rl_crc8(&out[2], 2 + RL_PAYLOAD_N);
}

void rl_send_state(radio_link_t *l, const rl_state_t *s)
{
    uint8_t pay[RL_PAYLOAD_N];
    uint8_t f[RL_FRAME_LEN];
    pay[0] = s->mode;
    pay[1] = (uint8_t)(s->v_mmps & 0xFF);
    pay[2] = (uint8_t)((s->v_mmps >> 8) & 0xFF);
    pay[3] = (uint8_t)(s->odo_mm & 0xFF);
    pay[4] = (uint8_t)((s->odo_mm >> 8) & 0xFF);
    pay[5] = (uint8_t)((s->odo_mm >> 16) & 0xFF);
    pay[6] = (uint8_t)((s->odo_mm >> 24) & 0xFF);
    pay[7] = s->checkpoint;
    pay[8] = s->flags;
    pay[9] = 0;
    rl_build(l, RL_T_STATE, pay, f);
    if (l->tx) l->tx(f, RL_FRAME_LEN);
}

bool rl_send_event(radio_link_t *l, uint8_t event_id, uint32_t arg, uint32_t now_ms)
{
    if (l->ev_pending) return false;      /* 一次只挂一个待确认事件 */
    uint8_t pay[RL_PAYLOAD_N] = {0};
    pay[0] = event_id;
    pay[1] = (uint8_t)(arg & 0xFF);
    pay[2] = (uint8_t)((arg >> 8) & 0xFF);
    pay[3] = (uint8_t)((arg >> 16) & 0xFF);
    pay[4] = (uint8_t)((arg >> 24) & 0xFF);
    rl_build(l, RL_T_EVENT, pay, l->ev_frame);
    l->ev_wait_seq = l->ev_frame[3];
    l->ev_pending  = 3;                   /* 首发 + 最多重发 2 次 */
    l->ev_next_ms  = now_ms;              /* rl_poll 立即发出 */
    return true;
}

void rl_poll(radio_link_t *l, uint32_t now_ms)
{
    if (!l->ev_pending) return;
    if ((int32_t)(now_ms - l->ev_next_ms) < 0) return;
    if (l->tx) l->tx(l->ev_frame, RL_FRAME_LEN);
    l->ev_pending--;
    l->ev_next_ms = now_ms + 30u;         /* 重发间隔 */
    if (!l->ev_pending) l->ev_fail++;     /* 用尽仍无 ACK: 计一次失败 */
}

bool rl_take_event(radio_link_t *l, uint8_t *id, uint32_t *arg)
{
    if (!l->ev_rx_new) return false;
    l->ev_rx_new = false;
    if (id)  *id  = l->ev_rx_id;
    if (arg) *arg = l->ev_rx_arg;
    return true;
}

static void rl_on_frame(radio_link_t *l, uint32_t now_ms)
{
    /* seq 丢帧统计(对端所有帧共用一个滚动 seq) */
    if (l->seq_seen) {
        uint8_t gap = (uint8_t)(l->seq - l->last_seq);
        if (gap > 1u) l->lost += (uint16_t)(gap - 1u);
    }
    l->seq_seen = true;
    l->last_seq = l->seq;
    l->rx_ok++;

    switch (l->type) {
    case RL_T_STATE:
        l->peer.mode    = l->buf[0];
        l->peer.v_mmps  = (int16_t)((uint16_t)l->buf[1] | ((uint16_t)l->buf[2] << 8));
        l->peer.odo_mm  = (int32_t)((uint32_t)l->buf[3] | ((uint32_t)l->buf[4] << 8) |
                          ((uint32_t)l->buf[5] << 16) | ((uint32_t)l->buf[6] << 24));
        l->peer.checkpoint = l->buf[7];
        l->peer.flags      = l->buf[8];
        l->peer_ms    = now_ms;
        l->peer_valid = true;
        break;

    case RL_T_EVENT: {
        /* 去重: 同一 seq 的重发只上报一次, 但每次都回 ACK */
        if (!l->ev_rx_seen || l->seq != l->ev_rx_seq) {
            l->ev_rx_seen = true;
            l->ev_rx_seq  = l->seq;
            l->ev_rx_id   = l->buf[0];
            l->ev_rx_arg  = (uint32_t)l->buf[1] | ((uint32_t)l->buf[2] << 8) |
                            ((uint32_t)l->buf[3] << 16) | ((uint32_t)l->buf[4] << 24);
            l->ev_rx_new  = true;
        }
        uint8_t pay[RL_PAYLOAD_N] = {0};
        uint8_t f[RL_FRAME_LEN];
        pay[0] = l->seq;                  /* ACK 带被确认事件的 seq */
        rl_build(l, RL_T_ACK, pay, f);
        if (l->tx) l->tx(f, RL_FRAME_LEN);
        break;
    }

    case RL_T_ACK:
        if (l->ev_pending && l->buf[0] == l->ev_wait_seq)
            l->ev_pending = 0;            /* 确认到位, 停止重发 */
        break;

    default:
        break;
    }
}

void rl_rx_byte(radio_link_t *l, uint8_t b, uint32_t now_ms)
{
    switch (l->st) {
    case 0: l->st = (b == 0xAA) ? 1 : 0; break;
    case 1: l->st = (b == 0x55) ? 2 : (b == 0xAA ? 1 : 0); break;
    case 2: l->type = b; l->crc_acc = 0; l->st = 3; break;
    case 3: l->seq = b; l->idx = 0; l->st = 4; break;
    case 4:
        l->buf[l->idx++] = b;
        if (l->idx >= RL_PAYLOAD_N) l->st = 5;
        break;
    case 5: {
        uint8_t hdr[2 + RL_PAYLOAD_N];
        hdr[0] = l->type; hdr[1] = l->seq;
        memcpy(&hdr[2], l->buf, RL_PAYLOAD_N);
        if (rl_crc8(hdr, sizeof(hdr)) == b) rl_on_frame(l, now_ms);
        else l->crc_err++;
        l->st = 0;
        break;
    }
    default: l->st = 0; break;
    }
}
