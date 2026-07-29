/**
 * radio_link.h — 双车/跨设备协同帧协议(物理层无关: HC-05/NRF透传/ESP-NOW桥均可)
 *
 * 设计依据(见 2026电赛小车算法方法参考.md 第5节):
 *  - 周期状态帧(领头车 20-50Hz 广播): 幂等, 丢了下一帧覆盖, 收端只取最新;
 *  - 事件帧(START/换道/FINISH/角色互换): 带 seq, 需 ACK, 自动重发 3 次去重;
 *  - 心跳 = 状态帧兼任: rl_link_alive() 超时(建议 300-500ms)后任务层降级
 *    (斜坡减速安全停车, 别急停);
 *  - 帧格式: AA 55 | type | seq | payload[10] | CRC8  共 15 字节定长。
 *
 * 用法(领头车):  rl_send_state() 周期发 + rl_poll() 周期调;
 * 用法(跟随车):  app_on_radio_byte -> rl_rx_byte(); 用 peer.v_mmps 做速度前馈,
 *               peer.odo_mm 与本车里程差做间距环反馈; rl_take_event() 取事件。
 * 同一份固件烧两车, 角色用 flags 里的 role 位区分。
 */
#ifndef RADIO_LINK_H
#define RADIO_LINK_H

#include <stdint.h>
#include <stdbool.h>

#define RL_FRAME_LEN   15
#define RL_PAYLOAD_N   10

#define RL_T_STATE     0x01
#define RL_T_EVENT     0x02
#define RL_T_ACK       0x03

/* flags 位定义(按题扩展) */
#define RL_FLAG_ROLE_LEADER  0x01
#define RL_FLAG_ESTOP        0x80

/* 事件 id 建议值(按题扩展) */
#define RL_EV_START    0x01
#define RL_EV_FINISH   0x02
#define RL_EV_SWAP     0x03   /* 角色互换 */
#define RL_EV_LANE     0x04   /* 换道/超车 */

typedef struct {
    uint8_t  mode;        /* 任务模式/问号 */
    int16_t  v_mmps;      /* 目标速度 mm/s(前馈用) */
    int32_t  odo_mm;      /* 累计里程 mm(间距=双方里程差, 无弯道盲区) */
    uint8_t  checkpoint;  /* 停止线/圈计数(从车投票纠偏用) */
    uint8_t  flags;       /* RL_FLAG_* */
} rl_state_t;

typedef void (*rl_tx_fn)(const uint8_t *data, uint16_t len);

typedef struct {
    /* --- 接收解析状态机 --- */
    uint8_t  st, type, seq, idx;
    uint8_t  buf[RL_PAYLOAD_N];
    uint8_t  crc_acc;
    bool     seq_seen;
    uint8_t  last_seq;

    /* --- 对端状态(最新一帧) --- */
    rl_state_t peer;
    uint32_t peer_ms;     /* 最后有效状态帧时刻(心跳) */
    bool     peer_valid;

    /* --- 收到的事件(去重后待取) --- */
    bool     ev_rx_new;
    uint8_t  ev_rx_id;
    uint32_t ev_rx_arg;
    uint8_t  ev_rx_seq;
    bool     ev_rx_seen;

    /* --- 发送 --- */
    rl_tx_fn tx;
    uint8_t  tx_seq;
    uint8_t  ev_frame[RL_FRAME_LEN];  /* 待确认事件帧(重发用) */
    uint8_t  ev_pending;              /* 0=空闲, 否则=剩余重发次数 */
    uint8_t  ev_wait_seq;
    uint32_t ev_next_ms;

    /* --- 链路统计(接入 VOFA/OLED 链路页) --- */
    uint16_t rx_ok, crc_err, lost, ev_fail;
} radio_link_t;

void    rl_init(radio_link_t *l, rl_tx_fn tx);
void    rl_rx_byte(radio_link_t *l, uint8_t b, uint32_t now_ms);
bool    rl_link_alive(const radio_link_t *l, uint32_t now_ms, uint32_t timeout_ms);
void    rl_send_state(radio_link_t *l, const rl_state_t *s);
/* 发事件(带 ACK+重发); 返回 false = 上一个事件还没确认完 */
bool    rl_send_event(radio_link_t *l, uint8_t event_id, uint32_t arg, uint32_t now_ms);
/* 周期调用(20-50ms): 驱动事件重发 */
void    rl_poll(radio_link_t *l, uint32_t now_ms);
/* 取收到的事件(已按 seq 去重); 返回 false = 没有新事件 */
bool    rl_take_event(radio_link_t *l, uint8_t *id, uint32_t *arg);
uint8_t rl_crc8(const uint8_t *d, uint16_t n);

#endif
