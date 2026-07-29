/**
 * sim_main.c — PC 端控制算法仿真验证 (gcc 可直接编译, 不依赖硬件)
 * 模型: 差速车运动学 + 一阶电机响应 + S形弯道虚拟赛道 + 虚拟8路灰度
 * 测试:
 *   T1 速度环阶跃: 上升时间/超调/稳态误差
 *   T2 S弯循迹: 最大线偏差(车辆中心相对线, cm)
 *   T3 航向保持直行: 横向漂移
 *   T4 90度定角转弯: 终角误差
 *   T5 demo_mission 全任务链
 *   T6 双车帧协议: 解析/CRC/丢帧统计/事件ACK/心跳超时
 * 判据在 main() 末尾, 全 PASS 才算通过。
 */
#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include "../firmware/control/pid.h"
#include "../firmware/control/chassis.h"
#include "../firmware/control/odometry.h"
#include "../firmware/control/track_ctrl.h"
#include "../firmware/sensors/gray8.h"
#include "../firmware/sensors/imu_jy61p.h"
#include "../firmware/sensors/openmv_link.h"
#include "../firmware/app/app_mission.h"
#include "../firmware/comm/radio_link.h"
#include "../firmware/bsp/bsp.h"

/* ---- mission 依赖的 BSP 桩 ---- */
static int s_beeps = 0;
void bsp_buzzer(bool on) { if (on) s_beeps++; }

/* ---- T6 双车协议回环: A 的 tx 进 q_ab, B 的 tx 进 q_ba ---- */
static uint8_t s_q_ab[512]; static int s_n_ab = 0;
static uint8_t s_q_ba[512]; static int s_n_ba = 0;
static void t6_tx_a(const uint8_t *d, uint16_t n)
{ for (uint16_t i = 0; i < n && s_n_ab < 512; i++) s_q_ab[s_n_ab++] = d[i]; }
static void t6_tx_b(const uint8_t *d, uint16_t n)
{ for (uint16_t i = 0; i < n && s_n_ba < 512; i++) s_q_ba[s_n_ba++] = d[i]; }

#define DT CTRL_DT              /* 5ms */
#define MOTOR_TAU 0.08f         /* 电机一阶时间常数 */
#define MOTOR_KV  2.0f          /* 满PWM对应稳态轮速 m/s */
#define CPR   1560.0f
#define WHEEL 0.065f
#define TRACK 0.16f

/* ---- 虚拟车物理状态 ---- */
typedef struct {
    float vl, vr;        /* 实际轮速 */
    float x, y, th;      /* 真实位姿 */
    float cnt_l, cnt_r;  /* 编码器累计(浮点累计后取整) */
} plant_t;

static void plant_step(plant_t *p, float duty_l, float duty_r)
{
    /* 一阶电机模型 + 小死区 */
    float tl = fabsf(duty_l) < 0.03f ? 0 : duty_l * MOTOR_KV;
    float tr = fabsf(duty_r) < 0.03f ? 0 : duty_r * MOTOR_KV;
    p->vl += (tl - p->vl) * DT / MOTOR_TAU;
    p->vr += (tr - p->vr) * DT / MOTOR_TAU;

    float v = 0.5f * (p->vl + p->vr);
    float w = (p->vr - p->vl) / TRACK;
    p->th += w * DT;
    p->x += v * cosf(p->th) * DT;
    p->y += v * sinf(p->th) * DT;
    p->cnt_l += p->vl * DT / (3.14159265f * WHEEL) * CPR;
    p->cnt_r += p->vr * DT / (3.14159265f * WHEEL) * CPR;
}

/* ---- 虚拟赛道: y = A*sin(k*x) 的黑线, 计算车下8路灰度 ---- */
static float track_line_y(float x) { return 0.35f * sinf(2.0f * x); }

static void fake_gray(const plant_t *p, gray8_t *g)
{
    /* 传感器排在车头前 8cm, 横向 ±7cm */
    uint16_t raw[8];
    float sx = p->x + 0.08f * cosf(p->th);
    float sy = p->y + 0.08f * sinf(p->th);
    for (int i = 0; i < 8; i++) {
        /* index0 = 最左传感器(车体左侧, th=0 时 +y 方向) */
        float off = -(i - 3.5f) / 3.5f * 0.07f;
        float px = sx - off * sinf(p->th);
        float py = sy + off * cosf(p->th);
        float d = fabsf(py - track_line_y(px));       /* 到线距离 */
        /* 线宽2cm: 距离<1cm 全黑, 1..2cm 渐变 */
        float dark = d < 0.01f ? 1.0f : (d < 0.02f ? (0.02f - d) / 0.01f : 0.0f);
        raw[i] = (uint16_t)(3500 - dark * 3000);      /* 白3500 黑500 */
    }
    gray8_update_analog(g, raw);
}

static uint32_t s_ms = 0;

int main(void)
{
    int pass = 1;

    /* ================= T1: 速度环阶跃 ================= */
    {
        chassis_t c;
        chassis_init(&c, CPR, WHEEL, TRACK);
        c.v_ramp_step = 1.0f;             /* 关斜坡, 看裸阶跃 */
        chassis_enable(&c, 1);
        chassis_set(&c, 1.0f, 0.0f);
        plant_t p = {0};
        float t_rise = -1, overshoot = 0, final_v = 0;
        for (int k = 0; k < 600; k++) {   /* 3s */
            chassis_update(&c, (int32_t)p.cnt_l, (int32_t)p.cnt_r);
            plant_step(&p, c.out_l, c.out_r);
            float v = 0.5f * (p.vl + p.vr);
            if (t_rise < 0 && v >= 0.9f) t_rise = k * DT;
            if (v - 1.0f > overshoot) overshoot = v - 1.0f;
            final_v = v;
        }
        float ss_err = fabsf(final_v - 1.0f);
        int ok = t_rise > 0 && t_rise < 0.5f && overshoot < 0.15f && ss_err < 0.02f;
        printf("[T1] speed step: rise=%.0fms overshoot=%.1f%% ss_err=%.3f  %s\n",
               t_rise * 1000, overshoot * 100, ss_err, ok ? "PASS" : "FAIL");
        pass &= ok;
    }

    /* ================= T2: S弯循迹 ================= */
    {
        chassis_t c; track_t t; gray8_t g; imu_t im; openmv_t mv; odom_t od;
        chassis_init(&c, CPR, WHEEL, TRACK);
        track_init(&t); gray8_init(&g); imu_init(&im); openmv_init(&mv); odom_init(&od);
        chassis_enable(&c, 1);
        plant_t p = {0};
        p.y = 0.03f;                       /* 起步偏线3cm */
        float max_dev = 0;
        for (int k = 0; k < 4000; k++) {   /* 20s, 5ms步 */
            s_ms += 5;
            fake_gray(&p, &g);
            imu_feed_yaw(&im, p.th * 57.2958f, s_ms);   /* 理想陀螺仪 */
            if (k % 2 == 0) {              /* 100Hz 转向环 */
                float steer = track_update(&t, &g, &im, &mv, s_ms);
                chassis_set(&c, 0.8f, steer);
            }
            chassis_update(&c, (int32_t)p.cnt_l, (int32_t)p.cnt_r);
            plant_step(&p, c.out_l, c.out_r);
            if (k > 400) {                 /* 跳过起步收敛段 */
                float dev = fabsf(p.y - track_line_y(p.x));
                if (dev > max_dev) max_dev = dev;
            }
        }
        int ok = max_dev < 0.05f && p.x > 2.0f;
        printf("[T2] S-line follow: max_dev=%.1fcm dist_x=%.2fm  %s\n",
               max_dev * 100, p.x, ok ? "PASS" : "FAIL");
        pass &= ok;
    }

    /* ================= T3: 航向保持直行 1.5m ================= */
    {
        chassis_t c; track_t t; gray8_t g; imu_t im; openmv_t mv;
        chassis_init(&c, CPR, WHEEL, TRACK);
        track_init(&t); gray8_init(&g); imu_init(&im); openmv_init(&mv);
        chassis_enable(&c, 1);
        track_use(&t, SRC_YAW);
        track_set_yaw_target(&t, 0.0f);
        plant_t p = {0};
        p.th = 0.06f;                      /* 起步歪 3.4 度 */
        while (p.x < 1.5f) {
            s_ms += 5;
            imu_feed_yaw(&im, p.th * 57.2958f, s_ms);
            float steer = track_update(&t, &g, &im, &mv, s_ms);
            chassis_set(&c, 0.7f, steer);
            chassis_update(&c, (int32_t)p.cnt_l, (int32_t)p.cnt_r);
            plant_step(&p, c.out_l, c.out_r);
            if (s_ms > 60000) break;
        }
        int ok = fabsf(p.y) < 0.03f;
        printf("[T3] dead-straight 1.5m: lateral_drift=%.1fcm  %s\n",
               p.y * 100, ok ? "PASS" : "FAIL");
        pass &= ok;
    }

    /* ================= T4: 原地转90度 ================= */
    {
        chassis_t c; track_t t; gray8_t g; imu_t im; openmv_t mv;
        chassis_init(&c, CPR, WHEEL, TRACK);
        track_init(&t); gray8_init(&g); imu_init(&im); openmv_init(&mv);
        chassis_enable(&c, 1);
        track_use(&t, SRC_YAW);
        track_set_yaw_target(&t, 90.0f);
        plant_t p = {0};
        for (int k = 0; k < 800; k++) {    /* 4s */
            s_ms += 5;
            imu_feed_yaw(&im, p.th * 57.2958f, s_ms);
            float steer = track_update(&t, &g, &im, &mv, s_ms);
            chassis_set(&c, 0.0f, steer);
            chassis_update(&c, (int32_t)p.cnt_l, (int32_t)p.cnt_r);
            plant_step(&p, c.out_l, c.out_r);
        }
        float err = fabsf(p.th * 57.2958f - 90.0f);
        int ok = err < 3.0f;
        printf("[T4] turn-to-90: final_err=%.2fdeg  %s\n", err, ok ? "PASS" : "FAIL");
        pass &= ok;
    }

    /* ===== T5: demo_mission 全流程 (循迹->十字->盲走->转弯->循迹->停车->蜂鸣) ===== */
    {
        chassis_t c; track_t t; gray8_t g; imu_t im; openmv_t mv; odom_t od; mission_t ms;
        chassis_init(&c, CPR, WHEEL, TRACK);
        track_init(&t); gray8_init(&g); imu_init(&im); openmv_init(&mv);
        odom_init(&od); mission_init(&ms);
        plant_t p = {0};

        /* 虚拟赛道: 主线(0,0)-(1,0), 十字横杆 x=1 y∈±0.1,
         * 无线区 x∈(1,2), 竖线 (2,0)-(2,-1.2) */
        imu_feed_yaw(&im, 0, ++s_ms);
        chassis_enable(&c, 1);
        mission_start(&ms, demo_mission, s_ms, &od, &g, &im, &t);

        for (int k = 0; k < 12000 && ms.running; k++) {   /* 最多60s */
            s_ms += 5;
            /* 灰度: 距离三段线的最小距离 */
            uint16_t raw[8];
            float sx = p.x + 0.08f * cosf(p.th);
            float sy = p.y + 0.08f * sinf(p.th);
            for (int i = 0; i < 8; i++) {
                float off = -(i - 3.5f) / 3.5f * 0.07f;
                float px = sx - off * sinf(p.th);
                float py = sy + off * cosf(p.th);
                float d = 1e9f;
                if (px >= 0 && px <= 1.0f) { float dd = fabsf(py); if (dd < d) d = dd; }
                if (py >= -0.1f && py <= 0.1f) { float dd = fabsf(px - 1.0f); if (dd < d) d = dd; }
                if (py <= 0.0f && py >= -1.2f) { float dd = fabsf(px - 2.0f); if (dd < d) d = dd; }
                float dark = d < 0.01f ? 1.0f : (d < 0.02f ? (0.02f - d) / 0.01f : 0.0f);
                raw[i] = (uint16_t)(3500 - dark * 3000);
            }
            if (k % 2 == 0) gray8_update_analog(&g, raw);
            imu_feed_yaw(&im, p.th * 57.2958f, s_ms);
            if (k % 2 == 0) {
                odom_update(&od, c.enc_l.dist_m, c.enc_r.dist_m, TRACK,
                            imu_yaw(&im) * 0.0174533f, true);
                mission_update(&ms, &c, &t, &od, &g, &im, &mv, s_ms);
            }
            chassis_update(&c, (int32_t)p.cnt_l, (int32_t)p.cnt_r);
            plant_step(&p, c.out_l, c.out_r);
        }
        float ex = fabsf(p.x - 2.0f), ey = fabsf(p.y - (-0.9f));
        int ok = !ms.running && ex < 0.05f && ey < 0.05f && s_beeps >= 3;
        printf("[T5] full mission: end=(%.2f,%.2f) want(2.00,-0.90) beeps=%d  %s\n",
               p.x, p.y, s_beeps, ok ? "PASS" : "FAIL");
        pass &= ok;
    }

    /* ===== T6: 双车帧协议 回环测试 ===== */
    {
        radio_link_t A, B;
        rl_init(&A, t6_tx_a); rl_init(&B, t6_tx_b);
        s_n_ab = s_n_ba = 0;
        uint32_t now = 1000;
        int ok = 1;

        /* 1) 状态帧 A->B: 字段完整解析 */
        rl_state_t st = { 2, 500, 12345, 3, RL_FLAG_ROLE_LEADER };
        rl_send_state(&A, &st);
        for (int i = 0; i < s_n_ab; i++) rl_rx_byte(&B, s_q_ab[i], now);
        s_n_ab = 0;
        ok &= B.peer_valid && B.peer.mode == 2 && B.peer.v_mmps == 500 &&
              B.peer.odo_mm == 12345 && B.peer.checkpoint == 3 &&
              (B.peer.flags & RL_FLAG_ROLE_LEADER) && B.rx_ok == 1;

        /* 2) CRC 损坏帧被丢弃, 不污染 peer */
        rl_send_state(&A, &st);
        s_q_ab[7] ^= 0xFF;
        for (int i = 0; i < s_n_ab; i++) rl_rx_byte(&B, s_q_ab[i], now);
        s_n_ab = 0;
        ok &= B.crc_err == 1 && B.rx_ok == 1;

        /* 3) 坏帧占用了 seq, 下一好帧应计出丢帧 */
        rl_send_state(&A, &st);
        for (int i = 0; i < s_n_ab; i++) rl_rx_byte(&B, s_q_ab[i], now);
        s_n_ab = 0;
        ok &= B.lost >= 1 && B.rx_ok == 2;

        /* 4) 事件 B->A + 自动 ACK 回 B */
        now += 10;
        ok &= rl_send_event(&B, RL_EV_LANE, 42u, now);
        rl_poll(&B, now);
        for (int i = 0; i < s_n_ba; i++) rl_rx_byte(&A, s_q_ba[i], now);
        s_n_ba = 0;
        uint8_t id = 0; uint32_t arg = 0;
        ok &= rl_take_event(&A, &id, &arg) && id == RL_EV_LANE && arg == 42u;
        for (int i = 0; i < s_n_ab; i++) rl_rx_byte(&B, s_q_ab[i], now);
        s_n_ab = 0;
        ok &= B.ev_pending == 0 && B.ev_fail == 0;

        /* 5) 心跳: 500ms 超时判失联 */
        ok &= rl_link_alive(&B, now + 100, 500) && !rl_link_alive(&B, now + 2000, 500);

        printf("[T6] radio link: rx=%u crc_err=%u lost=%u ack_ok  %s\n",
               B.rx_ok, B.crc_err, B.lost, ok ? "PASS" : "FAIL");
        pass &= ok;
    }

    printf("======== %s ========\n", pass ? "ALL PASS" : "SOME FAILED");
    return pass ? 0 : 1;
}
