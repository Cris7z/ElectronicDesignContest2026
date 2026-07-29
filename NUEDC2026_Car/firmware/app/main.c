/**
 * main.c — 主循环与中断粘合层
 * 时序设计:
 *   5ms  定时器中断: 速度环 chassis_update (硬实时)
 *   10ms 主循环:     灰度采样 + 里程计 + 任务状态机(转向环 100Hz)
 *   20ms 主循环:     菜单 + 遥测
 * 串口字节全部在中断里喂解析器, 无阻塞。
 */
#include "../bsp/bsp.h"
#include "../bsp/bsp_config.h"
#include "app_globals.h"
#include "app_menu.h"
#include "app_telemetry.h"

chassis_t g_chassis;
track_t   g_track;
odom_t    g_odom;
gray8_t   g_gray;
imu_t     g_imu;
openmv_t  g_openmv;
mission_t g_mission;
radio_link_t g_radio;
float     g_v_cruise = CFG_V_CRUISE;

/* ---------- 中断回调 ---------- */
void app_on_imu_byte(uint8_t b)    { imu_rx_byte(&g_imu, b, bsp_millis()); }
void app_on_openmv_byte(uint8_t b) { openmv_rx_byte(&g_openmv, b, bsp_millis()); }
void app_on_debug_byte(uint8_t b)  { telemetry_rx_byte(b); }
void app_on_radio_byte(uint8_t b)  { rl_rx_byte(&g_radio, b, bsp_millis()); }

void app_on_ctrl_tick(void)        /* 5ms 速度环 */
{
    chassis_update(&g_chassis, bsp_encoder_left(), bsp_encoder_right());
    bsp_motor_set(g_chassis.out_l, g_chassis.out_r);
}

/* ---------- 菜单动作 ---------- */
void menu_action_run(void)
{
    chassis_enable(&g_chassis, 1);
    mission_start(&g_mission, demo_mission, bsp_millis(),
                  &g_odom, &g_gray, &g_imu, &g_track);
}
void menu_action_stop(void) { mission_abort(&g_mission, &g_chassis); }

void menu_action_cal_white(void)
{
    uint16_t raw[8];
    bsp_gray_read_analog(raw);
    gray8_cal_white(&g_gray, raw);
    bsp_led(0, 1);
}
void menu_action_cal_black(void)
{
    uint16_t raw[8];
    bsp_gray_read_analog(raw);
    gray8_cal_black(&g_gray, raw);
    bsp_led(1, 1);
}

int main(void)
{
    bsp_init();
    chassis_init(&g_chassis, CFG_COUNTS_PER_REV, CFG_WHEEL_D_M, CFG_TRACK_W_M);
    track_init(&g_track);
    odom_init(&g_odom);
    gray8_init(&g_gray);
    imu_init(&g_imu);
    openmv_init(&g_openmv);
    mission_init(&g_mission);
    rl_init(&g_radio, bsp_uart_radio_tx);   /* 双车链路(协议见 comm/radio_link.h) */
    app_menu_init();          /* 内部会从 flash 加载参数 */

    uint32_t t10 = 0, t20 = 0;
    for (;;) {
        uint32_t now = bsp_millis();

        if (now - t10 >= 10u) {           /* 100Hz: 感知 + 决策 */
            t10 = now;
#if CFG_GRAY_ANALOG
            uint16_t raw[8];
            bsp_gray_read_analog(raw);
            gray8_update_analog(&g_gray, raw);
#else
            gray8_update_digital(&g_gray, bsp_gray_read_digital());
#endif
            odom_update(&g_odom, g_chassis.enc_l.dist_m, g_chassis.enc_r.dist_m,
                        g_chassis.track_width_m,
                        imu_yaw(&g_imu) * 0.0174533f,
                        imu_alive(&g_imu, now));
            mission_update(&g_mission, &g_chassis, &g_track, &g_odom,
                           &g_gray, &g_imu, &g_openmv, now);
        }

        if (now - t20 >= 20u) {           /* 50Hz: 人机 + 遥测 + 双车链路 */
            t20 = now;
            app_menu_tick(now);
            telemetry_tick(now);
            rl_poll(&g_radio, now);       /* 事件重发; 状态帧广播按题在任务层发 */
        }
    }
}
