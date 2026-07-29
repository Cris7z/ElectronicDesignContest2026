#include "app_telemetry.h"
#include "app_globals.h"
#include "app_menu.h"
#include "../bsp/bsp.h"
#include <string.h>
#include <stdlib.h>

/* ---- JustFloat: N个float + 帧尾 00 00 80 7F ---- */
void telemetry_tick(uint32_t now_ms)
{
    (void)now_ms;
    float ch[8];
    ch[0] = g_chassis.enc_l.speed_mps;
    ch[1] = g_chassis.enc_r.speed_mps;
    ch[2] = g_chassis.v_ramped;
    ch[3] = g_gray.err_lpf;
    ch[4] = imu_yaw(&g_imu);
    ch[5] = g_chassis.out_l;
    ch[6] = g_chassis.out_r;
    ch[7] = (float)g_mission.step;

    uint8_t frame[sizeof(ch) + 4];
    memcpy(frame, ch, sizeof(ch));
    frame[sizeof(ch) + 0] = 0x00;
    frame[sizeof(ch) + 1] = 0x00;
    frame[sizeof(ch) + 2] = 0x80;
    frame[sizeof(ch) + 3] = 0x7F;
    bsp_uart_debug_tx(frame, sizeof(frame));
}

/* ---- 命令解析 ---- */
static char s_line[32];
static uint8_t s_len = 0;

static void exec_line(const char *cmd)
{
    if (cmd[0] == 'r') { menu_action_run(); return; }
    if (cmd[0] == 's') { menu_action_stop(); return; }
    if (cmd[0] == 'p') {
        /* p<idx>=<value>, idx 从1开始 -> 简单映射几个常用参数 */
        const char *eq = strchr(cmd, '=');
        if (!eq) return;
        int idx = atoi(cmd + 1);
        float v = (float)atof(eq + 1);
        switch (idx) {
        case 1: g_track.pid_line.kp = v; break;
        case 2: g_track.pid_line.kd = v; break;
        case 3: g_track.pid_yaw.kp = v; break;
        case 4: g_track.pid_yaw.kd = v; break;
        case 5: g_chassis.pid_l.kp = g_chassis.pid_r.kp = v; break;
        case 6: g_chassis.pid_l.ki = g_chassis.pid_r.ki = v; break;
        case 7: g_chassis.pid_l.kf = g_chassis.pid_r.kf = v; break;
        case 8: g_v_cruise = v; break;
        default: break;
        }
    }
}

void telemetry_rx_byte(uint8_t b)
{
    if (b == '\n' || b == '\r') {
        if (s_len) { s_line[s_len] = 0; exec_line(s_line); s_len = 0; }
        return;
    }
    if (s_len < sizeof(s_line) - 1) s_line[s_len++] = (char)b;
}
