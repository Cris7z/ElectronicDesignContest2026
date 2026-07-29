#include "app_menu.h"
#include "app_globals.h"
#include "../drivers/oled_ssd1306.h"
#include "../bsp/bsp.h"
#include <string.h>

#define LONG_MS 600

/* ---- 可调参数表: 加参数只需在这里加一行 ---- */
static float s_dummy;
static menu_item_t s_items[] = {
    { "RUN/STOP",  &s_dummy,            0,     0, 0 },
    { "CAL GRAY",  &s_dummy,            0,     0, 0 },
    { "V cruise",  &g_v_cruise,         0.05f, 0, 3.0f },
    { "Line Kp",   &g_track.pid_line.kp, 0.05f, 0, 10 },
    { "Line Kd",   &g_track.pid_line.kd, 0.2f,  0, 50 },
    { "Yaw Kp",    &g_track.pid_yaw.kp,  0.005f,0, 1 },
    { "Yaw Kd",    &g_track.pid_yaw.kd,  0.01f, 0, 2 },
    { "Spd Kp",    &g_chassis.pid_l.kp,  0.05f, 0, 5 },
    { "Spd Ki",    &g_chassis.pid_l.ki,  0.01f, 0, 2 },
    { "Spd Kf",    &g_chassis.pid_l.kf,  0.02f, 0, 2 },
    { "SlowK",     &g_chassis.slow_k,    0.05f, 0, 3 },
    { "Ramp",      &g_chassis.v_ramp_step,0.005f,0,0.2f },
};
#define N_ITEMS (int)(sizeof(s_items)/sizeof(s_items[0]))

static int s_sel = 0;
static uint8_t s_last_keys = 0;
static uint32_t s_press_ms[4];
static float s_step_mul = 1.0f;

typedef struct { float v[16]; uint32_t magic; } param_blob_t;

static void params_save(void)
{
    param_blob_t b;
    memset(&b, 0, sizeof(b));
    for (int i = 0; i < N_ITEMS && i < 16; i++) b.v[i] = *s_items[i].val;
    b.magic = 0x26A55A26u;
    bsp_flash_save(&b, sizeof(b));
    bsp_buzzer(1);
}

static void params_load(void)
{
    param_blob_t b;
    if (bsp_flash_load(&b, sizeof(b)) && b.magic == 0x26A55A26u)
        for (int i = 0; i < N_ITEMS && i < 16; i++)
            if (s_items[i].step > 0) *s_items[i].val = b.v[i];
    /* 左右速度环参数保持一致 */
    g_chassis.pid_r.kp = g_chassis.pid_l.kp;
    g_chassis.pid_r.ki = g_chassis.pid_l.ki;
    g_chassis.pid_r.kf = g_chassis.pid_l.kf;
}

void app_menu_init(void)
{
    params_load();
    oled_init();
}

static void adjust(int dir)
{
    menu_item_t *it = &s_items[s_sel];
    if (s_sel == 0) {                      /* RUN/STOP */
        if (dir > 0) menu_action_run(); else menu_action_stop();
        return;
    }
    if (s_sel == 1) {                      /* 灰度校准 */
        if (dir > 0) menu_action_cal_black(); else menu_action_cal_white();
        return;
    }
    float v = *it->val + dir * it->step * s_step_mul;
    if (v < it->min) v = it->min;
    if (v > it->max) v = it->max;
    *it->val = v;
    /* 速度环同步左右 */
    g_chassis.pid_r.kp = g_chassis.pid_l.kp;
    g_chassis.pid_r.ki = g_chassis.pid_l.ki;
    g_chassis.pid_r.kf = g_chassis.pid_l.kf;
}

void app_menu_tick(uint32_t now_ms)
{
    uint8_t keys = bsp_keys();
    uint8_t rise = keys & ~s_last_keys;
    uint8_t fall = ~keys & s_last_keys;

    for (int i = 0; i < 4; i++)
        if (rise & (1 << i)) s_press_ms[i] = now_ms;

    /* 短按(松开时判定) */
    if (fall & 0x01) {
        if (now_ms - s_press_ms[0] >= LONG_MS) params_save();
        else s_sel = (s_sel + N_ITEMS - 1) % N_ITEMS;
    }
    if (fall & 0x02) {
        if (now_ms - s_press_ms[1] >= LONG_MS)
            s_step_mul = (s_step_mul > 5.0f) ? 1.0f : 10.0f;
        else s_sel = (s_sel + 1) % N_ITEMS;
    }
    if (fall & 0x04) adjust(-1);
    if (fall & 0x08) adjust(+1);
    /* 长按 3/4 连调 */
    if ((keys & 0x04) && now_ms - s_press_ms[2] >= LONG_MS &&
        (now_ms % 100u) < 20u) adjust(-1);
    if ((keys & 0x08) && now_ms - s_press_ms[3] >= LONG_MS &&
        (now_ms % 100u) < 20u) adjust(+1);
    s_last_keys = keys;

    /* ---- 刷屏(4行) ---- */
    oled_printf(0, 0, "%c%-9s x%-2.0f   ",
                g_mission.running ? '*' : ' ',
                g_mission.running ? "RUNNING" : "READY", s_step_mul);
    menu_item_t *it = &s_items[s_sel];
    if (s_sel <= 1) oled_printf(1, 0, ">%-15s", it->name);
    else oled_printf(1, 0, ">%-8s%7.3f", it->name, *it->val);
    oled_printf(2, 0, "v%4.2f e%+5.2f %s ",
                chassis_speed(&g_chassis), g_gray.err_lpf,
                g_track.active == SRC_GRAY ? "GR" :
                g_track.active == SRC_VISION ? "MV" : "YW");
    oled_printf(3, 0, "y%+6.1f X%-2u s%-2d ",
                imu_yaw(&g_imu), g_gray.cross_events, g_mission.step);
}
