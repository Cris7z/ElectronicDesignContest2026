/**
 * app_menu.h — 四键 OLED 调参菜单 + flash 掉电保存
 * KEY1: 上一项  KEY2: 下一项  KEY3: 值-(长按快调)  KEY4: 值+(长按快调)
 * KEY1 长按: 保存到 flash   KEY2 长按: 进入/退出编辑步进切换(x1/x10)
 * 特殊项: [RUN] 短按 KEY3/4 启动/急停任务; [CAL] 灰度校准
 */
#ifndef APP_MENU_H
#define APP_MENU_H

#include <stdint.h>

typedef struct {
    const char *name;
    float *val;
    float step;
    float min, max;
} menu_item_t;

void app_menu_init(void);
void app_menu_tick(uint32_t now_ms);   /* 20ms 调用 */

/* 由 main.c 提供的动作钩子 */
void menu_action_run(void);
void menu_action_stop(void);
void menu_action_cal_white(void);
void menu_action_cal_black(void);

#endif
