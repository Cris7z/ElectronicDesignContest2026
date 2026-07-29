/** app_globals.h — 全局对象(菜单/遥测/任务共享) */
#ifndef APP_GLOBALS_H
#define APP_GLOBALS_H

#include "../control/chassis.h"
#include "../control/track_ctrl.h"
#include "../control/odometry.h"
#include "../sensors/gray8.h"
#include "../sensors/imu_jy61p.h"
#include "../sensors/openmv_link.h"
#include "../comm/radio_link.h"
#include "app_mission.h"

extern chassis_t g_chassis;
extern track_t   g_track;
extern odom_t    g_odom;
extern gray8_t   g_gray;
extern imu_t     g_imu;
extern openmv_t  g_openmv;
extern mission_t g_mission;
extern radio_link_t g_radio;
extern float     g_v_cruise;

#endif
