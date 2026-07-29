/**
 * app_telemetry.h — VOFA+ JustFloat 遥测 + 简易串口命令
 * VOFA+ 上位机选 JustFloat 引擎即可实时看 8 条曲线, 现场调参神器。
 * 串口命令(115200, 换行结尾): 也可用手机蓝牙串口 APP 发
 *   "r"        启动任务      "s"  急停
 *   "p1=1.25"  设参数(p1..p9 对应菜单参数表第3项起)
 */
#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H
#include <stdint.h>
void telemetry_tick(uint32_t now_ms);   /* 20ms 调用 */
void telemetry_rx_byte(uint8_t b);
#endif
