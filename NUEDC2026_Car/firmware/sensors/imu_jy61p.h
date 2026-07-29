/**
 * imu_jy61p.h — 维特智能 JY61P/JY901 串口协议解析(0x55 帧)
 * 也兼容 MPU6050+外部解算方案: 只要往 imu_feed_yaw() 里灌 yaw 即可。
 * yaw 输出为连续角(去 ±180° 跳变), 单位度。
 */
#ifndef IMU_JY61P_H
#define IMU_JY61P_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    float roll, pitch, yaw;    /* 原始欧拉角(度) */
    float yaw_cont;            /* 连续化 yaw(过 ±180 不跳变) */
    float yaw_offset;          /* 上电/任务起点清零用 */
    float gz_dps;              /* z 轴角速度 度/秒 */
    uint32_t frames;           /* 收到的有效帧数(用于判活) */
    uint32_t last_ms;          /* 最近一帧时间戳 */

    /* 解析状态机 */
    uint8_t buf[11];
    uint8_t idx;
    float _prev_yaw;
    bool _first;
} imu_t;

void  imu_init(imu_t *m);
void  imu_rx_byte(imu_t *m, uint8_t b, uint32_t now_ms);  /* UART 中断里逐字节喂 */
void  imu_feed_yaw(imu_t *m, float yaw_deg, uint32_t now_ms); /* 备用方案入口 */
void  imu_zero_yaw(imu_t *m);                             /* 当前朝向清零 */
float imu_yaw(const imu_t *m);                            /* 相对起点的连续 yaw */
bool  imu_alive(const imu_t *m, uint32_t now_ms);         /* 200ms 内有帧 */

#endif
