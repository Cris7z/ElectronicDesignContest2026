/**
 * openmv_link.h — OpenMV/K210 串口帧协议解析(与 openmv 目录下脚本配套)
 * 帧格式(小端): 0xAA 0x55 | type(1) | payload(6) | sum(1) | 0x0D
 *   type=0x01 循迹帧: int16 err(-1000..1000, 线偏差*1000), int16 angle(-900..900, 0.1度), u8 flags, u8 保留
 *              flags bit0=有效线 bit1=看到十字 bit2=看到目标
 *   type=0x02 目标帧: u8 class_id, int16 cx, int16 cy, u8 保留
 * sum = type~payload 逐字节累加取低8位
 */
#ifndef OPENMV_LINK_H
#define OPENMV_LINK_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    /* 循迹结果 */
    float line_err;      /* [-1,1] */
    float line_angle_deg;
    bool  line_valid;
    bool  see_cross;
    /* 目标识别结果 */
    uint8_t target_class;   /* 0=无 */
    int16_t target_cx, target_cy;
    uint32_t frames;
    uint32_t last_ms;

    uint8_t buf[12];
    uint8_t idx;
} openmv_t;

void openmv_init(openmv_t *o);
void openmv_rx_byte(openmv_t *o, uint8_t b, uint32_t now_ms);
bool openmv_alive(const openmv_t *o, uint32_t now_ms);  /* 150ms 判活 */

#endif
