# 通信协议

## 1. OpenMV → 主控 (UART2, 115200)

帧: `0xAA 0x55 | type(1B) | payload(6B) | sum(1B) | 0x0D`  共11字节
sum = type 和 payload 共 7 字节逐字节累加取低 8 位。

### type=0x01 循迹帧 (≥50Hz 连发)
| 偏移 | 类型 | 含义 |
|---|---|---|
| 0 | i16 LE | 线偏差 ×1000, 范围 ±1000, 左负右正 |
| 2 | i16 LE | 线倾角 ×10 (度) |
| 4 | u8 | flags: bit0 有效线 / bit1 十字 / bit2 见目标 |
| 5 | u8 | 保留 |

### type=0x02 目标帧 (识别到才发)
| 偏移 | 类型 | 含义 |
|---|---|---|
| 0 | u8 | class_id, 0=无目标 |
| 1 | i16 LE | 目标中心 x (相对画面中心, 像素) |
| 3 | i16 LE | 目标中心 y |
| 5 | u8 | 保留 |

主控 150ms 收不到帧判失联, 自动降级灰度/陀螺仪 (track_ctrl.c)。

## 2. 调试口 (UART0, 115200, 蓝牙透传)

**下行遥测**: VOFA+ JustFloat, 8×float32 + 帧尾 `00 00 80 7F`, 50Hz:
`[0]左轮速 [1]右轮速 [2]目标速 [3]线误差 [4]yaw [5]PWM_L [6]PWM_R [7]任务步`

**上行命令**(文本, \n 结尾): `r` 启动 | `s` 急停 |
`p1..p8=值` → LineKp LineKd YawKp YawKd SpdKp SpdKi SpdKf V巡航

## 3. 双车协同口 (UART3, NRF24L01 透传模块)

预留 `app_on_radio_byte()` / `bsp_uart_radio_tx()`。建议帧:
`0xBB | 车号(1B) | 事件(1B) | 参数(4B) | sum | 0x0E`
事件例: 0x01 我已启动 / 0x02 我在第N个十字 / 0x03 任务完成 / 0x10 心跳(500ms)。
超时 2s 未收到心跳 → 按赛题预案(等待或单车模式)。
