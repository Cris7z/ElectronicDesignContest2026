/**
 * bsp_config.h — 整车参数与功能开关(改这里, 不用翻代码)
 */
#ifndef BSP_CONFIG_H
#define BSP_CONFIG_H

/* ===== 机械参数 — 必须按自己的车实测修改! =====
 * 注意: 编码器方案是 GPIO 中断 2 倍频(A/B 相各上升沿),
 * 每转计数 = 编码器线数 * 2 * 减速比。13线*2*30 = 780。
 * (若换成 4 倍频或 QEI 方案, 记得同步改这里) */
#define CFG_COUNTS_PER_REV   780.0f   /* 输出轴每转计数: 13线*2倍频*30减速比 */
#define CFG_WHEEL_D_M        0.065f   /* 轮径 65mm */
#define CFG_TRACK_W_M        0.16f    /* 轮距 160mm */

/* ===== 方向修正 — 上电自检时不对就翻这里的符号(+1/-1) =====
 * 自检顺序见 docs/TUNING.md 第 0 节: 手推前进, 两个编码器计数都应递增;
 * 之后开环给正 duty, 两轮都应向前转。 */
#define CFG_ENC_L_SIGN       (+1)
#define CFG_ENC_R_SIGN       (-1)     /* 右编码器镜像安装, 默认取负 */
#define CFG_MOTOR_L_SIGN     (+1)
#define CFG_MOTOR_R_SIGN     (+1)

/* ===== 灰度传感器 ===== */
#define CFG_GRAY_ANALOG      1        /* 1=模拟8通道ADC, 0=数字8bit */

/* ===== 默认速度 ===== */
#define CFG_V_CRUISE         0.8f     /* 巡航 m/s, 调稳后再往上加 */

/* ===== 遥测 ===== */
#define CFG_VOFA_HZ          50       /* JustFloat 发送频率 */

/* ===== MSPM0 外设规划(与 car_mspm0g3507.syscfg 对应) =====
 * PWM:    TIMG0 CC0/CC1 20kHz -> TB6612 PWMA/PWMB (PA12/PA13)
 * 方向脚: GPIO_MOTOR 组 AIN1 AIN2 BIN1 BIN2 (同一端口, SysConfig 自动分配)
 * 编码器: GPIO 上升沿中断软件解码(G3507 只有 TIMG8 一个硬件 QEI, 不够两轮):
 *         GPIO_ENC_L 组 A/B 在 PORTA, GPIO_ENC_R 组 A/B 在 PORTB,
 *         共用 GROUP1 中断
 * ADC0:   8通道序列 MEM0..7 -> 灰度(raw[0]=最左..raw[7]=最右, 接 CH0..CH7)
 * UART0:  115200 调试/蓝牙/VOFA (PA10 TX / PA11 RX, LaunchPad 背板直通)
 * UART1:  9600 JY61P(默认波特率; 若模块改过, 在 SysConfig 里同步改)
 * UART2:  115200 OpenMV
 * UART3:  115200 NRF24L01透传板/图传
 * I2C1:   400kHz OLED SSD1306 (PB2 SCL / PB3 SDA)
 * GPIO:   GPIO_KEY 组 K1..K4(上拉输入) GPIO_MISC 组 BUZZER LED1 LED2
 * 定时器: TIMG6 5ms 周期中断(速度环)
 */

#endif
