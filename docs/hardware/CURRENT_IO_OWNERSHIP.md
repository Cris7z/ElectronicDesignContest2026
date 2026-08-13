# 当前纯循迹 IO 占用表

> 本表是 C07A V1.1/S27F 循迹任务程序的引脚所有权。`OWNED` 表示 SysConfig/程序已使用；`RESERVED_DEBUG` 表示调试占用；没有出现在表中的引脚不表示可自由分配，未来硬件进入前仍须先查原理图、底板资源表与本表。

## 0. 目标封装与显示接口边界

- 当前 SysConfig 固定为 `MSPM0G3507` / `LQFP-64(PM)`，并保留 `LP_MSPM0G3507` LaunchPad 板级绑定；它与用户确认的可运行 `c07a-line-tracker-preferred-20260731` 基线逐文件一致。除非新的实物证据表明冲突，不得改为其他封装或移除板级绑定。
- 已接入的外接四针 I2C OLED 使用 `PA0` / `PA1`；任务程序用它显示模式、时间、里程和状态，不能分给后续控制器。
- 原四线 OLED 接口使用 `PA28`（SCLK）、`PA31`（SDIN）、`PB14`（RST）和 `PB15`（DC），是 SPI 类显示接口而非 I2C。它们当前没有进入纯循迹 SysConfig，也不因“当前未用”成为可随意复用的未来 IO；若要恢复或改接该接口，必须先核验原接插件和目标屏的电平、协议与原理图网络。

## 1. MCU 引脚所有权

| 引脚 | 外设/复用 | 所有者 | 方向 | 上电安全态 | 连接对象 | 状态 |
|---|---|---|---|---|---|---|
| PA0 | I2C0 SDA | OLED 总线 | 双向开漏 | I2C 空闲 | 外接 OLED | `OWNED_DISPLAY` |
| PA1 | I2C0 SCL | OLED 总线 | 输出/开漏 | I2C 空闲 | 外接 OLED | `OWNED_DISPLAY` |
| PA10 | UART0 TX | 调试串口 | 输出 | UART 空闲 | 调试器/串口 | `RESERVED_DEBUG` |
| PA11 | UART0 RX | 调试串口 | 输入 | UART 空闲 | 调试器/串口 | `RESERVED_DEBUG` |
| PA12 | GPIO | CD4051 AD2 | 输出 | 低 | 灰度选择 bit2 | `OWNED` |
| PA13 | GPIO | TB A IN2 | 输出 | 低 | 右轮方向 | `OWNED` |
| PA14 | GPIO | TB A IN1 | 输出 | 低 | 右轮方向 | `OWNED` |
| PA16 | GPIO | TB B IN1 | 输出 | 低 | 左轮方向 | `OWNED` |
| PA17 | GPIO | TB B IN2 | 输出 | 低 | 左轮方向 | `OWNED` |
| PA18 | GPIO | BLS | 输入 | 外部 47 kΩ 下拉 | 板载旧键；新任务程序不读取 | `LEGACY_CONNECTED_UNUSED` |
| PA25 | GPIO 双沿中断 | 右编码器 A | 输入 | 外部模块决定 | 右轮编码器 | `OWNED_DIAGNOSTIC` |
| PA26 | GPIO 双沿中断 | 右编码器 B | 输入 | 外部模块决定 | 右轮编码器 | `OWNED_DIAGNOSTIC` |
| PA27 | GPIO | CD4051 AD1 | 输出 | 低 | 灰度选择 bit1 | `OWNED` |
| PB2 | TIMA1 CCP0 | TB A PWMA | 输出/PWM | PWM=0 | 右轮 PWM | `OWNED` |
| PB3 | TIMA1 CCP1 | TB B PWMB | 输出/PWM | PWM=0 | 左轮 PWM | `OWNED` |
| PB8 | GPIO | C07A V1.1 J1-1 | 未配置 | 不适用 | 外接备用 IO | `RESERVED_SPARE` |
| PB9 | GPIO | 状态 LED | 输出 | 低 | 板载 LED | `OWNED` |
| PB16 | GPIO | CD4051 AD0 | 输出 | 低 | 灰度选择 bit0 | `OWNED` |
| PB17 | ADC1_A1_4 | 灰度 OUT | 模拟输入 | 高阻 ADC | CD4051 OUT | `OWNED` |
| PB18 | GPIO | C07A V1.1 J1-2 / START OUT | 输入 | 模块外部下拉；内部无上下拉 | 外接 3.3 V 高有效启动/停车模块 | `OWNED_PENDING_ACTION_TEST` |
| PB19 | GPIO | C07A V1.1 J1-3 / MODE OUT | 输入 | 模块外部下拉；内部无上下拉 | 外接 3.3 V 高有效模式模块 | `OWNED_PENDING_ACTION_TEST` |
| PB20 | GPIO 双沿中断 | 左编码器 A | 输入 | 外部模块决定 | 左轮编码器 | `OWNED_DIAGNOSTIC` |
| PB24 | GPIO 双沿中断 | 左编码器 B | 输入 | 外部模块决定 | 左轮编码器 | `OWNED_DIAGNOSTIC` |

## 2. 外设所有权与时序

| 外设 | 分配 | 作用 | 约束 |
|---|---|---|---|
| TIMA1 | CCP0=右轮 A，CCP1=左轮 B | 双路 PWM | 定时器输出有效高时间为 `(period - compare) / period`；直接按 compare 写占空比会反转含义。 |
| TIMG7 | 5 ms 周期中断 | 仅置控制 tick/超时计数 | 控制逻辑不在 ISR 中运行；未消费 tick 视为 overrun 并故障停车。 |
| ADC1 | A1_4/PB17 | 灰度单通道采样 | 单帧扫描预算 `≤ 1.2 ms`。 |
| GPIO 中断 | PA25/PA26、PB20/PB24 | 两轮四沿正交计数 | 当前为诊断/影子速度环；速度修正开关在基线中关闭。 |
| I2C0 | PA0/PA1，400 kHz | SSD1306 接口 | 异步刷新模式、时间、里程和状态。 |
| UART0 | PA10/PA11，115200 | 调试输出 | 不得把未来 K230 业务通信直接塞入该接口。 |
| 主 Flash 末扇区 | `0x1FC00…0x1FFFF`，1 KiB | 灰度标定记录 | 不含可执行代码；正式镜像每次烧录恢复 CRC=`0x74D9` 的冻结标定，MODE 现场标定仅在运行时按需覆盖。 |

## 3. 明确排除的 IO 需求

| 硬件/功能 | 当前占用 | 规则 |
|---|---|---|
| D36A / MS42CG | RCT6 专属 | 现有滚球台架线束见第 4 节；不占用 C07A IO。 |
| 01Studio CanMV K230 / 摄像头 / 图传 | RCT6 PA10/PA9 双向 UART | K230 GPIO3/TX→PA10/RX；RCT6 PA9/TX→K230 GPIO4/RX；不接 C07A。 |
| STM32F103RCT6 | RCT6 专属 | 当前滚球实时控制器；无正式 C07A↔STM32 引脚分配。 |
| 圈数、终点、赛题计时显示 | 无新增 IO | 当前代码基底不实现；后续经 H-R02 单独切片审批。 |

## 4. STM32F103RCT6 滚球台架 IO

| 引脚 | 外设/模式 | 对端 | 所有权 |
|---|---|---|---|
| PB6 | TIM4_CH1 | D36A ST1 | `OWNED` |
| PB8 | GPIO 输出 | D36A EN1 | `OWNED` |
| PB9 | GPIO 输出 | D36A DIR1 | `OWNED` |
| PA0 / PA1 | TIM2_CH1 / CH2 | MS42CG A / B | `OWNED` |
| PA6 | TIM3_CH1 输入捕获 | MS42CG PWM | `OWNED` |
| PA12 | EXTI12 | MS42CG Z | `OWNED` |
| PC8 | GPIO 输入，上拉 | 板载 K3 / 备用 RTSP | `OWNED_MODE_INPUT` |
| PC9 | GPIO 输入，上拉 | 板载 K4 / H-R03 启动中止 | `OWNED_Q3_INPUT` |
| PA9 | USART1_TX | K230 GPIO4/UART1_RX + 板载 CH340 RX | `OWNED_MODE_TX_SHARED_DEBUG` |
| PA10 | USART1_RX | K230 GPIO3/UART1_TX + 板载 CH340 TX 网络 | `OWNED_VISION_RX_ACCEPTED_RISK` |
