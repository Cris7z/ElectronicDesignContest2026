# ElectronicDesignContest2026

2026 年电子设计竞赛 H 题“车载平衡滚球运动控制系统”硬件工程。

> 当前阶段：架构与首版接线冻结，等待 01Studio CanMV K230 标准版和整车实物
> 联调。本文是仓库主入口；接线以本文和
> [`硬件及接线清单.md`](硬件及接线清单.md) 为准。

| 项目 | 当前冻结结果 |
|---|---|
| 实时主控 | WHEELTEC C07A + S27F/S28A 模块化控制器 / MSPM0G3507 |
| 底盘驱动 | D157B / 双 AT8236 + MG513XP28 12 V 编码电机 |
| 主循迹 | 亚博 YB-MVX05-V1.0 八路灰度，CD4051 复用数字输出 |
| 备用循迹 | LF04 四路红外；与八路灰度共用接口，二选一装车 |
| 球杆执行器 | D36A + 42 步进电机 + MS42CG A/B/PWM 编码器 |
| 视觉与图传 | 01Studio CanMV K230 标准版 + 标配 GC2093 70° |
| 控制通信 | K230 UART2 单向发送球状态给 MSPM0 |
| 无线视频 | K230 硬件 H.264/RTSP + 2.4 GHz AP/热点 + 场外 OBS/VLC |
| 控制器内置/插接模块 | P03B 稳压、MPU6050、D103A/TB6612、OLED、蓝牙 |
| 车载显示 | 模块化控制器 OLED，用于启动计时和故障状态 |

## 系统架构

```mermaid
flowchart LR
    GRAY["YB-MVX05 八路灰度\n主循迹"] --> MCU["MSPM0G3507 / 模块化控制器\n唯一实时控制主机"]
    LF["LF04 四路红外\n备用、与灰度二选一"] -.-> MCU
    IMU["MPU6050\n车体姿态"] --> MCU
    WE["左右轮编码器"] --> MCU
    BE["MS42CG A/B/PWM"] --> MCU

    CAM["GC2093 70°\n覆盖 25 cm 球杆"] --> K230["01Studio CanMV K230"]
    K230 -->|"UART2：x、vx、置信度、时戳"| MCU
    K230 -->|"H.264 / RTSP"| AP["2.4 GHz AP/热点\n路由器、手机或电脑"]
    AP --> PC["场外笔记本\nVLC/OBS 显示、录像、回放"]

    MCU --> D157["D157B / AT8236"]
    D157 --> WHEEL["左右 12 V 编码电机"]
    MCU --> D36["D36A\nSTEP/DIR/EN"]
    D36 --> BEAM["42 步进 + 25 cm 球杆"]
    MCU --> UI["OLED + 启动键"]
```

控制数据面和视频传输面逻辑隔离：MSPM0 永远不等待 RTSP。无线图传卡顿、
断开或录像失败不得进入控制闭环。

## 视觉与无线图传

<p align="center">
  <img src="docs/assets/01studio-canmv-k230-kit.png"
       alt="01Studio CanMV K230 标准版套装"
       width="680">
</p>

图示套装包括 CanMV K230 标准板、板上 GC2093、Type-C 线、
XH-1.25 转 2.54 mm 四线、散热片、亚克力底板和铜柱螺丝、16 GB MicroSD
及 Mini 读卡器。产品图来自 01Studio，原图由项目成员提供。

标配摄像头为 GC2093、70°、24P。若把 70° 暂按水平视场估算，覆盖 25 cm
球杆至少需要约 18 cm 拍摄距离；装车优先使用官方 24P/15 cm 延长排线，让
摄像头上置、K230 主板低位固定。

最终 K230 媒体结构：

```text
GC2093 / Sensor(id=2)
  ├─低分辨率 RGB/灰度通道 → 球检测 → UART2 → MSPM0
  └─1280×720 YUV420SP → 硬件 H.264 VENC → RTSP
       → K230 板载 2.4 GHz Wi-Fi STA
       → 专用路由器 → 场外 VLC/OBS
```

固件冻结为 CanMV v1.8 的 01Studio 标准板非 eMMC 镜像：

```text
CanMV_K230_01Studio_micropython_v1.8-0-gc2d1f5c_nncase_v2.11.0.img.gz
```

无屏首测使用官方 `rtsp_server.py`。`ai_rtsp.py` 默认
`display_mode="lcd"`，套装不含 LCD，不能原样作为成品运行。

## 硬件清单

| 硬件 | 状态 | 首版用途 |
|---|---|---|
| C07A（MSPM0G3507）+ S27F/S28A 模块化控制器 | 已购 | 唯一实时控制主机；底板版本按实物丝印确认 |
| P03B 12 V→5 V 稳压模块 | 控制器内含 | 给 C07A、OLED、MPU6050 和循迹传感器供电 |
| MPU6050 模块 | 控制器内含 | 保留，PA0/PA1 I²C、PA7 INT |
| OLED | 控制器内含 | 启动计时、参数和故障显示 |
| D103A/TB6612 双路驱动模块 | 控制器内含 | 已登记；首版拔下，避免与 D157B/D36A 占脚冲突 |
| 主从一体蓝牙模块 | 控制器内含 | 已登记；K230 占用 PB7 时拔下 |
| D157B 双 AT8236 | 已购 | 左右轮驱动 |
| 32 cm × 24 cm 三轮底盘 | 已购 | 机械底盘；横向仅余 1 cm 合规空间 |
| MG513XP28 12 V 编码电机 ×2 | 已购 | 左右轮闭环 |
| YB-MVX05-V1.0 八路灰度 | 已购 | 主循迹传感器 |
| LF04 四路红外 | 已购 | 备用循迹；与八路灰度不能同时接 U3 |
| D36A | **型号已确定** | 球杆 STEP/DIR/EN 驱动 |
| 42 步进 + MS42CG 编码器 | 已购 | 球杆角度闭环 |
| 25 cm PPR 管、钢球、铰链和传动件 | 已有/已购 | 球杆机械系统 |
| 01Studio CanMV K230 标准版 1 GB 套装 | 拟购 | 球检测与 RTSP；容量须压力测试 |
| K230 5 V 电源 | 待负载测试 | 先用 Type-C；P03B 通过压力测试可共用，否则另加 ≥2 A 降压 |
| 2.4 GHz AP/热点 | 可选 | 可用现有路由器、手机/电脑热点或 K230 AP |

完整 BOM 状态和限制见 [硬件及接线清单](硬件及接线清单.md)。

## 供电

```mermaid
flowchart TD
    BAT["3S/12 V 动力电池\n容量待峰值电流核算"] --> SW["总开关 + 保险"]
    SW --> STAR["星形电源分配"]
    STAR --> M12["12 V → D157B → 左右轮"]
    STAR --> S12["12 V → D36A → 42 步进"]
    STAR --> P03["P03B：12 V → 5 V"]
    P03 --> C5["模块化控制器 + OLED + MPU6050\n+ 当前循迹传感器"]
    STAR --> K5["K230 5 V\nP03B 验证通过后共用，否则独立降压"]
    K5 --> K230P["Type-C 或 40Pin 5V/GND → K230"]
```

- 台架首次点亮 K230 使用 Type-C。正式装车前对 P03B 做满载纹波和温升测试；
  通过后可共用，未通过再增加独立 5 V/≥2 A 降压，不预先重复采购。
- P03B 与 D157B 的 5 V 输出不得并联；默认 P03B 供逻辑，D157B 只驱动电机。
- K230 要求严格 5 V；不得从 UART 4P 座的 3V3 给主板供电。
- 所有信号必须共地，电机/步进动力回流不得穿过 K230 或 C07A 地线。
- 电池、保险和线径按轮堵转、步进锁止及 K230 推流峰值核算，并留至少 30%
  余量。

## 关键接线

### 底盘、循迹和人机接口

| 功能 | MSPM0 引脚/资源 | 对端与限制 |
|---|---|---|
| 左电机 AIN1/AIN2 | PB2 / PB3，TIMG6_CCP0/1 | D157B；禁止照搬旧 TIMA1 配置 |
| 右电机 BIN1/BIN2 | PA8 / PA9，TIMA0 两通道 | D157B |
| 左轮编码器 A/B | PA25 / PA26 | GPIO 中断软件正交 |
| 右轮编码器 A/B | PB20 / PB24 | GPIO 中断软件正交 |
| 八路灰度 OUT | PB17 输入 | U3 Pin 3，读取当前选中的数字通道 |
| 八路灰度 AD0/AD1/AD2 | PB16 / PA12 / PA27 输出 | U3 Pin 4/5/6，约 100 µs 后采样 OUT |
| 八路灰度供电 | S28A/S27F 5 V + U3 GND | U3 Pin 2 是 3.3 V，不能误作模块 5 V |
| LF04 备用 O1/O2/O3/O4 | PB17 / PB16 / PA12 / PA27 | 使用同一组 U3 GPIO，和八路灰度二选一 |
| OLED SCL/SDA/RST/DC | PA28 / PA31 / PB14 / PB15 | C07A 板载 OLED |
| 启动键 BLS | PA18 | 消抖；长按急停 |
| 状态 LED | PB9 | 运行/故障指示 |
| 5 ms 控制节拍 | TIMG7 | TIMG6 已给左电机 |
| USB 调试 UART0 | PA10 TX / PA11 RX | 115200 |
| SWD | PA19 / PA20 | 下载和调试 |

### D36A 与球杆编码器

| 信号 | MSPM0 端 | 对端 | 注意 |
|---|---|---|---|
| STEP | PA24 / TIMA1_CCP1 | D36A ST1 | 硬件脉冲 |
| DIR | PA13 GPIO | D36A DIR1 | 首次低速确认方向 |
| EN | PA22 GPIO | D36A EN1 | 外部上/下拉保证 MCU 复位时失能 |
| Encoder A | PB18 GPIO | MS42CG A | C07A V1.1 引出，双边沿软件正交 |
| Encoder B | PB19 GPIO | MS42CG B | C07A V1.1 引出，双边沿软件正交 |
| Encoder PWM | PA14 / TIMG12_CCP0 | MS42CG PWM | 双边沿捕获和断线超时 |
| Encoder Z | 暂不接 | MS42CG Z | 首版非必要 |
| Encoder VCC/GND | 3.3 V / GND | VCC/GND | **严禁 5 V** |

MPU6050 保留原装连接：PA0=SDA、PA1=SCL、PA7=INT。厂商 D36A 示例把
PA0/PA1 用作硬件 QEI，不能照搬；本项目把 A/B 移到 C07A V1.1 新引出的
PB18/PB19。PA14、PA13 同时连到 D103A/TB6612 插座，PA22 同时引到
CCD/J3-CN2，因此首版拔下 D103A，CCD 口保持空置。

### K230 到 MSPM0

| CanMV K230 | C07A/MSPM0 | 说明 |
|---|---|---|
| UART2 TX / GPIO11 | PB7 / UART1_RX | 球状态，115200 8N1 |
| GND | GND | 必须共地 |
| UART2 RX / GPIO12 | 首版不接 | 单向传输 |
| 3V3 | 不接 | 不并联两板电源 |

K230 背面 4P 座按板端丝印核对：
`GND / 3V3 / IO12-RX2 / IO11-TX2`，不得根据转接线颜色猜信号。PB7 与 C07A
板载蓝牙共用，接 K230 前必须物理断开蓝牙 TX。

## 机械冻结

- 整车外廓必须通过 35 cm × 25 cm 检查框。
- 32 cm × 24 cm 底盘横向每侧只有约 5 mm 余量，支架、轮毂和线束全部计入。
- 球杆沿车身纵向、居中布置，降低弯道横向加速度沿杆推动钢球的影响。
- 电池、K230 和步进电机尽量低位安装；仅摄像头上置。
- C07A OLED 是小车启动计时显示；K230 不安装显示屏。

## 上电与联调顺序

1. 检查所有电源支路、极性、保险和共地，动力支路先断开。
2. 点亮模块化控制器、OLED、MPU6050 和八路灰度，确认启动键与 5 ms 节拍。
3. 架空验证 D157B 四输入、左右电机方向和编码器符号。
4. 不接步进动力，用示波器确认 D36A `STEP/DIR/EN` 和复位失能状态。
5. 接 3.3 V 编码器，验证 PB18/PB19 软件正交和绝对 PWM 后再低速接入步进电机。
6. K230 先跑摄像头、UART2，再跑无屏 H.264/RTSP。
7. 静止台架完成球的 `0 → +5 cm → -5 cm`，再装车低速循迹。
8. 最终连续两小时运行球检测、UART、H.264/RTSP 和 OBS 录像。

## 软件与验证状态

| 验证项 | 当前结果 |
|---|---|
| Python `ball_tracker.py` 语法检查 | PASS |
| 原通用小车 PC 仿真 6 项 | PASS |
| H1 球状态帧、CRC 和超时 | PASS |
| H2 YB-MVX05 八路译码与 LF04 备用 | PASS |
| H3 球位置外环模型 | PASS，最终误差约 0.2 mm |
| 真车 D157B/D36A/K230 联调 | 待实物 |

当前代码入口：

- [MSPM0 H 题模块](NUEDC2026_Car/firmware/h2026/)
- [K230 球检测与到货流程](NUEDC2026_Car/k230/)
- [PC 仿真](NUEDC2026_Car/sim/)
- [详细架构、冲突和验收门](NUEDC2026_Car/docs/H2026_ARCHITECTURE.md)

## 仓库结构

```text
ElectronicDesignContest2026/
├─ README.md                         项目主入口
├─ 硬件及接线清单.md                完整 BOM、供电和接线表
├─ docs/assets/                      README 使用的项目图片
├─ NUEDC2026_Car/
│  ├─ firmware/                      MSPM0 固件与 H 题模块
│  ├─ k230/                          K230 视觉、UART 和到货流程
│  ├─ sim/                           PC 回归仿真
│  └─ docs/                          架构、协议和调参文档
├─ .gitignore                        排除本机环境、原始资料和生成物
└─ .gitattributes                    文本与二进制属性
```

CCS 本机工作区、题目原文、采购表与截图、厂商资料副本、第三方参考工程、
旧方案、`tmp/`、编译产物和本地工具状态均不提交。所用器件与开源资料通过
README/文档中的型号、用途和原始链接追溯。

## 当前未解除的硬件风险

1. 01Studio K230 标准版尚未到台架，无线图传目前只有官方资料依据。
2. D36A 的细分、限流、EN 有效极性和复位失能仍须实测。
3. C07A 是否为 V1.1、底板是 S27F 还是 S28A，以及电机 CPR/减速比/堵转电流尚须按实物登记。
4. 标配 GC2093 的实际钢球像素数、视场和安装高度尚未验证。
5. 24 cm 底盘装车后存在超宽风险。

## 官方资料

- [01Studio CanMV K230 教程目录](https://wiki.01studio.cc/docs/canmv_k230/)
- [CanMV K230 标准版参数](https://wiki.01studio.cc/docs/canmv_k230/intro/canmv_k230/)
- [GC2093 与延长排线](https://wiki.01studio.cc/docs/canmv_k230/intro/module/)
- [UART2 GPIO11/GPIO12](https://wiki.01studio.cc/docs/canmv_k230/basic_examples/uart/)
- [K230 供电方式](https://wiki.01studio.cc/docs/canmv_k230/getting_start/power_supply/)
- [CanMV v1.8 发布与 01Studio 镜像](https://github.com/kendryte/canmv_k230/releases/tag/v1.8)
- [无屏 H.264/RTSP 示例](https://github.com/kendryte/canmv_k230/blob/v1.8/resources/examples/02-Media/rtsp_server.py)
- [01Studio K230 资源入口](https://github.com/01studio-lab/K230_Resource)

## 资料与发布边界

本仓库当前为私人硬件开发仓库。第三方厂商资料和参考工程各自保留原许可证；
在公开发布前必须重新检查许可证、个人路径、采购截图和所有生成物。
