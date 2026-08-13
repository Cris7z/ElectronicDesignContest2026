# H2026 硬件总清单（结项）

> 状态：结项快照；接线仍以 `CURRENT_WIRING.md` 和 `CURRENT_IO_OWNERSHIP.md` 为唯一入口。
> 初次盘点：2026-07-31；结项整理：2026-08-13
> 正式版本：`h2026-project-final-20260813`；旧 C07A 回退基线：`c07a-line-tracker-preferred-20260731` / `0ef4fe0`
> 相关未决问题：见 [`../decisions/OPEN_QUESTIONS.md`](../decisions/OPEN_QUESTIONS.md)

## 1. 状态定义

| 状态 | 含义 |
|---|---|
| `CURRENT_INSTALLED` | 已安装并纳入当前三板系统。 |
| `CURRENT_PRESENT_UNUSED` | 实物在控制器/底板上，但当前循迹代码不使用。 |
| `FUTURE_CONFIRMED` | 已购或已确认用于后续 H 题阶段，当前未接入。 |
| `PENDING_IDENTIFICATION` | 型号、版本、实物状态或关键映射尚未确认。 |

## 2. 证据来源

| ID | 来源 | 可信用途 |
|---|---|---|
| E-04 | `D:\BaiduNetdiskDownload\WHEELTEC C07A核心板(Ti-MSPM0G3507)附送资料` | C07A、S27F、D103A/TB6612 的厂家资料包。 |
| E-05 | `D:\BaiduNetdiskDownload\【WHEELTEC】D36A步进电机驱动附送资料` | D36A 厂家资料包。 |
| E-07 | `D:\X\ElectronicDesignContest2026\硬件购买及开源资料\113dea6ebc231725c9ab2edf696659af_720.png` 与 `tb_image_share_*.png` | 采购/产品截图：01Studio CanMV K230 套件、球杆传动件、D36A 和底盘。仅作为套件证据。 |
| E-08 | `D:\X\ElectronicDesignContest2026\硬件购买及开源资料\开源网址与地址.txt` | 当前三板系统相关资料入口。 |
| E-09 | 用户于 2026-07-31 提供的控制板实物照片及型号确认；`D:\X\ElectronicDesignContest2026\reference\2024H_keil_early\Project.uvprojx` | 实物确认为 STM32F103RCT6；旧校赛 Keil 工程目标为 `STM32F103RC`，可用于核对芯片族和历史驱动结构，不作为新接线真值。 |
| E-10 | `D:\X\单片机\03-手册资料\STM32F103RCT6系统板资料（信泰微电子）(1).zip` | 信泰微电子系统板资料：原理图、BOM、尺寸图、厂家注意事项和旧 Keil 示例。摘要、矛盾和禁用项见 [`STM32F103RCT6_BOARD_REFERENCE.md`](STM32F103RCT6_BOARD_REFERENCE.md)。 |

## 3. 硬件总清单

| ID | 硬件 | 当前状态 | H 题用途/当前结论 | 版本或实物确认 | 本阶段结论 |
|---|---|---|---|---|---|
| H-CTRL-01 | WHEELTEC C07A V1.1 / TI MSPM0G3507 | `CURRENT_INSTALLED` | 负责循迹、任务按键、里程停车与 OLED。 | 两个底部按键与厂家识别图确认 V1.1。 | C07A V1.1 原理图已冻结为当前控制器资料依据。 |
| H-CTRL-02 | S27F 模块化底板 | `CURRENT_INSTALLED` | 承载 C07A 外设和模块。 | 用户于 2026-07-31 确认 S27F。 | 只按 S27F 资料整理底板资源。 |
| H-CTRL-03 | STM32F103RCT6 开发板 | `CURRENT_INSTALLED` | 负责 D36A、MS42CG 与 K230 球位置闭环。 | ST-Link 实测 Device ID=`0x10036414`、Flash=256 KiB；E-10 原理图/BOM的 `F103RE` 不能覆盖实物型号。 | 正式接线见 `CURRENT_WIRING.md`；活动源与归档 V16 一致。 |
| H-POWER-01 | P03B 12/24 V→5 V / 3.3 V | `CURRENT_INSTALLED` | 当前循迹控制器、灰度模块等的逻辑电源路径；未来是否承担 K230 负载待测。 | 原理图已找到；具体持续/峰值能力待实测。 | 当前保留；K230 供电不冻结。 |
| H-UI-02 | 外接四针 I²C OLED | `CURRENT_INSTALLED` | 显示模式、时间、里程和 WAIT/RUN/FAULT。 | 地址 `0x3C`，PA0/PA1；目标程序编译通过，显示实物效果仍待按键接线时联验。 | 保留现有接线，不得将 PA0/PA1 复用为按键。 |
| H-UI-03 | BLS、状态 LED | `CURRENT_INSTALLED` | PB9 保持状态提示；PA18/BLS 保留电气配置，但新任务程序不读取。 | PA18 外部 47 kΩ 下拉，PB9 为输出。 | 不拆除现有硬件。 |
| H-UI-04 | C07A 外接三针任务按键模块 ×2 | `CURRENT_INSTALLED` | PB18 START；PB19 MODE；PB8 保留备用。 | 模块为 VCC/OUT/G：VCC=3.3 V、G=公共 GND、OUT 高有效；内部不上下拉。实物已接，动作待复验。 | 已写入 SysConfig 和任务程序；逐键验收。见 Q-010。 |
| H-MOTION-01 | D103A / TB6612 双路电机驱动 | `CURRENT_INSTALLED` | 当前左右 MG513XP28 驱动。 | 用户已确认：左轮为 B 路、右轮为 A 路。 | 映射按纯循迹标签冻结；实车矛盾时重新打开 Q-002。 |
| H-MOTION-02 | MG513XP28 12 V 编码减速电机 ×2 | `CURRENT_INSTALLED` | 当前差速底盘左右轮。 | CPR、减速比、相序、堵转电流待实测。 | 当前循迹边界内；参数未冻结。 |
| H-SENSOR-01 | CD4051 八路模拟灰度模块 | `CURRENT_INSTALLED` | 当前主循迹传感器。 | 基线记录：5 V，AD2/AD1/AD0/OUT 至 MSPM0；黑线高 ADC。 | 当前循迹边界内；地址逻辑、电压裕量和建立时间仍待测。 |
| H-MOTION-04 | D36A V1.1 步进电机驱动器 | `CURRENT_INSTALLED` | 球杆执行器的 `STEP/DIR/EN` 驱动。 | D36A 1 s 独立动作链已实测；最终拨码与风险记录见验证日志。 | 保持当前实物接线，不为整理线束而重接。 |
| H-MOTION-05 | 42 型步进电机 + MS42CG 编码器 | `CURRENT_INSTALLED` | 球杆角度闭环。 | A/B、PWM、Z 已纳入 RCT6 工程。 | 接线与计数语义见正式 IO 表。 |
| H-MECH-01 | 32 cm × 24 cm 三轮差速底盘 | `CURRENT_INSTALLED` | 当前车体。 | 几何边界和实际轮距待最终实测。 | 当前循迹边界内。 |
| H-MECH-02 | 25 cm PPR 管、约 1 cm 钢球、铰链与传动 | `CURRENT_INSTALLED` | H 题球杆机械系统。 | 最终装配由现场实物确认。 | 软件归档不改变机械零位和安装关系。 |
| H-VISION-01 | 01Studio CanMV K230 | `CURRENT_INSTALLED` | 球检测、毫米位置、LCD 与 RTSP 图传。 | 现场可用 RTSP 基线独立冻结于 `k230-rtsp-working-20260801`。 | K230 不直接驱动电机。 |
| H-VISION-02 | GC2093 摄像头、24P 排线及 K230 附件 | `CURRENT_INSTALLED` | K230 图像输入。 | 以最终套装实物与 SD 卡内容为准。 | 模型和私密网络配置仍是本地资产，不提交 Git。 |
| H-POWER-02 | 3S/12 V 电池、总开关、保险、星形分配 | `PENDING_IDENTIFICATION` | 整车动力与安全供电。 | 容量、C 倍率、保险、线规和峰值电流未冻结。 | 不能给未来功能作供电承诺。 |
| H-OPS-01 | 2.4 GHz AP、场外笔记本、VLC/OBS | `PENDING_IDENTIFICATION` | 后续图传、录像、回放。 | AP 方案和实际可用设备待确认。 | 当前纯循迹不依赖。 |

## 4. 当前三板硬件边界

正式工程只依赖已选三板架构：C07A/MSPM0G3507 负责底盘循迹/UI，STM32F103RCT6 负责球杆实时控制，01Studio CanMV K230/GC2093 负责视觉与图传。C07A 不与 RCT6 通信，K230 不直接驱动执行器。

所有正式信号只采用 `CURRENT_WIRING.md` 和 `CURRENT_IO_OWNERSHIP.md` 中已记录的实物线束；未列出的接口不得从旧资料推断。

## 5. 结项后维护规则

- 不根据旧文档重新分配 STM32F103RCT6、D36A、MS42CG 或 K230 IO；
- 不改变已验证的 TB6612 左右通道、循迹 PD 核心和机械零位；
- 不把厂家资料整包复制进 Git；
- 构建产物可重新生成，正式镜像哈希和恢复入口保存在 `docs/FINAL_RELEASE.md`。
