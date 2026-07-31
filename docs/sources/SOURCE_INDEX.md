# H2026 来源索引

> 目的：将厂家资料、官方在线文档和赛题文件登记为可追溯来源；Git 只保存整理后的文字与表格，**不镜像**体积大的资料包或受版权保护的 PDF。
>
> 更新日期：2026-07-31。状态中的“已阅读”表示已提取与当前阶段有关的内容，并不代表所有页都已经完成逐页整理。
>
> 已入手的关键本地 PDF 的大小与 SHA-256 见 [`FILE_FINGERPRINTS.md`](FILE_FINGERPRINTS.md)。

| ID | 来源 | 类型 / 版本 | 原始位置 | 状态 | 对应整理文档 | 备注 |
|---|---|---|---|---|---|---|
| S-C07-01 | WHEELTEC C07A 厂家资料包 | 本地资料包 | `D:\BaiduNetdiskDownload\WHEELTEC C07A核心板(Ti-MSPM0G3507)附送资料` | 分批阅读中 | [`../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md`](../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md) | 当前只整理纯循迹相关内容。 |
| S-C07-02 | S27F 底板原理图 | PDF | `...\7.底板相关资料\2.S27F底板资料\1.S27F底板原理图.pdf` | 已阅读 | [`../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md`](../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md) | S27F 已由用户确认。 |
| S-C07-03 | C07A 搭配 S27F 底板资源分配表 | PDF，2026-07-14 | `...\7.底板相关资料\2.S27F底板资料\2.C07A搭配S27F底板资源分配表(2026.07.14).pdf` | 已阅读 | [`../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md`](../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md) | 只提取当前循迹已涉及资源。 |
| S-C07-04 | D103A / TB6612FNG 原理图 | PDF | `...\2.D103A_TB6612精简版模块资料\1.TB6612FNG模块原理图（D103A）.pdf` | 已阅读 | [`../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md`](../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md) | 左右轮映射以已关闭的 Q-002 为准。 |
| S-C07-05 | C07A 核心板 V1.0 / V1.1 原理图及更新记录 | PDF / TXT / PNG | `...\3.原理图\C07A核心板原理图_V1.0（MSPM0G3507）.pdf`、`...V1.1...pdf`、`C07A硬件更新内容记录.txt` | 已阅读版本差异 | [`../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md`](../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md) | 核心板已由两个底部按键与厂商识别图确认是 V1.1。 |
| S-C07-06 | P03B 5 V / 3.3 V 稳压模块资料包入口 | TXT，百度网盘链接 | `...\底板所用其他模块资料包\1.P03B电源模块（12V转5V降压）资料包.txt` | 原件待补 | [`../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md`](../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md) | 本机没有该模块的原理图或参数资料，不能推定负载能力。 |
| S-C07-07 | MPU6050 模块资料包入口及芯片手册 | TXT / PDF | `...\底板所用其他模块资料包\3.MPU6050模块资料包.txt`；`...\5.芯片数据手册\MPU6050` | 模块原件待补 | [`../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md`](../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md) | 芯片手册不能替代模块电路资料。 |
| S-SENSOR-01 | Yahboom 八路灰度巡线模块页面 | 在线页面 | `https://www.yahboom.com/study_module/8-GS` | 候选来源，待实物核对 | [`../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md`](../hardware/C07A_S27F_LINE_TRACKER_HARDWARE.md) | 页面只有图像与下载入口，不能从中冻结当前 CD4051 的针序或电气参数。 |
| S-D36-01 | D36A 驱动用户手册 | PDF，V1.5，2026-06-17 | `D:\BaiduNetdiskDownload\【WHEELTEC】D36A步进电机驱动附送资料\...\1.用户手册与使用教程\D36A驱动用户手册_V1.5_2026.6.17.pdf` | 已阅读相关页 | [`../hardware/D36A_STEPPER_DRIVER.md`](../hardware/D36A_STEPPER_DRIVER.md) | 接口、拨码、安全注意事项。 |
| S-D36-02 | D36A V1.1 原理图 | PDF，V1.1，2026-04-20 | `D:\BaiduNetdiskDownload\【WHEELTEC】D36A步进电机驱动附送资料\...\3.原理图\D36A双路步进电机驱动模块_V1.1(2026.04.20).pdf` | 已阅读相关页 | [`../hardware/D36A_STEPPER_DRIVER.md`](../hardware/D36A_STEPPER_DRIVER.md) | 后续网络表复核的原始来源。 |
| S-D36-03 | D36A 版本说明与识别图 | TXT / PNG，V1.1 资料包 | `D:\BaiduNetdiskDownload\【WHEELTEC】D36A步进电机驱动附送资料\...\3.原理图` | 已阅读 | [`../hardware/D36A_STEPPER_DRIVER.md`](../hardware/D36A_STEPPER_DRIVER.md) | V1.1 的 ESD / TVS 差异与丝印位置。 |
| S-K230-01 | CanMV K230 官方在线教程首页 | 在线文档 | `https://wiki.01studio.cc/docs/canmv_k230` | 已阅读目录 | [`../hardware/K230_VISION_MODULE.md`](../hardware/K230_VISION_MODULE.md) | 最终 K230 文档的主索引。 |
| S-K230-02 | CanMV K230 供电说明 | 在线文档 | `https://wiki.01studio.cc/docs/canmv_k230/getting_start/power_supply/` | 已阅读 | [`../hardware/K230_VISION_MODULE.md`](../hardware/K230_VISION_MODULE.md) | 5 V 供电边界。 |
| S-K230-03 | CanMV K230 UART 教程 | 在线文档 | `https://wiki.01studio.cc/docs/canmv_k230/basic_examples/uart/` | 已阅读 | [`../hardware/K230_VISION_MODULE.md`](../hardware/K230_VISION_MODULE.md) | UART1 / UART2 和终端串口占用。 |
| S-K230-04 | CanMV K230 Camera 教程 | 在线文档 | `https://wiki.01studio.cc/docs/canmv_k230/machine_vision/camera/` | 已阅读 | [`../hardware/K230_VISION_MODULE.md`](../hardware/K230_VISION_MODULE.md) | CSI2、GC2093、传感器实例。 |
| S-K230-05 | CanMV K230 在线训练模型教程 | 在线文档 | `https://wiki.01studio.cc/docs/canmv_k230/machine_vision/train/` | 已阅读 | [`../hardware/K230_VISION_MODULE.md`](../hardware/K230_VISION_MODULE.md) | K230 模型导出和部署流程。 |
| S-K230-06 | 01Studio 在线训练模型平台 | 在线服务 | `https://ai.01studio.cc` | 已登记 | [`../hardware/K230_VISION_MODULE.md`](../hardware/K230_VISION_MODULE.md) | 不把账号、Cookie、密钥、私有数据或模型成品提交至 Git。 |
| S-PUR-01 | 项目采购/产品截图 | PNG | `D:\X\ElectronicDesignContest2026\硬件购买及开源资料\113dea6ebc231725c9ab2edf696659af_720.png` 及 `tb_image_share_*.png` | 已阅读 | 硬件清单、D36A/K230 文档 | 只证明购买套件标注和配件，不能代替实物丝印、原理图或额定参数。 |
| S-H-01 | 2026 年赛题 H 题 PDF | PDF | `D:\X\ElectronicDesignContest2026\H题_车载平衡滚球运动控制系统.pdf` | **尚未阅读** | 阶段 3 待建 | 按重构计划，在硬件资料完成后逐页整理五个问题。 |

## 已排除的历史来源

| 来源 | 处理 | 原因 |
|---|---|---|
| 正点原子 DNK230D / K230D BOX 资料与订单截图 | `HISTORICAL_EXCLUDED` | 用户已确认最终板卡是 **01Studio CanMV K230**。这些资料只能保留为采购历史线索，不能用作引脚、供电、固件、相机或代码依据。 |

## 使用规则

1. 新的硬件结论必须在本表登记原始来源后，再写入技术文档。
2. 同一问题的资料与实物出现矛盾时，记录到 `docs/decisions/OPEN_QUESTIONS.md`，等待用户确认；不得自行选边。
3. 每一份原理图完成 AI 可读化后，应在其硬件文档中保留页号、网络名、信号方向和不确定项，而不是只贴截图。
