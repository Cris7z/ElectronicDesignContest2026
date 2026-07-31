# H2026 硬件总清单（阶段 1）

> 状态：初版盘点；只记录证据与当前状态，不分配未来 IO。  
> 盘点日期：2026-07-31  
> 代码基线：`c07a-line-tracker-preferred-20260731` / `0ef4fe0`  
> 相关未决问题：见 [`../decisions/OPEN_QUESTIONS.md`](../decisions/OPEN_QUESTIONS.md)

## 1. 状态定义

| 状态 | 含义 |
|---|---|
| `CURRENT_INSTALLED` | 已安装于当前车，属于当前循迹硬件边界。 |
| `CURRENT_PRESENT_UNUSED` | 实物在控制器/底板上，但当前循迹代码不使用。 |
| `FUTURE_CONFIRMED` | 已购或已确认用于后续 H 题阶段，当前未接入。 |
| `PENDING_IDENTIFICATION` | 型号、版本、实物状态或关键映射尚未确认。 |
| `REMOVED` | 已拔除或明确不能与当前方案并接。 |
| `OBSOLETE` | 不进入重构分支的未来实现，仅保留历史来源。 |

## 2. 证据来源

| ID | 来源 | 可信用途 |
|---|---|---|
| E-01 | 基线标签中的 [`硬件及接线清单.md`](../../硬件及接线清单.md) | 基线分支的历史硬件方案与当前循迹资源。 |
| E-02 | 基线标签中的 [`AGENTS.md`](../../AGENTS.md) | 基线代码的硬件约束。 |
| E-03 | 原工作树中未提交的 2026-07-31 接线表/`AGENTS.md` | 较新的实车记录线索；与基线冲突时不能自动采用。 |
| E-04 | `D:\BaiduNetdiskDownload\WHEELTEC C07A核心板(Ti-MSPM0G3507)附送资料` | C07A、S27F/S28A、D103A/TB6612 的厂家资料包。 |
| E-05 | `D:\BaiduNetdiskDownload\【WHEELTEC】D36A步进电机驱动附送资料` | D36A 厂家资料包。 |
| E-06 | `D:\BaiduNetdiskDownload\【正点原子】DNK230D开发板` | 正点原子 DNK230D 板级资料包。 |
| E-07 | `D:\X\ElectronicDesignContest2026\硬件购买及开源资料\113dea6ebc231725c9ab2edf696659af_720.png` 与 `tb_image_share_*.png` | 采购/产品截图：01Studio CanMV K230 套件、球杆传动件“带编码器步进电机版本 + 双路步进电机驱动”、LF04、底盘等。仅作为套件证据。 |
| E-08 | `D:\X\ElectronicDesignContest2026\硬件购买及开源资料\开源网址与地址.txt` | 现有开源资料入口；其中正点原子 K230 URL 已排除，Yahboom 灰度链接仅作候选来源。 |

## 3. 硬件总清单

| ID | 硬件 | 当前状态 | H 题用途/当前结论 | 版本或实物确认 | 本阶段结论 |
|---|---|---|---|---|---|
| H-CTRL-01 | WHEELTEC C07A / TI MSPM0G3507 | `CURRENT_INSTALLED` | 当前唯一实时控制主机。 | C07A 具体版本待看实物丝印。 | 可作为当前循迹控制器；精确原理图尚未冻结。 |
| H-CTRL-02 | S27F 模块化底板 | `CURRENT_INSTALLED` | 承载 C07A 外设和模块。 | 用户于 2026-07-31 确认 S27F；C07A 核心板版本仍另行确认。 | 后续只按 S27F 资料整理底板资源。 |
| H-POWER-01 | P03B 12/24 V→5 V / 3.3 V | `CURRENT_INSTALLED` | 当前循迹控制器、灰度模块等的逻辑电源路径；未来是否承担 K230 负载待测。 | 原理图已找到；具体持续/峰值能力待实测。 | 当前保留；K230 供电不冻结。 |
| H-UI-01 | MPU6050（H5） | `OBSOLETE` | 用户于 2026-07-31 明确弃用陀螺仪。 | 板上模块可能仍物理存在，但不作为项目硬件。 | 禁止为它新增代码、接线或 IO 占用；只保留历史来源。 |
| H-UI-02 | 外接四针 I²C OLED | `CURRENT_PRESENT_UNUSED` | 物理已接入；当前纯循迹不刷新它。 | 地址/接线已有记录，但不作为本阶段代码目标。 | 保留为现有硬件，不增加显示功能。 |
| H-UI-03 | BLS 启动键、状态 LED | `CURRENT_INSTALLED` | 当前循迹启动、停车和状态提示。 | PA18/PB9 为现有记录；电平实测记录待并入后续接线验收。 | 当前循迹边界内。 |
| H-MOTION-01 | D103A / TB6612 双路电机驱动 | `CURRENT_INSTALLED` | 当前左右 MG513XP28 驱动。 | **左右 A/B 物理映射冲突，见 Q-002。** | 模块确定，通道映射未冻结。 |
| H-MOTION-02 | MG513XP28 12 V 编码减速电机 ×2 | `CURRENT_INSTALLED` | 当前差速底盘左右轮。 | CPR、减速比、相序、堵转电流待实测。 | 当前循迹边界内；参数未冻结。 |
| H-MOTION-03 | D157B / 双 AT8236 | `REMOVED` | 先前方案的外置电机驱动。 | 已购但当前不接入。 | 不与 D103A/TB6612 并用。 |
| H-SENSOR-01 | CD4051 八路模拟灰度模块 | `CURRENT_INSTALLED` | 当前主循迹传感器。 | 基线记录：5 V，AD2/AD1/AD0/OUT 至 MSPM0；黑线高 ADC。 | 当前循迹边界内；地址逻辑、电压裕量和建立时间仍待测。 |
| H-SENSOR-02 | HiWonder LineFollower_8CH / LF04 | `REMOVED` | 旧/备选循迹传感器。 | 与已占用的 U3 灰度线束不兼容。 | 不进入当前循迹接线或代码。 |
| H-MOTION-04 | D36A V1.1 步进电机驱动器 | `FUTURE_CONFIRMED` | 球杆执行器的 `STEP/DIR/EN` 驱动。 | 用户依据实物正反面照片于 2026-07-31 确认 V1.1；厂家资料说明 V1.1 相对 V1.0 增加输入 ESD 与电源 TVS。 | 仅整理资料；拨码档位、电机额定电流和台架安全态待确认。 |
| H-MOTION-05 | 42 型步进电机 + MS42CG 编码器 | `FUTURE_CONFIRMED` | 球杆角度闭环。 | 补充资料提供 MS42CG V2.0 手册；实物标签、方向和线束仍待核验。 | 仅整理资料；编码器优先按 3.3 V 约束复核。 |
| H-MECH-01 | 32 cm × 24 cm 三轮差速底盘 | `CURRENT_INSTALLED` | 当前车体。 | 几何边界和实际轮距待最终实测。 | 当前循迹边界内。 |
| H-MECH-02 | 25 cm PPR 管、约 1 cm 钢球、铰链与传动 | `FUTURE_CONFIRMED` | H 题球杆机械系统。 | 机构尺寸和装配状态待记录。 | 仅做资料与机械清单，不做控制实现。 |
| H-VISION-01 | 01Studio CanMV K230 | `FUTURE_CONFIRMED` | 后续球检测与图传。 | 用户于 2026-07-31 确认使用 01Studio；先前正点原子订单截图不作为最终选型依据。 | 后续只按 01Studio 对应资料整理；不写 K230 代码。 |
| H-VISION-02 | GC2093 摄像头、24P 排线及 K230 附件 | `FUTURE_CONFIRMED` | K230 图像输入与装车。 | 以最终 01Studio 套装实物标签为准。 | 随 K230 技术文档整理，当前不接入。 |
| H-POWER-02 | 3S/12 V 电池、总开关、保险、星形分配 | `PENDING_IDENTIFICATION` | 整车动力与安全供电。 | 容量、C 倍率、保险、线规和峰值电流未冻结。 | 不能给未来功能作供电承诺。 |
| H-OPS-01 | 2.4 GHz AP、场外笔记本、VLC/OBS | `PENDING_IDENTIFICATION` | 后续图传、录像、回放。 | AP 方案和实际可用设备待确认。 | 当前纯循迹不依赖。 |

## 4. 当前循迹硬件边界

当前代码只能依赖以下硬件类别：C07A/MSPM0G3507、实际模块化底板、D103A/TB6612、两侧 MG513XP28、CD4051、BLS、LED，以及已经实际占用底板资源的 OLED/调试接口。MPU6050 即使物理存在也属于弃用硬件。

以下硬件在当前分支必须保持“有资料、无实现、无正式新接线”：D36A、MS42CG、42 型步进电机、K230、GC2093、无线图传、电池升级和球杆机构。

## 5. 本阶段不做的事

- 不根据旧文档为 D36A、MS42CG 或 K230 分配 MSPM0 IO；
- 不把 K230D BOX 当作 CanMV K230，或反过来；
- 不解决 TB6612 左右通道冲突后再改循迹代码；
- 不把厂家资料整包复制进 Git；
- 不删除旧模块或改写纯循迹代码。
