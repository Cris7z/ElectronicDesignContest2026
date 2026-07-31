# 纯循迹代码迁移清单（执行前冻结）

> 基线：`c07a-line-tracker-preferred-20260731`，`0ef4fe0ac1f4fc11c8abcb3771d886cda78e2f65`。
>
> 目的：下一步只把可运行的 C07A 纯循迹实现迁到 `firmware/line_tracker/`。本清单先定义范围；实际复制、改路径、构建验证和删除旧树必须是后续独立提交。

## 1. 迁移目标

```text
firmware/line_tracker/
├─ app/main.c
├─ bsp/h2026_bsp.c
├─ bsp/h2026_bsp.h
├─ bsp/h2026_q2.syscfg
├─ core/line_tracker.c
├─ core/line_tracker.h
├─ core/wheel_speed_pi.c
├─ core/wheel_speed_pi.h
├─ sim/Makefile
├─ sim/test_line_tracker.c
├─ sim/test_wheel_speed_pi.c
└─ tools/build_ticlang.ps1
```

这是一个**代码迁移目标**，不是提前创建 D36A/K230/球杆子目录。`h2026_bsp` 名称先保留，避免在首次迁移中混入功能性重命名。

## 2. `KEEP / MOVE`：最小可构建纯循迹集

| 基线路径 | 目标路径 | 理由 |
|---|---|---|
| `NUEDC2026_Car/firmware/line_tracker/app/main.c` | `firmware/line_tracker/app/main.c` | 唯一生产入口：BLS、八路采样、PD 差速、TB6612 安全态。 |
| `NUEDC2026_Car/firmware/line_tracker/core/line_tracker.[ch]` | `firmware/line_tracker/core/` | 与硬件无关的循迹状态机/控制核心。 |
| `NUEDC2026_Car/firmware/line_tracker/core/wheel_speed_pi.[ch]` | `firmware/line_tracker/core/` | 当前已编译的影子速度诊断；主 PWM 修正开关保持关闭。 |
| `NUEDC2026_Car/firmware/line_tracker/sim/Makefile`、`test_line_tracker.c`、`test_wheel_speed_pi.c` | `firmware/line_tracker/sim/` | 主机侧控制核心回归测试。 |
| `NUEDC2026_Car/firmware/h2026_q2/bsp/h2026_bsp.[ch]` | `firmware/line_tracker/bsp/` | 当前 `main.c` 所依赖的 C07A/TB6612/CD4051/BLS BSP。 |
| `NUEDC2026_Car/firmware/h2026_q2/bsp/h2026_q2.syscfg` | `firmware/line_tracker/bsp/` | 与已冻结 IO 表一致的 MSPM0 SysConfig。 |
| `NUEDC2026_Car/firmware/line_tracker/tools/build_ticlang.ps1` | `firmware/line_tracker/tools/` | 作为构建入口；迁移时只改相对路径，不改算法。 |

## 3. `DO NOT MOVE`：有意不进入新项目的内容

| 路径/类别 | 原因 |
|---|---|
| `firmware/h2026/`、`firmware/h2026_q2/app/`、`firmware/h2026_q2/core/` | 旧完整赛题、摆球/通信/显示流程；不属于纯循迹。 |
| `k230/`、`openmv/`、`firmware/sensors/openmv_link.*` | 未来视觉/图传，尚未批准进入代码。 |
| `firmware/h2026/ball_*`、`firmware/h2026/lf04.*` | 球杆与已拔除的 LF04 历史方案。 |
| `firmware/sensors/imu_jy61p.*`、MPU6050 相关代码 | 用户已弃用陀螺仪。 |
| `firmware/comm/radio_link.*`、蓝牙相关配置 | 当前蓝牙未接入；不能占用未来接口。 |
| `app/main_deadband_sweep.c`、`main_wheel_pi_bench.c` 及即时采集脚本 | 仅一次性调试/台架入口，不是唯一生产固件；仍可从基线标签取回。 |
| 原 `h2026_q2` CCS projectspec | 文件指向旧 H2026 Q2 的 app/core，迁入会误编译旧功能；以后为纯循迹单独生成项目描述。 |
| 构建产物、`.hex/.out/.map`、SDK、缓存、临时目录 | 不进 Git。 |

## 4. 首次迁移的不可变约束

1. 只允许路径整理、构建脚本相对路径修复、无关源排除和 README 更新；不修改白黑标定、PID、占空比、极性、扫描时序或安全态。
2. 迁移后逐文件对比基线内容；唯一允许的源差异必须记录为“构建路径/包含路径”差异。
3. 至少运行主机 `sim` 两项测试；TI 工具链可用时再运行一次 MSPM0 构建。
4. 在上述验证前，不删除工作树中的任何旧源码；删除是单独、可审查的后续提交。
5. 迁移前若当前灰度模块、轮映射或 C07A/S27F 实物出现新矛盾，暂停迁移并回到硬件记录处理。

