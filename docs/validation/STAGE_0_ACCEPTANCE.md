# 阶段 0：架构与边界冻结验收记录

> 验收日期：2026-07-31
> 验收门：G0
> 结论：**通过，允许进入阶段 1；不允许跳过阶段 1 进入 STM32 实施。**

## 范围

本记录只验证三板架构、历史参考边界与现有 C07A 纯循迹基线；不接线、不刷写、不运行 STM32、K230、D36A 或 MS42CG。

## G0 条件与证据

| G0 条件 | 证据 | 结论 |
|---|---|---|
| 活动文档只有唯一三板架构 | [README](../../README.md)、[开发约束](../../AGENTS.md)、[硬件清单](../hardware/HARDWARE_INVENTORY.md) 和[实施计划](../PROJECT_REBUILD_PLAN.md)均定义为“C07A 循迹/UI + STM32F103RCT6 滚球闭环 + 01Studio CanMV K230 视觉/图传”。 | 通过 |
| STM32 实物型号与历史校赛工程核对 | 用户提供的实物照片并确认 `STM32F103RCT6`；历史 `Project.uvprojx` 的第 17 行为 `<Device>STM32F103RC</Device>`。历史工程仅可参考 F1 启动、UART、定时器经验。 | 通过 |
| 没有提前生成后续运行代码 | `firmware/ball_controller/` 与 `k230/ball_vision/` 均不存在；当前唯一业务代码仍是 `firmware/line_tracker/`。 | 通过 |
| 阶段 0 未改变 C07A 接线或行为 | 本阶段架构提交 `228ca53` 的文件差异不含 `firmware/line_tracker/`；[当前接线表](../hardware/CURRENT_WIRING.md)与[IO 占用表](../hardware/CURRENT_IO_OWNERSHIP.md)仍只覆盖 C07A/S27F 纯循迹基线。 | 通过 |
| C07A 基线可构建、可恢复 | `firmware/line_tracker/sim` 执行 `mingw32-make test`：`line_tracker`、`wheel_speed_pi` 两组主机测试均通过；`build_ticlang.ps1 -OutputDirectory .\\Build\\stage0_verify` 构建成功，生成 `line_tracker.hex`。 | 通过 |

## 已知但不构成 G0 放行的限制

- 尚未在官方尺寸场地完成 H-R02 的整圈、停车、计时和不脱线实车验收；不得称为赛道通过。
- STM32 开发板连接器、供电入口、正式 IO 和跨板电平仍为 Q-008；不得创建 STM32 驱动或新接线。
- D36A 拨码、限流、电机线束与上电默认失能仍为 Q-005；整车电源裕量仍为 Q-006。

## 下一步

仅进入[阶段 1：C07A 的 H-R02 基线](../PROJECT_REBUILD_PLAN.md#阶段-1完成-c07a-的-h-r02-基线)：在官方尺寸赛道完成整圈基础验证，并记录时间、停车偏差、脱线情况、故障与原始视频。
