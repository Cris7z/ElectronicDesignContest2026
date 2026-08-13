# ElectronicDesignContest2026 — H 题结项仓库

2026 年电子设计竞赛 H 题“车载平衡滚球运动控制系统”的最终工程。系统由 **C07A 循迹/UI + STM32F103RCT6 滚球闭环 + 01Studio CanMV K230 视觉/图传** 三部分组成。

当前结项分支为 `codex/h2026-final-20260813`，统一发布标签为 `h2026-project-final-20260813`。C07A 最后一次实车烧录由用户确认为正式版；旧标签继续保留用于回退。完整版本、镜像哈希与恢复入口见 [结项版本清单](docs/FINAL_RELEASE.md)。

## 从这里开始

- 赛题要求：[H2026 官方要求](docs/requirements/H2026_OFFICIAL_REQUIREMENTS.md)
- 唯一实施计划：[三板系统实施计划](docs/PROJECT_REBUILD_PLAN.md)
- 当前硬件与接线：[硬件清单](docs/hardware/HARDWARE_INVENTORY.md)、[接线表](docs/hardware/CURRENT_WIRING.md)、[IO 表](docs/hardware/CURRENT_IO_OWNERSHIP.md)
- 结项交付：[正式版本与恢复清单](docs/FINAL_RELEASE.md)、[RCT6 构建说明](firmware/ball_beam/stm32f103_rct6/RCT6_BUILD.md)
- 开发约束：[AGENTS.md](AGENTS.md)

## 构建与验证

```powershell
cd firmware/line_tracker/sim
mingw32-make test

cd ..
.\tools\build_ticlang.ps1 -OutputDirectory .\Build\verify

cd ..\ball_beam\stm32f103_rct6
mingw32-make
```

主机测试和编译成功只证明软件可构建；实物成绩、现场录像和台架边界以 `docs/validation/` 中记录为准。
