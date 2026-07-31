# ElectronicDesignContest2026

2026 年电子设计竞赛 H 题“车载平衡滚球运动控制系统”的渐进式重构仓库。

唯一架构是 **C07A 循迹/UI + STM32F103RCT6 滚球闭环 + 01Studio CanMV K230 视觉/图传**。当前唯一代码仍是 C07A V1.1 + S27F 的八路灰度纯循迹固件；其余部分严格按实施计划逐阶段加入。

## 从这里开始

- 赛题要求：[H2026 官方要求](docs/requirements/H2026_OFFICIAL_REQUIREMENTS.md)
- 唯一实施计划：[三板系统实施计划](docs/PROJECT_REBUILD_PLAN.md)
- 当前硬件与接线：[硬件清单](docs/hardware/HARDWARE_INVENTORY.md)、[接线表](docs/hardware/CURRENT_WIRING.md)、[IO 表](docs/hardware/CURRENT_IO_OWNERSHIP.md)
- 开发约束：[AGENTS.md](AGENTS.md)

## 构建与验证

```powershell
cd firmware/line_tracker/sim
mingw32-make test

cd ..
.\tools\build_ticlang.ps1 -OutputDirectory .\Build\verify
```

主机测试只验证控制核心；赛题 H-R02 仍需要在官方尺寸场地完成实车一圈、停车、计时和不脱线验收。
