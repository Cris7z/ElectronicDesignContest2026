# ElectronicDesignContest2026

2026 年电子设计竞赛 H 题“车载平衡滚球运动控制系统”的渐进式重构仓库。

当前唯一代码是 C07A V1.1 + S27F 的八路灰度纯循迹固件；不包含 D36A、球杆、K230、图传、陀螺仪、蓝牙或整圈赛题流程。

## 从这里开始

- 赛题要求：[H2026 官方要求](docs/requirements/H2026_OFFICIAL_REQUIREMENTS.md)
- 当前硬件与接线：[硬件清单](docs/hardware/HARDWARE_INVENTORY.md)、[接线表](docs/hardware/CURRENT_WIRING.md)、[IO 表](docs/hardware/CURRENT_IO_OWNERSHIP.md)
- 当前代码范围：[迁移清单](docs/LINE_TRACKER_MIGRATION_MANIFEST.md)
- 开发约束：[AGENTS.md](AGENTS.md)

## 构建与验证

```powershell
cd firmware/line_tracker/sim
mingw32-make test

cd ..
.\tools\build_ticlang.ps1 -OutputDirectory .\Build\verify
```

主机测试只验证控制核心；赛题 H-R02 仍需要在官方尺寸场地完成实车一圈、停车、计时和不脱线验收。

