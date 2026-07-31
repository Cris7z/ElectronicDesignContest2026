# STM32 历史工程使用边界

当前只保留一份与唯一三板架构直接相关的历史工程：

| 来源 | 可参考 | 禁止直接照搬 |
|---|---|---|
| `D:\X\ElectronicDesignContest2026\reference\2024H_keil_early` | STM32F103RC 启动、UART、定时器和既有板卡使用经验。 | 旧循迹、电机、OLED、引脚分配、控制参数和业务状态机。2026 H 题必须重新分配 IO、实现协议并完成台架安全验收。 |

当前真值入口依次是：

1. `docs/PROJECT_REBUILD_PLAN.md`；
2. `docs/decisions/OPEN_QUESTIONS.md`；
3. `docs/hardware/HARDWARE_INVENTORY.md`；
4. 对应硬件技术文档；
5. 后续正式接线表、IO 表和实物验收记录。
