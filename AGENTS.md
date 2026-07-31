# 项目开发约束

## 当前边界

- 当前唯一允许修改的业务代码：`firmware/line_tracker/`。
- 不得提前加入 D36A、MS42CG、K230、摄像头/图传、球杆、MPU6050、蓝牙、圈数或终点逻辑。
- 接线与 IO 的唯一入口是 `docs/hardware/CURRENT_WIRING.md` 和 `docs/hardware/CURRENT_IO_OWNERSHIP.md`。
- 发生实物、原理图、代码或接线冲突时，停止相关修改，更新 `docs/decisions/OPEN_QUESTIONS.md` 并询问用户。

## 循迹安全规则

- 左轮=TB6612 B（PB3、PA16、PA17）；右轮=TB6612 A（PB2、PA14、PA13）。
- PA18：松开=0、按下=1；外部 47 kΩ 下拉，禁止内部上下拉。
- 电机上电/WAIT/FAULT 必须滑行并 disarm；反向前至少有一个 5 ms 滑行周期。
- CD4051 为 5 V 供电时，OUT 必须不超过 MSPM0 VDD；更换板子或调整安装必须重新测标定。

## 验证要求

修改控制核心后先运行 `firmware/line_tracker/sim` 的全部测试；能访问 TI 工具链时再运行 `build_ticlang.ps1`。没有实车时不得声称赛道验证通过。

