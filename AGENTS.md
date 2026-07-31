# 项目开发约束

## 当前边界

- 当前唯一允许修改的业务代码：`firmware/line_tracker/`。
- 阶段 1 的 H-R02 功能已冻结；停车里程的赛前微调不阻塞后续准备。Q-005、Q-006、Q-008 的台架门槛关闭前，仍不得建立 STM32F103RCT6、D36A、MS42CG、K230 或跨板通信运行代码，也不得改变现有接线、SysConfig、轮电机映射或循迹 PD 核心。
- 不得加入三板架构之外的控制器、传感器或通信方案；不得提前加入 STM32F103RCT6、D36A、MS42CG、K230、摄像头/图传或球杆控制。
- 后续三板架构已选择 C07A（循迹/UI）+ STM32F103RCT6（滚球实时控制）+ 01Studio CanMV K230（视觉/图传）；在 Q-005、Q-006、Q-008 的台架门槛关闭前，不得据此分配正式 IO 或建立未来控制代码。
- 接线与 IO 的唯一入口是 `docs/hardware/CURRENT_WIRING.md` 和 `docs/hardware/CURRENT_IO_OWNERSHIP.md`。
- 发生实物、原理图、代码或接线冲突时，停止相关修改，更新 `docs/decisions/OPEN_QUESTIONS.md` 并询问用户。

## 循迹安全规则

- 左轮=TB6612 B（PB3、PA16、PA17）；右轮=TB6612 A（PB2、PA14、PA13）。
- PA18：松开=0、按下=1；外部 47 kΩ 下拉，禁止内部上下拉。
- 电机上电/WAIT/FAULT 必须滑行并 disarm；反向前至少有一个 5 ms 滑行周期。
- CD4051 为 5 V 供电时，OUT 必须不超过 MSPM0 VDD；更换板子或调整安装必须重新测标定。
- H-R02 起跑/停车的评分基准是 A 点 30 cm 的细停车基准虚线：BLS 短按松手启动，车身停车标记起跑和停车都对准该线；停车标记在灰度传感器后方 100 mm。

## 验证要求

修改控制核心后先运行 `firmware/line_tracker/sim` 的全部测试；能访问 TI 工具链时再运行 `build_ticlang.ps1`。没有实车时不得声称赛道验证通过。
