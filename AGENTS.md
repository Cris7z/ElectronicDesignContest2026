# 项目开发约束

## 当前边界

- 当前允许修改的业务代码：`firmware/line_tracker/`（阶段 1）和 `k230/ball_vision/`（阶段 6）。两者不得互相改变接线、SysConfig、轮电机映射或循迹 PD 核心。
- 阶段 0–2 已搭接，验收证据仍待同步；阶段 3–5 由队友并行开发；用户已授权完整阶段 6 并行实施。
- K230 阶段 6 只负责 GC2093 采集、钢球毫米位置、质量/失效状态、RTSP 图传、录像和主机验收。K230 不直接驱动电机、D36A 或安全使能。
- 不得加入三板架构之外的控制器、传感器或通信方案。STM32F103RCT6、D36A、MS42CG、车载供电和机械角度闭环仍由阶段 3–5 的台架门槛管理。
- Q-005、Q-006、Q-008 未关闭前，不得在 `CURRENT_WIRING.md` 或 `CURRENT_IO_OWNERSHIP.md` 分配 K230/STM32/D36A 正式 IO；不得冻结 K230—STM32 UART 引脚、线序、波特率或二进制帧。阶段 6 仅冻结内部 `VisionSampleV1` 和诊断 JSONL。
- 接线与 IO 的唯一入口是 `docs/hardware/CURRENT_WIRING.md` 和 `docs/hardware/CURRENT_IO_OWNERSHIP.md`。
- 发生实物、原理图、代码或接线冲突时，停止相关修改，更新 `docs/decisions/OPEN_QUESTIONS.md` 并询问用户。

## K230 阶段 6 规则

- 对外坐标固定为摆杆中心 `O=0 mm`，从铰点指向执行端为正；相机镜像只能由标定配置处理。
- 每个采集帧必须生成新序号；无效帧不得复用上一有效坐标。连续 3 帧或 100 ms 无效进入 `LOST`，恢复需连续 3 帧通过 ROI、置信度、尺寸和时序门控。
- RTSP 固定使用 K230 自建隔离 AP；密码、模型、数据集、校准实参、录像和板端快照均为本地资产，不得提交 Git。
- 现有 `K230.zip` 中 F407 引脚、旧 JPEG/TCP 接收器、`/deploy-main` 源码泄露入口及第三方 System32 DLL 操作均不得复用。

## 循迹安全规则

- 左轮=TB6612 B（PB3、PA16、PA17）；右轮=TB6612 A（PB2、PA14、PA13）。
- PA18：松开=0、按下=1；外部 47 kΩ 下拉，禁止内部上下拉。
- 电机上电/WAIT/FAULT 必须滑行并 disarm；反向前至少有一个 5 ms 滑行周期。
- CD4051 为 5 V 供电时，OUT 必须不超过 MSPM0 VDD；更换板子或调整安装必须重新测标定。

## 验证要求

修改控制核心后先运行 `firmware/line_tracker/sim` 的全部测试；能访问 TI 工具链时再运行 `build_ticlang.ps1`。没有实车时不得声称赛道验证通过。
