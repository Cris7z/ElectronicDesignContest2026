# K230 阶段 6 Agent 约束

- 允许修改范围：仅 `k230/ball_vision/` 及阶段 6 文档/测试工具。
- 01Studio CanMV K230 + GC2093 只负责视觉、RTSP、录像和主机证据；不得控制 D36A、电机或安全使能。
- 对外测量为 `VisionSampleV1`：`version`、`seq`、`capture_ms`、`x_mm`、`quality`、`status`。坐标 O=0，从铰点到执行端为正。
- 每个采集帧递增序号；无效帧不得沿用旧坐标；3 帧或 100 ms 无效进入 `LOST`，连续 3 帧门控通过才恢复 `VALID`。
- K230 自建隔离 AP；RTSP 固定 `rtsp://192.168.4.1:8554/ball`。密码只放 `private_config.py`，该文件不可提交。
- `models/`、`datasets/`、`recordings/`、`logs/`、`board_snapshots/`、真实 `calibration.json` 和 `.kmodel` 均不可提交。
- 不冻结 UART 引脚、线序、波特率或二进制协议；它们属于阶段 7/Q-008。
- 运行主机单测：`python -m unittest discover -s k230/ball_vision/tests -v`。
- 无 K230 实物时，只能报告主机测试通过；不得声称相机、RTSP 或赛题验收通过。
