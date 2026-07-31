# K230 钢球视觉与 RTSP

本目录是阶段 6 的独立 K230 实现。它不连接 STM32，也不分配跨板 IO。

## 部署

1. 先导出当前工作卡的脚本、模型和版本信息到本地快照；不要覆盖唯一工作卡。
2. 在备用 SD 卡烧录已审计的 01Studio CanMV K230 v1.8 镜像。
3. 将本目录文件复制到 `/sdcard/ball_vision/`；模型和真实标定文件分别放到已忽略的 `models/`、`calibration.json`。
4. 将 `config_example.py` 复制为 `private_config.py`，仅在该本地文件填 AP 密码、模型路径、Sensor ID、模型哈希和校准路径。
5. 启动 `main.py`。电脑连接 K230 AP 后使用：

```text
rtsp://192.168.4.1:8554/ball
```

### 首次图传探针

首选 `rtsp_sensor_probe.py` + `rtsp_sensor_transport.py`：这是基于
CanMV v1.8 官方 `rtsp_server.py` 的直接相机→YUV420SP→硬编码器→RTSP
路径，先验证 AP 稳定性、H.264 码流与完整相机画面覆盖。部署时同时带上
`rtsp_probe_config_example.py`，并复制为不提交的
`private_rtsp_config.py` 设置 WPA2 密码。

`rtsp_probe.py` + `rtsp_writeback.py` 保留为第二步方案：它从 Display
writeback 取帧，因此只在确认需要把检测框和诊断叠加进图传时再启用。

## 主机工具

- `host/snapshot_board.ps1`：从 CanMV 盘或 IDE 导出的目录复制脚本/模型/配置并生成 SHA-256 清单。
- `host/record_rtsp.ps1`：查看或录制单次 RTSP。
- `host/endurance_record_rtsp.ps1`：10 分钟分段耐久录像并生成 SHA-256 清单。
- `host/verify_recording.ps1`：用 `ffprobe` 核验录像。
- `tests/`：只验证标定和状态机，不模拟 K230 硬件。

## 不可提交内容

真实 AP 密码、模型、数据集、校准实参、录像、日志和板端快照全部保持在 `D:\X\ElectronicDesignContest2026-Rebuild-local\k230\`。
