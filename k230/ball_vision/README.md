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

## 主机工具

- `host/snapshot_board.ps1`：从 CanMV 盘或 IDE 导出的目录复制脚本/模型/配置并生成 SHA-256 清单。
- `host/record_rtsp.ps1`：查看或录制单次 RTSP。
- `host/endurance_record_rtsp.ps1`：10 分钟分段耐久录像并生成 SHA-256 清单。
- `host/verify_recording.ps1`：用 `ffprobe` 核验录像。
- `tests/`：只验证标定和状态机，不模拟 K230 硬件。

## 不可提交内容

真实 AP 密码、模型、数据集、校准实参、录像、日志和板端快照全部保持在 `D:\X\ElectronicDesignContest2026-Rebuild-local\k230\`。
