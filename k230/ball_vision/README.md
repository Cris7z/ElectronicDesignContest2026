# K230 钢球视觉与 RTSP

本目录包含阶段 6 的通用实现，以及已经过现场确认的 SD 卡冻结版本。

## 当前冻结基线

2026-08-01，用户确认移除旧 JPEG/TCP 图传并换成本地 H.264 RTSP 后
“可以用了”。当前冻结版位于：

```text
k230/ball_vision/releases/20260801_rtsp_working_v1/
```

其清单记录了活动源码、模型和部署配置的 SHA-256、私有配置边界及回滚方法。
后续实验不得直接覆盖该版本。

顶层 `main.py`、`vision_contract.py` 等仍是正式测量契约开发线；冻结版记录的
是当前实机可用状态，两者不要混作同一份板端验收结果。

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
