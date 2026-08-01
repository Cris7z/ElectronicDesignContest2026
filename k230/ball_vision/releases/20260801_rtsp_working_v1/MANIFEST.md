# K230 RTSP 现场可用冻结版（2026-08-01）

## 冻结结论

- 用户在 SD 卡实机替换后确认“可以用了”。
- 当前启动链为 `/sdcard/main.py` → `/sdcard/ball_app.py`。
- 旧 JPEG/TCP 推流、接收端扫描和相机通道 1 已从活动应用移除。
- 图传改为本地已验证的 Display writeback H.264 RTSP：
  `rtsp://192.168.4.1:8554/ball`。
- 识别、LCD/OSD、距离显示和当时 SD 上的异步 UART 实现保持不变。

这一定版表示“当前现场可用基线”，不自动代表定位精度、长时间稳定性、
RTSP 断线恢复或 K230→RCT6 连续有效协议帧已经完成正式验收。

## 可提交源码指纹

| 文件 | SHA-256 | 说明 |
|---|---|---|
| `main.py` | `A1CB6729EC93A80FEBAA24AD3EB31FD7BDE4B9A9228CE33FE02AF11DB7C6F4B6` | 与实测 SD 文件一致 |
| `rct6_uart.py` | `59DAE3A51CFE95C106FB52119DCA021370E8C4074F39BECDF61FDB5899017451` | 与实测 SD 文件一致 |
| `rtsp_writeback.py` | `1992AFD329B87F1D864105F5302C0913AEBA917CC18A1AA3F0F28ED898F0E21A` | 与实测 SD、本地 RTSP 文件一致 |
| `ball_app.py` | `FA4BABC67DEA23401D5D5A382504727A3636ED0510744F24FA626D9F855891E7` | 仅将 AP 密码回退值替换为占位符后的可提交版本 |
| `private_rtsp_config.example.py` | `D535FF599239EDC3869DD649D1A7E3E3AD52667208D42F0754469E0868601C61` | 不含真实密码 |

实测 SD 上的 `ball_app.py` SHA-256 为
`CBDAFDE57B927070EF00DB741A6333494242F488FC76BB831DB8338562819B54`。
它与这里的版本只有私有 AP 密码回退值不同；未脱敏原件保存在 Git 忽略的
`board_snapshots/20260801_133148_rtsp_restore/ball_app_rtsp_candidate.py`。

## 外部资产指纹

以下实测资产不提交到 Git，重新制卡时必须逐项核对：

| SD 路径 | SHA-256 |
|---|---|
| `/sdcard/private_rtsp_config.py` | `B10FC92360906751DA8AC9D2758035BA50B7A016F814BCEE3BEB92CDC7B5664A` |
| `/sdcard/mp_deployment_source/deploy_config.json` | `8E74C2F0DF8455FFCE5E87C54DED27846478FA7EB7DD5983A663508F6231C0F6` |
| `/sdcard/mp_deployment_source/best_AnchorBaseDet_can2_5_s_20260730113421.kmodel` | `4A67044838ABE7E6E96ADA50F823311552D4702B81D8440271F6E5987ED35F03` |

## 部署

1. 先完整备份目标 SD 卡。
2. 将本目录的 `main.py`、`ball_app.py`、`rct6_uart.py`、
   `rtsp_writeback.py` 复制到 `/sdcard/` 根目录。
3. 将 `private_rtsp_config.example.py` 复制为
   `/sdcard/private_rtsp_config.py`，只在该私有文件填写 AP 密码。
4. 保留并核对上表中的模型及部署配置。
5. 启动后核对 LCD/识别/距离显示，并打开固定 RTSP 地址。

后续实验必须先另存备份或新建版本，不直接覆盖这一定版。
