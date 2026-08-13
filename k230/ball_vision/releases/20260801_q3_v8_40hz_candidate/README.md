# K230 H-R03 V8 UART 候选

此目录只保存待合并到 SD 卡实测版本的 `rct6_uart.py`，不替代或修改 `20260801_rtsp_working_v1` 回退基线。

- 保留 GPIO3/TX、GPIO4/RX、`$B,...*XOR` 位置帧、`$V,0/1*XOR` RTSP 命令和 `take_video_mode_request()` 接口。
- 发送周期下限为 25 ms，即最多 40 Hz。
- 每次 `publish()` 产生新的邮箱代次；后台线程每个代次最多发送一次，不重复旧序号维持 STM32 看门狗。
- RTSP 默认关闭及主线程启停逻辑仍由 SD 卡现有 `ball_app.py` 负责。本文件部署前必须先回读卡内当前版本并逐项比较，不能覆盖未备份的实测程序。
