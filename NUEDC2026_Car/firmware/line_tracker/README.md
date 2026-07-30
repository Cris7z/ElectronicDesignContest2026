# C07A 最小灰度循迹工程

这是重写后的第一阶段工程，只做一件事：BLS 启动后读取 CD4051 八路模拟灰度，以加权
PD 差速驱动 TB6612。硬件实测记录见 [MEASURED_HARDWARE.md](MEASURED_HARDWARE.md)。

控制结构参考 `doggdragon/TiNuedcCar` 的第一版思路：加权误差、PD、软启动、中心缝隙
直行、向最后见线方向短时搜索和丢线停车；但底层引脚与传感器极性完全按本车实测值重写。
首轮实车安全配置禁用中心缝隙直行，并将四路及以上黑色判为异常后锁停。

不包含：起终点线、圈数/计时、编码器闭环、OLED、MPU6050、K230、蓝牙、球杆和 Flash
自动标定。外接 OLED 保持静止是预期行为，不参与控制。

默认基础占空比为 `0.085`，是 PWM 极性修正后实测两轮都可转、且速度相近的最低点。
`app/main.c` 还会强制左/右正向最小输出为 `0.060/0.080`，避免转弯时跌入电机死区。

## 操作

- 短按并松开 BLS：`WAIT -> RUN`；故障时短按松开复位回 `WAIT`。
- 长按 BLS 约 1 s：立即停车并回 `WAIT`。
- 运行时若连续 300 ms 无有效线，先按最后方向搜索，随后进入 `FAULT` 并断能。

用于 CCS 观察的变量：`g_line_tracker_output`、`g_line_tracker_raw_adc`、
`g_line_tracker_tick_overruns`。

构建：

```powershell
cd D:\X\ElectronicDesignContest2026\ElectronicDesignContest2026\NUEDC2026_Car\firmware\line_tracker
.\tools\build_ticlang.ps1 -OutputDirectory .\Build\first
```
