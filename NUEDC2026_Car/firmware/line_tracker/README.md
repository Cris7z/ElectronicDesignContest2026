# C07A 最小灰度循迹工程

这是重写后的纯循迹工程：BLS 启动后读取 CD4051 八路模拟灰度，以连续位置误差、
PD 差速驱动 TB6612。硬件实测记录见 [MEASURED_HARDWARE.md](MEASURED_HARDWARE.md)。

控制结构保留随附 TiNuedcCar `CarControl` 的核心优点（误差越大速度越低、偏航 PID
差速、最外侧传感器辅助急转），但底层引脚、黑白极性和参数均按本车实测值实现。本版在
黑线状态机之外使用八路归一化灰度强度作连续质心：黑线掩码仍负责有效线、丢线和宽线判断，
位置误差则不再只能跳在离散权重上。四路及以上同时为黑色视为横线/宽线，低速直行通过，
而不是锁停。

不包含：起终点线、圈数/计时、编码器速度闭环、OLED、MPU6050、K230、蓝牙、球杆和
Flash 自动标定。外接 OLED 保持静止是预期行为，不参与控制。

当前实车参数为 `P=5, I=0, D=125`，基础最高占空比 `48.75%`、最低 `23%`，最终 PWM
仍可到 `51.75%`。中心直线从
`8%` 占空比起步，以每 5 ms `0.5%` 加速；速度误差幅值另经 `alpha=0.20` 低通，权重
不超过 1 时保持最高速度，在权重 1 到 4 间以平滑曲线降至最低速度，最多每 tick `0.8%`
降速。偏航在小误差时从 90% 平滑增益起步，到权重 4 恢复 100%；最外侧传感器在权重 5
到 7 间渐进混入边缘急转，避免突跳。
当前实车转向极性为 `-1`。`app/main.c` 还会强制左/右正向最小输出为 `0.060/0.080`，
避免转弯时跌入电机死区；编码器每 20 ms 运行左右独立的 P 型速度闭环（`Kp=0.30,
Ki=0`）。灰度外环的左右输出仍是各自的速度目标；各轮只加上不超过 `±3%` 的速度修正，
并在最终 PWM 限幅前合成。旧的 100 ms 相对均衡器已删除，避免两个闭环互相叠加。

## 操作

- 短按并松开 BLS：`WAIT -> RUN`；故障时短按松开复位回 `WAIT`。
- 长按 BLS 约 1 s：立即停车并回 `WAIT`。
- 有效 ADC 下连续 3 帧看不到黑线，才开始按最后方向搜索；确认搜线后连续 300 ms
  无有效线进入 `FAULT` 并断能。ADC 扫描无效则立即故障停车。
- 连续 2 帧四路及以上黑色才进入宽线低速直行；单帧宽黑保持上一帧输出。
- 当前保留独立限幅，避免急弯时为保差速而让内轮突然反转；保差速分配将在速度 PI
  目标层重新评估。

用于 CCS 观察的变量：`g_line_tracker_output`、`g_line_tracker_raw_adc`、
`g_line_tracker_tick_overruns`、`g_line_tracker_shadow_left_target`、
`g_line_tracker_shadow_right_target`、`g_line_tracker_shadow_left_measured`、
`g_line_tracker_shadow_right_measured`、`g_line_tracker_shadow_left_correction`、
`g_line_tracker_shadow_right_correction`。

构建：

```powershell
cd D:\X\ElectronicDesignContest2026\ElectronicDesignContest2026\NUEDC2026_Car\firmware\line_tracker
.\tools\build_ticlang.ps1 -OutputDirectory .\Build\first
```
