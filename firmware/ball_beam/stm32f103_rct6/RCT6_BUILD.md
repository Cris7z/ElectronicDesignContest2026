# RCT6 球杆滚球控制工程

这是针对当前实物的独立 STM32F103RCT6 工程。D36A、MS42CG 与 K230 接线均按当前实物保持不变：

- 编译目标为高密度 `STM32F10X_HD`（256 KiB Flash、48 KiB RAM）。
- K230 `GPIO3/UART1_TX` 接 `PA10/USART1_RX`；`PA9/USART1_TX` 同时连接 K230 `GPIO4/UART1_RX` 与板载 CH340 RX。
- V8 使用 K3/PC8 切换备用 RTSP，K4/PC9 启动或中止第三问 `+50 mm -> -50 mm` 自动摆球。两键均为内部上拉、按下低电平、30 ms 消抖；映射已经 SWD 按住实测确认。

现有引脚：PB6=STEP、PB8=EN、PB9=DIR、PA0/PA1=MS42CG A/B、PA6=PWM、PA12=Z、PA10=K230 TX、PA9=K230 RX/CH340 RX、PC8=K3、PC9=K4。

在 PowerShell 或 MSYS2 环境中执行：

```sh
mingw32-make
```

生成文件为 `build/ball_beam_rct6.elf` 和 `.bin`。接好 ST-Link 后可执行 `mingw32-make flash` 下载。下载动作会复位目标板；不要在电机动力上电且机械机构无人看守时执行。
