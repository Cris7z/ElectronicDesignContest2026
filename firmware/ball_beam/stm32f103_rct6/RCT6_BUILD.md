# RCT6 球杆滚球控制工程

这是针对当前实物的独立 STM32F103RCT6 工程。控制算法来自已验证的台架分支，D36A 和 MS42CG 接线保持不变；只做了两处目标适配：

- 编译目标为高密度 `STM32F10X_HD`（256 KiB Flash、48 KiB RAM）。
- K230 的 `TX1` 从原工程的 `PA10/USART1_RX` 改为实物已接好的 `PA3/USART2_RX`；`PA9/USART1_TX` 继续留给板载 CH340 输出日志。

现有引脚：PB6=STEP、PB8=EN、PB9=DIR、PA0/PA1=MS42CG A/B、PA6=PWM、PA12=Z、PA3=K230 TX1。

在 PowerShell 或 MSYS2 环境中执行：

```sh
mingw32-make
```

生成文件为 `build/ball_beam_rct6.elf` 和 `.bin`。接好 ST-Link 后可执行 `mingw32-make flash` 下载。下载动作会复位目标板；不要在电机动力上电且机械机构无人看守时执行。
