# STM32F103C8T6 + K230 台架滚球控制实验包

> **范围说明**：这是已在 STM32F103C8T6 台架上联调的独立实验实现，不替换仓库当前冻结的 C07A/MSPM0G3507 整车硬件方案。接入整车前必须重新进行引脚分配、供电复核和闭环验证。

本目录包含三部分：

- `k230/`：01Studio CanMV K230 钢球识别和 UART1 位置帧发送程序；
- `stm32_keil/`：可用 Keil 打开的 STM32F103C8T6 工程，含 D36A、MS42CG 和 K230 UART 接收；
- `docs/`：接线、协议、运行步骤、控制器参数与调参说明。

## 当前闭环

K230 在 1280×720 图像的杆体 ROI 内检测钢球，将像素位置映射为以 O 点为零的坐标，并经 UART1 单向发送给 STM32。STM32 在 200 Hz 控制节拍中使用预测型外环产生电机轴位置目标，再由 MS42CG A/B 编码器内环跟踪该目标。视觉失效超过 150 ms 时，D36A 脉冲立即停止。

```
GC2093 -> K230 检测 -> UART1 -> STM32F103 -> D36A -> 42 步进电机 -> 摆杆
                                      ^                              |
                                      +------- MS42CG A/B -----------+
```

球居中且速度足够小时，控制器使用 ±8 mm 静止带，目标是满足 ±1 cm 的精度要求并避免在中心高频抖动。

## 快速使用

1. 先按 [docs/wiring.md](docs/wiring.md) 完成电源、共地和信号线检查；动力上电前确认 D36A 电流和细分设置。
2. 将 `k230/main.py` 和 `k230/mp_deployment_source/deploy_config.json` 放到 K230 SD 卡；将对应 `.kmodel` 文件另行放进 `mp_deployment_source/`。模型二进制未随本目录提交。
3. 用 Keil 打开 `stm32_keil/USER/NewProject.uvprojx`，确认编译的 UART 源是 `SYSTEM/usart/usart_k230.c`，再下载 STM32。
4. 先完成电机方向和编码器符号测试，再开启视觉闭环。若方向相反，只按 [docs/control.md](docs/control.md) 的符号验证流程更改一个符号。

## 安全边界

- MS42CG 只能接 **3.3 V**；严禁接 5 V。
- K230、STM32 和 D36A 只共地；不要用 UART 线给任意板卡供电。
- D36A 的步进电机电源必须独立满足电流需求，电机回流不得经过 STM32/K230 的细信号地线。
- 当前工程默认 `MOTOR_MICROSTEP=16`。D36A 实际细分必须与该值一致，否则转速、位移和调参全都会失真。

详细引脚表在 [docs/wiring.md](docs/wiring.md)，控制与调参在 [docs/control.md](docs/control.md)。
