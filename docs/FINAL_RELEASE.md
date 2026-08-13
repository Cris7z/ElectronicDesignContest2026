# H2026 项目正式版本与恢复清单

> 结项日期：2026-08-13
> 正式 Git 标签：`h2026-project-final-20260813`
> 结项分支：`codex/h2026-final-20260813`

## 1. 正式版本

| 子系统 | 正式版本 | 恢复入口 | SHA-256 / 关键状态 |
|---|---|---|---|
| C07A 循迹/UI | 最后一次实车烧录版；用户明确将其作为正式版 | `firmware/line_tracker/` | `line_tracker.out`：`9C4D83D84752FD53D75F9311A4A98B80D20FAD285FE7D7B1F1F158CFFF4FFC5A` |
| C07A Intel HEX | 与上项同一次构建 | `firmware/line_tracker/Build/default/line_tracker.hex`（构建产物，不入 Git） | `0CE4F544134CE963EBC1279BB3524F0658C477DD913AD0AB7FE91C40985D260B` |
| RCT6 球杆控制 | V16 `LEVEL_COAST_HOLD` | `firmware/ball_beam/stm32f103_rct6/USER/main_ball_beam_k230_control.c` | 活动源与 V16 归档源相同：`C6D6E134EDEFDDC22E9EA74069B3F26E501C1065F194366E46BF9A5FEBC65D43` |
| RCT6 可烧录镜像 | V16 | `firmware/ball_beam/stm32f103_rct6/releases/20260801_v16_q3_level_coast_hold/ball_beam_rct6.bin` | `7F728CCA6F625C273C94BD04D8F176491EB44D8C8F5E5C7123865F8FCD20BC93` |
| K230 现场可用图传基线 | 2026-08-01 RTSP 冻结版 | 独立分支/工作树 `codex/stage6-k230`，标签 `k230-rtsp-working-20260801` | 模型、私密 AP 配置和 SD 卡快照不入 Git |

## 2. C07A 正式行为

- MODE 只循环 `M1 -> M3 -> M1`，不存在 M2/B 点停车模式。
- M1 保持冻结循迹控制，最终停车目标为 `7.060 m`，安全超时 `20 s`。
- M3 为已消除高频摆动的 `23%` 匀速版，停车目标 `7.040 m`，安全超时 `60 s`。
- 正式烧录恢复 CRC=`0x74D9` 的冻结灰度标定；现场按键标定只在运行时按需覆盖。
- 最后一次烧录后的目标状态为 `WAIT / M1`、`TICK_OVERRUNS=0`、`CAL_LOADED=1`、无 FAULT。

## 3. RCT6 目录约定

- `USER/main_ball_beam_k230_control.c`：正式活动源，当前等同 V16。
- `USER/main_ball_beam_k230_hr04_experimental.c`：H-R04 实验源，不是正式构建入口。
- `releases/20260801_v16_q3_level_coast_hold/`：正式 V16 源码与 BIN 回退点。
- `releases/` 其余 V6–V15：调参历史，只用于问题追溯。
- `Makefile` 默认编译正式活动源；`.elf/.map` 属于可再生构建产物，不归档。

## 4. 恢复与验证

```powershell
# C07A 主机测试与 TI 构建
mingw32-make -C firmware/line_tracker/sim test
cd firmware/line_tracker
.\tools\build_ticlang.ps1

# RCT6 构建
cd ..\ball_beam\stm32f103_rct6
mingw32-make
```

C07A 烧录必须使用 `firmware/line_tracker/tools/flash_target.py`，让应用程序与末端标定扇区一起下载并校验。RCT6 烧录会复位控制器，机械机构和动力电源无人看守时禁止执行。

## 5. 历史与边界

- `c07a-line-tracker-final-20260801` 保留 M1/M3 的 `7.040 m` 旧终版，可用于回退；不移动或覆盖该标签。
- K230 的 40 Hz/Q3 UART 代码仍保存在独立 Stage6 工作树中；它与现场 RTSP 冻结标签分开管理，不用候选目录覆盖已验证回退基线。
- 详细实物排障经过见 `docs/validation/`。其中早期 PA3/USART2 记录已被同一日志后部的 PA10/USART1 实测状态覆盖，不能只摘录早期结论。
