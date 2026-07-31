# STM32F103C8T6 D36A Keil motor test project

This is a complete Keil uVision Standard Peripheral Library project for testing one MS42CG motor with D36A channel 1. It is based on the supplied F103C8T6 project, with a new test entry point and new D36A driver module.

## Fixed wiring for this project

| D36A | STM32F103C8T6 |
|---|---|
| `ST1` | `PB6` (`TIM4_CH1` PWM) |
| `EN1` | `PB8` |
| `DIR1` | `PB9` |
| `ADC` | `PA3` (`ADC1_IN3`, optional) |
| `GND` | `GND` |
| `Vin+` / `Vin-` | Separate 12 V supply + / - |

The MS42CG motor connects to the D36A motor-output header as `1=A+`, `3=A-`, `4=B+`, `6=B-`. Leave motor cable pins 2 and 5 (common wires) disconnected.

The default test uses 1/16 microstep. Set D36A DIP switches 1/2/3 to `OFF/OFF/OFF`, and initially set switches 4/5/6 to `ON/ON/ON` (0.55 A). The motor is enabled and rotates at 10 RPM; it stops briefly and reverses direction every five seconds.

## Build and flash

1. Open `USER/NewProject.uvprojx` with Keil uVision 5.
2. Select target `Target 1`, then **Build** (`F7`). The project device is `STM32F103C8` and uses the medium-density startup file.
3. Connect ST-Link: `SWDIO -> PA13`, `SWCLK -> PA14`, `GND -> GND`, and power the C8T6 board appropriately.
4. Download with **Load** (`F8`).

Serial debug output is USART1 TX `PA9`, 115200 baud, 8-N-1. Connect a 3.3 V USB-to-TTL adapter as `adapter RX <- PA9` and connect GND. The program prints D36A input voltage and direction twice per second.

## MS42CG encoder wiring and output

The current firmware also reads the MS42CG encoder while keeping `PB6/PB8/PB9` for D36A motor control.

| MS42CG encoder cable pin | Signal | STM32F103C8T6 |
|---:|---|---|
| 12 | VCC | 3V3 |
| 7 | GND | GND |
| 11 | A | PA0 / TIM2_CH1 |
| 10 | B | PA1 / TIM2_CH2 |
| 9 | PWM absolute angle | PA6 / TIM3_CH1 |
| 8 | Z index | PA12 / EXTI12 |

At 115200 baud, the firmware prints `QEI` (accumulated A/B count), `SPD` (encoder speed), `PWM` (single-turn absolute angle) and `Z` (index-pulse count) every 100 ms. This is monitoring only; it does not yet use the encoder to close the motor-control loop.

## Important files

- `USER/main_d36a_test.c`: test application. Change `MOTOR_TEST_RPM` or `MOTOR_MICROSTEP` here.
- `HARDWARE/d36a_motor.c`: D36A GPIO and TIM4 PWM driver.
- `HARDWARE/d36a_motor.h`: driver interface and pin definitions.

Before increasing current, check driver/motor temperature. D36A provides up to 1.44 A per channel, below the MS42CG rated 1.65 A phase current.
