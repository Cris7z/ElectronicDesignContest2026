# ElectronicDesignContest2026 hardware contract

This repository is a hardware-first project. `硬件及接线清单.md` is the
canonical hardware source of truth; this file is its coding-contract mirror.
Before changing firmware, use these two files. Do not re-audit vendor PDFs or
web pages for facts already frozen here. If the two files ever disagree,
stop and update both before changing code.

Re-open original technical material only when:

- the user replaces a module or reports a different board revision;
- a fact below is explicitly marked `待实测` or `待实物确认`;
- real hardware behaviour contradicts the frozen record.

When hardware changes, update this file, `硬件及接线清单.md`, the root
`README.md`, affected drivers and host tests in the same change.

## Locked hardware stack

- Real-time controller: WHEELTEC C07A / TI MSPM0G3507 installed on the
  S27F/S28A modular base. The exact base-board silk is still to be recorded.
- Installed controller modules: P03B 12 V to 5 V regulator, MPU6050 on H5,
  four-wire OLED, BLS start button and status LED.
- Modules supplied with the controller but removed for the first build:
  D103A/TB6612 and the onboard Bluetooth module.
- Chassis drive: external D157B / dual AT8236 driving two MG513XP28 12 V
  encoder motors on a 32 cm by 24 cm three-wheel differential chassis.
- Main line sensor: HiWonder LineFollower_8CH v1.0, eight infrared probes,
  5 V/85 mA, fixed 7-bit I2C address 0x5D.
- Rule fallback line sensor: LF04 four-channel infrared module on U3.
- Beam drive: D36A plus a 42-size stepper and MS42CG A/B/PWM encoder.
- Vision and video: 01Studio CanMV K230 standard board, planned 1 GB version,
  with the bundled GC2093 70-degree 24-pin camera. The K230 is planned, not
  yet hardware-validated.
- Mechanics: 25 cm PPR beam, about 1 cm steel ball, hinge and transmission.

## Frozen MSPM0 resource ownership

| Function | MSPM0 resource | Hardware endpoint |
|---|---|---|
| Left chassis motor | PB2/PB3, TIMG6_CCP0/1 | D157B AIN1/AIN2 |
| Right chassis motor | PA8/PA9, TIMA0 channels | D157B BIN1/BIN2 |
| Left wheel encoder A/B | PA25/PA26 | GPIO quadrature |
| Right wheel encoder A/B | PB20/PB24 | GPIO quadrature |
| Shared I2C0 SDA/SCL | PA0/PA1 | MPU6050 plus HiWonder 8CH |
| MPU6050 interrupt | PA7 | H5 pin 8 |
| LF04 O1/O2/O3/O4 | PA27/PA12/PB16/PB17 | U3 pins 6/5/4/3 |
| OLED SCL/SDA/RST/DC | PA28/PA31/PB14/PB15 | Installed OLED |
| Start button / status LED | PA18/PB9 | BLS / LED |
| Control tick | TIMG7 | No external pin |
| USB debug UART0 | PA10 TX / PA11 RX | 115200 |
| K230 state receive | PB7 / UART1_RX | K230 GPIO11 / UART2 TX |
| D36A STEP | PA24 / TIMA1_CCP1 | D36A ST1 |
| D36A DIR / EN | PA13 / PA22 | D36A DIR1 / EN1 |
| MS42CG incremental A/B | PB18/PB19 | C07A V1.1 pads, software quadrature |
| MS42CG absolute PWM | PA14 / TIMG12_CCP0 | Dual-edge capture |
| SWD | PA19/PA20 | Programming and debug |

Do not use the old `firmware/bsp/car_mspm0g3507.syscfg` for this vehicle.
It is a generic-car baseline with incompatible PWM, I2C, display and sensor
assumptions. Build a new H2026 SysConfig only after the physical C07A and
S27F/S28A revisions are confirmed.

## Connector and protocol facts

- H5: pin 1=5 V, pin 2=GND, pin 3=PA1/SCL, pin 4=PA0/SDA,
  pin 8=PA7/MPU6050 INT. Use a split harness for the HiWonder sensor and keep
  the MPU6050 installed.
- HiWonder sensor connector: 5V/GND/SDA/SCL. Fast control reads register 5
  as one state byte. Registers 6..21 are eight little-endian analog values;
  22..37 are eight little-endian learned thresholds. Start I2C at 100 kHz.
- `linefollower_8ch.state_valid` is mandatory for motion. A failed state read
  invalidates the line sample; analog or threshold diagnostic reads must not
  restore it.
- LF04 follows the WHEELTEC example, not U3 pin-number order:
  O1/O2/O3/O4=PA27/PA12/PB16/PB17.
- K230 to MSPM0 is one-way in the first build:
  K230 GPIO11/TX2 -> PB7/UART1_RX plus common GND. GPIO12/RX2 and the K230
  3V3 pin are not connected.
- K230 state frames are 17 bytes as documented in
  `硬件及接线清单.md`; stale/invalid frames must drive the system to a safe
  state. RTSP is never part of the control loop.

## Mandatory conflict rules

- Keep MPU6050 on PA0/PA1/PA7. Do not copy the D36A vendor example that uses
  PA0/PA1 for QEI.
- D103A/TB6612 must be physically removed: it conflicts with D157B and uses
  PA13/PA14 needed by the beam system.
- Onboard Bluetooth must be physically disconnected before K230 uses PB7.
- The CCD connector must remain empty because PA22 is D36A EN.
- MS42CG uses 3.3 V only. Never power it from 5 V.
- P03B and D157B 5 V outputs must never be paralleled.
- Motor and stepper return currents must not flow through C07A or K230 signal
  ground wiring.

## Facts that still require hardware measurement

- S27F versus S28A base-board silk and C07A V1.1 PB18/PB19 availability.
- D36A EN polarity, microstep setting, current limit and reset-safe pull.
- MS42CG direction, counts per revolution, PWM period and absolute-angle
  alignment.
- MG513XP28 encoder CPR, reduction ratio, phase order and stall current.
- HiWonder SDA/SCL idle voltage and combined pull-ups with MPU6050. If the
  sensor side idles above about 3.6 V, isolate it with a bidirectional I2C
  level shifter before sharing H5.
- P03B load, ripple and temperature margin with the controller and K230.
- Battery capacity/C rating, fuse value and wire gauge from measured peaks.
- K230 memory, power, UART, camera and simultaneous H.264/RTSP two-hour test.
- Final vehicle envelope, centre of gravity and camera/beam geometry.
