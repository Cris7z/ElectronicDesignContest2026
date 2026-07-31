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
  BLS start button, status LED and D103A/TB6612 motor module. The failed
  four-wire OLED is replaced by an external four-pin I2C OLED.
- Modules supplied with the controller but removed for the current build: the
  onboard Bluetooth module.  The separately supplied D157B/AT8236 is not in
  the chassis drive path.
- Chassis drive: integrated D103A/TB6612 driving two MG513XP28 12 V encoder
  motors on a 32 cm by 24 cm three-wheel differential chassis.
- Main line sensor: CD4051 eight-channel gray module, 5 V supply, scanned as
  analog OUT through MSPM0 ADC1/A1_4. Its U3 harness is AD2=PA12, AD1=PA27,
  AD0=PB16 and OUT=PB17. The removed HiWonder UART/I2C module is not a
  fallback in this build.
- Rule fallback line sensor: LF04 four-channel infrared module, electrically
  incompatible with the occupied U3 gray-sensor harness.
- Beam drive: D36A plus a 42-size stepper and MS42CG A/B/PWM encoder.
- Vision and video: 01Studio CanMV K230 standard board, planned 1 GB version,
  with the bundled GC2093 70-degree 24-pin camera. The K230 is planned, not
  yet hardware-validated.
- Mechanics: 25 cm PPR beam, about 1 cm steel ball, hinge and transmission.

## Frozen MSPM0 resource ownership

| Function | MSPM0 resource | Hardware endpoint |
|---|---|---|
| Left chassis motor | PB2/TIMA1_CCP0 + PA14/PA13 | TB6612 A/PWMA channel; positive firmware duty is physical forward (re-measured 2026-07-31) |
| Right chassis motor | PB3/TIMA1_CCP1 + PA16/PA17 | TB6612 B/PWMB channel; positive firmware duty is physical forward (re-measured 2026-07-31) |
| Left wheel encoder A/B | PA25/PA26 | GPIO quadrature; physical-forward sign = -1 |
| Right wheel encoder A/B | PB20/PB24 | GPIO quadrature; physical-forward sign = +1 |
| I2C0 SDA/SCL | PA0/PA1 | MPU6050 (0x68) plus external OLED (0x3C) |
| Gray 8CH mux select | PA12/PA27/PB16 | CD4051 AD2/AD1/AD0 |
| Gray 8CH analog OUT | PB17 / ADC1_A1_4 | CD4051 OUT; 0..VDD only |
| MPU6050 interrupt | PA7 | H5 pin 8 |
| LF04 O1/O2/O3/O4 | unavailable | U3 is occupied by the gray 8CH harness |
| External OLED SDA/SCL | PA0/PA1, I2C0 | 4-pin SSD1306-compatible OLED, default 0x3C; VCC=3.3 V |
| Start button / status LED | PA18/PB9 | BLS / LED |
| Control tick | TIMG7 | No external pin |
| Debug UART | PA10 TX / PA11 RX, UART0 | 115200; separate from the line sensor |
| K230 state receive | PB7 / UART1_RX | Planned route; PB6/PB7 are released but not allocated by Q2 |
| D36A STEP | Unassigned in current build | TB6612 owns TIMA1 for PB2/PB3 |
| D36A DIR / EN | Unassigned in current build | TB6612 owns PA13; D36A is disconnected |
| MS42CG incremental A/B | PB18/PB19 | C07A V1.1 pads, software quadrature |
| MS42CG absolute PWM | Unassigned in current build | TB6612 owns PA14; MS42CG is disconnected |
| SWD | PA19/PA20 | Programming and debug |

Do not use the old `firmware/bsp/car_mspm0g3507.syscfg` for this vehicle.
It is a generic-car baseline with incompatible PWM, I2C, display and sensor
assumptions. Build a new H2026 SysConfig only after the physical C07A and
S27F/S28A revisions are confirmed.

## Connector and protocol facts

- H5: pin 1=5 V, pin 2=GND, pin 3=PA1/SCL, pin 4=PA0/SDA,
  pin 8=PA7/MPU6050 INT. I2C0 is shared electrically by the installed
  MPU6050 (0x68) and the external OLED (0x3C), but the OLED must draw VCC
  from a **3.3 V** point rather than H5 pin 1 (5 V). Connect its `SCL` to
  PA1 and `SDA` to PA0, with a common GND.
- CD4051 gray sensor: supply 5 V/GND, then wire AD2=PA12, AD1=PA27,
  AD0=PB16 and OUT=PB17/ADC1_A1_4. The installed module was measured at
  about ADC 170 on white and ADC 4095 on black; it is therefore an analog,
  high-on-black source. R0..R7 are physical left-to-right when the vehicle
  faces forward. Keep the OUT voltage within 0..VDD and use a divider if a
  future module exceeds that range. If a 5 V mux does not guarantee 3.3 V
  VIH, buffer AD0..AD2 with 74AHCT125.
- One complete 000..111 scan is mandatory for motion. Switch settle time,
  ADC validity, scan budget and per-channel white/black calibration are all
  safety gates; all-white and all-black are valid optical patterns, not a bus
  failure.
- LF04 cannot be connected while this harness occupies U3.
- K230 to MSPM0 is a later, one-way route:
  K230 GPIO11/TX2 -> PB7/UART1_RX plus common GND. GPIO12/RX2 and the K230
  3V3 pin are not connected. It is a later hardware route and remains
  unplugged during Q2 commissioning.
- K230 state frames are 17 bytes as documented in
  `硬件及接线清单.md`; stale/invalid frames must drive the system to a safe
  state. RTSP is never part of the control loop.

## Mandatory conflict rules

- Keep MPU6050 and the external OLED on PA0/PA1, with unique I2C addresses;
  do not copy the D36A vendor example that uses PA0/PA1 for QEI.
- D103A/TB6612 remains installed for chassis drive.  Its PA13/PA14 direction
  pins conflict with the current D36A/MS42CG plan, so D36A must remain
  disconnected until its direction and absolute-angle pins are reassigned.
- Onboard Bluetooth remains physically disconnected. PB6/PB7 are released by
  the gray conversion, but K230 remains a separate later hardware route.
- PB16/PB17 are dedicated to CD4051 AD0/OUT; moving back to LF04 is a separate
  hardware reroute.
- The CCD connector must remain empty because PA22 is D36A EN.
- MS42CG uses 3.3 V only. Never power it from 5 V.
- Motor and stepper return currents must not flow through C07A or K230 signal
  ground wiring.

## Facts that still require hardware measurement

- S27F versus S28A base-board silk and C07A V1.1 PB18/PB19 availability.
- D36A EN polarity, microstep setting, current limit and reset-safe pull.
- MS42CG direction, counts per revolution, PWM period and absolute-angle
  alignment.
- MG513XP28 encoder CPR, reduction ratio, phase order and stall current.
- CD4051 address-input 3.3 V logic margin, 1000-frame settle-time comparison
  and OUT voltage margin still need instrumented verification. The black/white
  response, physical R0..R7 order and centre coordinates have been measured:
  R0..R7 = -35,-25,-15,-5,+5,+15,+25,+35 mm; white is 171..174 ADC and black
  is 4095 ADC. The archived calibration CRC is 0x74D9.
- P03B load, ripple and temperature margin with the controller and K230.
- Battery capacity/C rating, fuse value and wire gauge from measured peaks.
- K230 memory, power, UART, camera and simultaneous H.264/RTSP two-hour test.
- Final vehicle envelope, centre of gravity and camera/beam geometry.
