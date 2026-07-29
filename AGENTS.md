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
  four-wire OLED, BLS start button, status LED and D103A/TB6612 motor module.
- Modules supplied with the controller but removed for the current build: the
  onboard Bluetooth module.  The separately supplied D157B/AT8236 is not in
  the chassis drive path.
- Chassis drive: integrated D103A/TB6612 driving two MG513XP28 12 V encoder
  motors on a 32 cm by 24 cm three-wheel differential chassis.
- Main line sensor: HiWonder LineFollower_8CH v1.0, eight infrared probes,
  5 V/85 mA.  The current live-hardware route uses its native 115200 UART
  state protocol; the 0x5D I2C route remains a disconnected fallback only.
- Rule fallback line sensor: LF04 four-channel infrared module on U3.
- Beam drive: D36A plus a 42-size stepper and MS42CG A/B/PWM encoder.
- Vision and video: 01Studio CanMV K230 standard board, planned 1 GB version,
  with the bundled GC2093 70-degree 24-pin camera. The K230 is planned, not
  yet hardware-validated.
- Mechanics: 25 cm PPR beam, about 1 cm steel ball, hinge and transmission.

## Frozen MSPM0 resource ownership

| Function | MSPM0 resource | Hardware endpoint |
|---|---|---|
| Left chassis motor | PB3/TIMA1_CCP1 + PA16/PA17 | TB6612 B channel; firmware positive is remapped toward physical forward |
| Right chassis motor | PB2/TIMA1_CCP0 + PA14/PA13 | TB6612 A channel; firmware positive is remapped toward physical forward |
| Left wheel encoder A/B | PA25/PA26 | GPIO quadrature; physical-forward sign = -1 |
| Right wheel encoder A/B | PB20/PB24 | GPIO quadrature; physical-forward sign = +1 |
| MPU6050 I2C0 SDA/SCL | PA0/PA1 | H5 original module only |
| HiWonder 8CH UART | PB6 TX/PB7 RX, UART1 | H8 pins 2/3; sensor RX/TX, 115200 |
| MPU6050 interrupt | PA7 | H5 pin 8 |
| LF04 O1/O2/O3/O4 | PA27/PA12/PB16/PB17 | U3 pins 6/5/4/3 |
| OLED SCL/SDA/RST/DC | PA28/PA31/PB14/PB15 | Installed OLED |
| Start button / status LED | PA18/PB9 | BLS / LED |
| Control tick | TIMG7 | No external pin |
| Debug UART | PA10 TX / PA11 RX, UART0 | 115200; separate from the line sensor |
| K230 state receive | PB7 / UART1_RX | Planned route; unavailable while the HiWonder UART uses H8 |
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
  pin 8=PA7/MPU6050 INT. It remains exclusively with the installed MPU6050.
- Q2 HiWonder connector (current UART trial): H8 pin 5=5 V, pin 4=GND,
  pin 2=PB6/UART1_TX to sensor RX, pin 3=PB7/UART1_RX from sensor TX.  The
  removable Bluetooth module and K230 must remain disconnected; disconnect
  the U3 SDA/SCL harness.  Firmware selects manual mode with byte 0 then
  sends byte 1 and receives the one-byte S1..S8 state at 115200 8N1.
  Earlier H8 continuity tests were negative, so this route remains pending an
  end-to-end UART response count before it is accepted as electrically valid.
- HiWonder sensor connector: the separate I2C header is unplugged for the
  current trial.  The old 0x5D register protocol remains documented only as
  a fallback; no control-loop transaction may use it while UART is selected.
- `linefollower_8ch.state_valid` is mandatory for motion. A failed state read
  invalidates the line sample; analog or threshold diagnostic reads must not
  restore it.
- LF04 follows the WHEELTEC example, not U3 pin-number order:
  O1/O2/O3/O4=PA27/PA12/PB16/PB17.
- K230 to MSPM0 is a later, one-way route:
  K230 GPIO11/TX2 -> PB7/UART1_RX plus common GND. GPIO12/RX2 and the K230
  3V3 pin are not connected. It is a later hardware route and remains
  unplugged during Q2 commissioning.
- K230 state frames are 17 bytes as documented in
  `硬件及接线清单.md`; stale/invalid frames must drive the system to a safe
  state. RTSP is never part of the control loop.

## Mandatory conflict rules

- Keep MPU6050 on PA0/PA1/PA7. Do not copy the D36A vendor example that uses
  PA0/PA1 for QEI.
- D103A/TB6612 remains installed for chassis drive.  Its PA13/PA14 direction
  pins conflict with the current D36A/MS42CG plan, so D36A must remain
  disconnected until its direction and absolute-angle pins are reassigned.
- Onboard Bluetooth must be physically disconnected before Q2 HiWonder or
  K230 uses H8 PB6/PB7.
- PB16/PB17 are disconnected from the HiWonder while UART is selected;
  moving back to its I2C fallback or to LF04 is a separate hardware reroute.
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
- HiWonder H8 PB6/PB7 UART end-to-end response is pending: confirm that the
  response counter rises and each physical S1..S8 occlusion changes its bit.
- P03B load, ripple and temperature margin with the controller and K230.
- Battery capacity/C rating, fuse value and wire gauge from measured peaks.
- K230 memory, power, UART, camera and simultaneous H.264/RTSP two-hour test.
- Final vehicle envelope, centre of gravity and camera/beam geometry.
