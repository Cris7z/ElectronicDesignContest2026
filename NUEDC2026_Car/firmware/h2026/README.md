# H2026 firmware scaffold

This directory is deliberately independent of the old generic-car BSP.

- `ball_link.*`: checked binary state frames from the 01Studio CanMV K230.
- `ball_balance.*`: ball-position outer loop; outputs a beam-angle target.
- `linefollower_8ch.*`: the primary HiWonder LineFollower_8CH v1.0 infrared
  line sensor. It defines the documented I2C register map, parses eight
  little-endian analog/threshold values, and produces a weighted line error
  plus lost/all-line flags from register 5.
- `lf04.*`: retained fallback processing for a separate four-channel LF04.

The next hardware-backed layer must provide:

1. D157B four-input motor drive with PB2/PB3 fixed to TIMG6, plus two
   GPIO-decoded wheel encoders;
2. TIMA1 STEP generation, D36A STEP/DIR/EN drive, TIMG8 QEI and
   TIMG12 PWM capture;
3. UART1 RX on PB7 after disconnecting the onboard Bluetooth module;
4. HiWonder LineFollower_8CH on I2C0 shared with the installed MPU6050:
   SDA=PA0/H5 pin 4, SCL=PA1/H5 pin 3, 5 V=H5 pin 1 and GND=H5 pin 2.
   Use a split harness so the MPU6050 remains fitted. The sensor uses the
   fixed 7-bit address 0x5D; read register 5 for the fast digital state and
   registers 6..21/22..37 for low-rate analog/threshold diagnostics. Start at
   100 kHz. Before connecting PA0/PA1, power the sensor alone and measure the
   SDA/SCL idle voltage. PA0/PA1 are 5 V-tolerant open-drain pins, but the
   MPU6050 module on the same bus is not confirmed 5 V-safe; if the idle
   voltage is above about 3.6 V, isolate the sensor with a bidirectional I2C
   level shifter. Confirm state polarity and bit 0..7 left/right order on the
   real module before enabling motion. Gate motion on `state_valid`; a failed
   state read invalidates the control sample even if a later diagnostic read
   succeeds;
5. a TIMG7-based 5 ms inner beam-angle/control tick and all safety interlocks.

Do not merge the old `car_mspm0g3507.syscfg` pin assignments. Generate a new
S28A-specific SysConfig only after checking the physical board revision and
the installed D157B mapping. The stepper driver is confirmed as D36A; still
bench-check its EN active level, microstep setting, current limit, and safe
disabled state before applying motor power. Add an external pull resistor that
keeps EN disabled while PA22 is high-impedance during reset. PA14 is shared
with the S28A H12/TB6612 socket and PA22 with the CCD connector, so both
unused connectors must remain empty or be electrically isolated. U3 remains
available for the LF04 rule fallback; the CCD connector remains empty because
PA22 is used for D36A EN.

The WHEELTEC LF04 example wires O1/O2/O3/O4 to PA27/PA12/PB16/PB17, which
are U3 pins 6/5/4/3. The BSP must map bit 0..3 back to O1..O4 and must not
assume U3 pin-number order equals the sensor's left-to-right order.
