# H2026 firmware scaffold

This directory is deliberately independent of the old generic-car BSP.

- `ball_link.*`: checked binary state frames from the 01Studio CanMV K230.
- `ball_balance.*`: ball-position outer loop; outputs a beam-angle target.
- `gray8_mux.*`: the primary YB-MVX05-V1.0 eight-channel digital line
  sensor. It sequences the three CD4051 address bits, accepts multiplexed OUT
  samples, and produces a weighted line error plus lost/all-dark flags.
- `lf04.*`: retained fallback processing for a separate four-channel LF04.

The next hardware-backed layer must provide:

1. D157B four-input motor drive with PB2/PB3 fixed to TIMG6, plus two
   GPIO-decoded wheel encoders;
2. TIMA1 STEP generation, D36A STEP/DIR/EN drive, TIMG8 QEI and
   TIMG12 PWM capture;
3. UART1 RX on PB7 after disconnecting the onboard Bluetooth module;
4. YB-MVX05 GPIO scan using AD2=PA27, AD1=PA12, AD0=PB16 and OUT=PB17.
   Drive the address, wait about 100 us, then sample OUT. The module requires
   5 V; S28A U3 pin 2 is only 3.3 V, so take 5 V separately and common GND.
   Measure the OUT high level and calibrate active polarity before enabling
   motion. Bit 0 through bit 7 must represent X1 through X8 from left to
   right;
5. a TIMG7-based 5 ms inner beam-angle/control tick and all safety interlocks.

Do not merge the old `car_mspm0g3507.syscfg` pin assignments. Generate a new
S28A-specific SysConfig only after checking the physical board revision and
the installed D157B mapping. The stepper driver is confirmed as D36A; still
bench-check its EN active level, microstep setting, current limit, and safe
disabled state before applying motor power. Add an external pull resistor that
keeps EN disabled while PA22 is high-impedance during reset. PA14 is shared
with the S28A H12/TB6612 socket and PA22 with the CCD connector, so both
unused connectors must remain empty or be electrically isolated. PA27 is
also shared with the CCD interface, so CCD and the primary gray sensor cannot
be connected simultaneously.
