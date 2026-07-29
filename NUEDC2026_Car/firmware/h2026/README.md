# H2026 firmware scaffold

This directory is deliberately independent of the old generic-car BSP.

- `ball_link.*`: checked binary state frames from the 01Studio CanMV K230.
- `ball_balance.*`: ball-position outer loop; outputs a beam-angle target.
- `lf04.*`: the purchased four-channel digital line sensor.

The next hardware-backed layer must provide:

1. D157B four-input motor drive with PB2/PB3 fixed to TIMG6, plus two
   GPIO-decoded wheel encoders;
2. TIMA1 STEP generation, D36A STEP/DIR/EN drive, TIMG8 QEI and
   TIMG12 PWM capture;
3. UART1 RX on PB7 after disconnecting the onboard Bluetooth module;
4. LF04 GPIO reads on PA27/PA12/PB16/PB17;
5. a TIMG7-based 5 ms inner beam-angle/control tick and all safety interlocks.

Do not merge the old `car_mspm0g3507.syscfg` pin assignments. Generate a new
S28A-specific SysConfig only after checking the physical board revision and
the installed D157B mapping. The stepper driver is confirmed as D36A; still
bench-check its EN active level, microstep setting, current limit, and safe
disabled state before applying motor power. Add an external pull resistor that
keeps EN disabled while PA22 is high-impedance during reset. PA14 is shared
with the S28A H12/TB6612 socket and PA22 with the CCD connector, so both
unused connectors must remain empty or be electrically isolated.
