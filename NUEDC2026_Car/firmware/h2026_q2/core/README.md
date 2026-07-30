# H2026 Q2 control core

This directory is a hardware-independent controller. It does not include
MSPM0 peripheral setup or measured chassis constants.

## Integration contract

- Call `h2026_q2_step()` exactly once per 5 ms control tick.
- Pass one complete `h2026_q2_line_sensor_frame_t` per tick. Its calibrated
  `strength[8]` values drive the weighted centroid; its hysteretic `raw_bits`
  drive the marker/FSM classifier. Mark failed, stale or over-budget scans
  `valid=false`; their bytes never enter marker or PD logic.
- Pass atomic cumulative encoder snapshots. A scheduler overrun must be
  handled as an application-level safety fault; do not feed an encoder delta
  spanning multiple ticks as though it covered one 5 ms tick.
- Apply `left_signed_duty` and `right_signed_duty` only when `brake=false`.
  When a non-emergency fault first occurs, the core can request zero-duty
  coasting before braking. An emergency stop requests braking immediately.
- `h2026_q2_reset()` is the only supported way to return a completed or
  faulted controller to `IDLE`.

## Required commissioning data

There are intentionally no production defaults. Before enabling motors,
inject a CRC-valid eight-channel white/black calibration and measured sensor
coordinates, left and right metres per encoder count, encoder signs, track
width, duty/feed-forward values, speed-loop
gains, marker shape limits, motion limits, timeout values, stop offset, and
the distance-indexed curvature profile. Host-test fixture numbers are
synthetic and must not be copied into vehicle firmware.

The start marker is captured as a normalized signature. A later finish
candidate must be a centered, continuous `WIDE` block, match the expanded
start signature, and remain within the configured start-relative active-bit
range. `0xFF` is classified as `ALL`, never as a marker.
