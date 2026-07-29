"""
01Studio CanMV K230 + onboard GC2093 ball-tracker scaffold for CanMV v1.8.

This standalone bring-up script emits ball state on UART2.  The final car
build will merge the detector with a display-free, direct-VENC RTSP pipeline
so that detection and H.264 streaming share one sensor instance.

Before closed-loop use, tune the ROI/radius limits, fix camera exposure, and
replace the two endpoint calibration constants with measured values.
"""
import os
import time
from machine import FPIOA, UART
from media.media import MediaManager
from media.sensor import Sensor


FRAME_TYPE = 0x10
FLAG_VALID = 0x01
FLAG_CALIBRATED = 0x02

# Placeholder QVGA geometry. The beam should fill at least 75% of the width.
BEAM_ROI = (20, 80, 280, 80)
BEAM_LEFT_PX = 28.0
BEAM_RIGHT_PX = 292.0
BEAM_HALF_LENGTH_MM = 125.0
CALIBRATION_CONFIRMED = False
MIN_RADIUS_PX = 3
MAX_RADIUS_PX = 10


def crc16_ccitt(data, start, length):
    crc = 0xFFFF
    for i in range(start, start + length):
        crc ^= data[i] << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def put_u16_le(buf, offset, value):
    value &= 0xFFFF
    buf[offset] = value & 0xFF
    buf[offset + 1] = (value >> 8) & 0xFF


def put_u32_le(buf, offset, value):
    value &= 0xFFFFFFFF
    for i in range(4):
        buf[offset + i] = (value >> (8 * i)) & 0xFF


def make_frame(sequence, timestamp_ms, x_mm, vx_mm_s, confidence, flags):
    frame = bytearray(17)
    frame[0] = 0xAA
    frame[1] = 0x55
    frame[2] = FRAME_TYPE
    frame[3] = sequence & 0xFF
    put_u32_le(frame, 4, timestamp_ms)
    put_u16_le(frame, 8, int(x_mm))
    put_u16_le(frame, 10, int(vx_mm_s))
    frame[12] = max(0, min(255, int(confidence)))
    frame[13] = flags & 0xFF
    crc = crc16_ccitt(frame, 2, 12)
    put_u16_le(frame, 14, crc)
    frame[16] = 0x0D
    return frame


def pixel_to_mm(pixel_x):
    span = BEAM_RIGHT_PX - BEAM_LEFT_PX
    normalised = (pixel_x - BEAM_LEFT_PX) / span
    return (normalised * 2.0 - 1.0) * BEAM_HALF_LENGTH_MM


def choose_circle(circles, predicted_x_px):
    best = None
    best_cost = 1000000.0
    roi_centre_y = BEAM_ROI[1] + 0.5 * BEAM_ROI[3]
    for circle in circles:
        dx = circle.x() - predicted_x_px
        dy = circle.y() - roi_centre_y
        cost = dx * dx + 4.0 * dy * dy
        if cost < best_cost:
            best = circle
            best_cost = cost
    return best


def main():
    fpioa = FPIOA()
    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)
    uart = UART(UART.UART2, baudrate=115200, bits=UART.EIGHTBITS,
                parity=UART.PARITY_NONE, stop=UART.STOPBITS_ONE)

    # Sensor() defaults to the standard board's onboard CSI2 GC2093.
    sensor = Sensor(width=1280, height=960)
    sensor.reset()
    sensor.set_framesize(Sensor.QVGA)
    sensor.set_pixformat(Sensor.RGB565)
    MediaManager.init()
    sensor.run()

    sequence = 0
    x_hat_mm = 0.0
    v_hat_mm_s = 0.0
    last_ms = time.ticks_ms()
    last_valid_ms = last_ms

    try:
        while True:
            os.exitpoint()
            now_ms = time.ticks_ms()
            dt_s = max(0.005, min(0.100,
                       time.ticks_diff(now_ms, last_ms) * 0.001))
            last_ms = now_ms

            x_hat_mm += v_hat_mm_s * dt_s
            predicted_px = BEAM_LEFT_PX + (
                (x_hat_mm / BEAM_HALF_LENGTH_MM + 1.0) * 0.5 *
                (BEAM_RIGHT_PX - BEAM_LEFT_PX))

            image = sensor.snapshot()
            circles = image.find_circles(
                roi=BEAM_ROI,
                threshold=3500,
                x_margin=8,
                y_margin=6,
                r_margin=4,
                r_min=MIN_RADIUS_PX,
                r_max=MAX_RADIUS_PX,
                r_step=1)
            circle = choose_circle(circles, predicted_px)

            valid = circle is not None
            confidence = 0
            if valid:
                measurement_mm = pixel_to_mm(circle.x())
                residual = measurement_mm - x_hat_mm
                x_hat_mm += 0.65 * residual
                v_hat_mm_s += 0.12 * residual / dt_s
                x_hat_mm = max(-140.0, min(140.0, x_hat_mm))
                v_hat_mm_s = max(-2000.0, min(2000.0, v_hat_mm_s))
                last_valid_ms = now_ms
                confidence = 220
            elif time.ticks_diff(now_ms, last_valid_ms) > 100:
                v_hat_mm_s *= 0.8

            flags = FLAG_CALIBRATED if CALIBRATION_CONFIRMED else 0
            if valid:
                flags |= FLAG_VALID
            uart.write(make_frame(sequence, now_ms, x_hat_mm, v_hat_mm_s,
                                  confidence, flags))
            sequence = (sequence + 1) & 0xFF
    finally:
        sensor.stop()
        MediaManager.deinit()
        uart.deinit()


main()
