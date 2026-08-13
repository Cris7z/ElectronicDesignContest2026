"""One-way K230 vision publisher for the STM32F103RCT6 ball controller."""

import time

try:
    from .vision_contract import VisionStatus
except ImportError:
    try:
        # CanMV starts main.py as a script from this directory.
        from vision_contract import VisionStatus
    except ImportError:
        # The live SD application's older three-field publisher has no
        # VisionSampleV1 module.  Its call path does not need this import.
        VisionStatus = None


_MISSING = object()


def _format_rct6_fields(sequence, position_mm, quality):
    """Return one ASCII record from primitive RCT6 measurement fields."""
    sequence = int(sequence) & 0xFF
    valid = position_mm is not None
    position_mm = int(position_mm) if valid else 0
    quality = int(quality) if valid else 0
    quality = max(0, min(100, quality))
    payload = "B,%d,%d,%d" % (sequence, position_mm, quality)
    checksum = 0
    for character in payload:
        checksum ^= ord(character)
    return "$%s*%02X\r\n" % (payload, checksum)


def format_rct6_frame(sample):
    """Return the ASCII frame consumed by USER/main_d36a_test.c."""
    if VisionStatus is None:
        raise RuntimeError("VisionSampleV1 formatting is unavailable on this image")
    valid = sample.status == VisionStatus.VALID and sample.x_mm is not None
    return _format_rct6_fields(
        sample.seq, sample.x_mm if valid else None, sample.quality if valid else 0
    )


class Rct6UartPublisher:
    """Publish current vision samples through UART1 without a worker thread.

    The CanMV AI+UART reference sends directly from the image loop.  This
    class follows that model while rate-limiting output to 40 Hz so the RCT6
    watchdog has margin.  Only calls carrying a new sample can transmit; no
    worker repeats an old sequence to keep the watchdog alive.  A failed or
    short write is recorded and the
    affected frame is discarded; there is deliberately no retry loop.
    """

    def __init__(
        self,
        baudrate=115200,
        period_ms=25,
        clock=None,
        ticks_diff=None,
        fpioa_cls=None,
        uart_cls=None,
        enabled=True,
    ):
        self.enabled = bool(enabled)
        self._ticks_ms = clock
        self._ticks_diff = ticks_diff
        self._period_ms = max(1, int(period_ms))
        self._last_attempt_ms = None
        self._pending_frame = None
        self._closed = False
        self._fpioa = None
        self._uart = None
        self.frames_sent = 0
        self.write_failures = 0
        self.short_writes = 0
        if not self.enabled:
            # Baseline mode: neither FPIOA nor the UART driver is touched.
            # It is used to isolate vision/RTSP from a suspected UART fault.
            return

        if fpioa_cls is None or uart_cls is None:
            from machine import FPIOA, UART

            if fpioa_cls is None:
                fpioa_cls = FPIOA
            if uart_cls is None:
                uart_cls = UART
        if self._ticks_ms is None:
            self._ticks_ms = time.ticks_ms
        if self._ticks_diff is None:
            self._ticks_diff = time.ticks_diff

        # Keep the FPIOA instance alive.  CanMV UART1 requires both routes,
        # even though GPIO4/RX1 is physically left unconnected in this link.
        self._fpioa = fpioa_cls()
        self._fpioa.set_function(3, fpioa_cls.UART1_TXD)
        self._fpioa.set_function(4, fpioa_cls.UART1_RXD)
        self._uart = uart_cls(
            uart_cls.UART1,
            baudrate=baudrate,
            bits=uart_cls.EIGHTBITS,
            parity=uart_cls.PARITY_NONE,
            stop=uart_cls.STOPBITS_ONE,
        )

    def publish(self, sample_or_sequence, position_mm=_MISSING, quality=_MISSING):
        """Offer one new sample and send at most one frame in this call.

        ``publish(sample)`` is used by the staged VisionSampleV1 application.
        ``publish(seq, position_mm, quality)`` keeps the live SD application
        compatible without changing its model or RTSP loop.
        """
        if self._closed or not self.enabled:
            return False
        if position_mm is _MISSING and quality is _MISSING:
            self._pending_frame = format_rct6_frame(sample_or_sequence)
        elif position_mm is not _MISSING and quality is not _MISSING:
            self._pending_frame = _format_rct6_fields(
                sample_or_sequence, position_mm, quality
            )
        else:
            raise TypeError("publish needs a sample or seq, position_mm, quality")
        now_ms = self._ticks_ms()
        if self._last_attempt_ms is not None and self._ticks_diff(
            now_ms, self._last_attempt_ms
        ) < self._period_ms:
            return False

        frame = self._pending_frame
        self._pending_frame = None
        self._last_attempt_ms = now_ms
        try:
            written = self._uart.write(frame.encode("ascii"))
        except BaseException:
            self.write_failures += 1
            return False
        if written == len(frame):
            self.frames_sent += 1
            return True

        # Do not complete a partial frame later: the next '$' lets the RCT6
        # parser resynchronise, and avoiding a retry keeps vision nonblocking.
        self.short_writes += 1
        return False

    def close(self):
        """Release UART resources once the image loop has stopped."""
        if self._closed:
            return
        self._closed = True
        self._pending_frame = None
        if self._uart is not None:
            try:
                self._uart.deinit()
            except BaseException:
                pass
