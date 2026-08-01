"""One-way K230 measurement output for the STM32F103RCT6 controller."""

import _thread
import time


def _frame(sequence, position_mm, quality):
    """Build the RCT6 `$B,SEQ,POS_MM,CONF*XOR` UART record."""
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


class Rct6UartPublisher:
    """GPIO3 / UART1_TXD -> RCT6 PA3 / USART2_RX, 115200 8N1."""

    def __init__(self, baudrate=115200):
        from machine import FPIOA, UART

        # The image loop calls gc.collect() every frame, so retain this
        # mapping object for the lifetime of the UART publisher.
        self._fpioa = FPIOA()
        # Match CanMV's proven AI UART example: explicitly enable the FPIOA
        # input/output path instead of relying on board-default drive state.
        self._fpioa.set_function(3, FPIOA.UART1_TXD, ie=1, oe=1)
        # This CanMV v1.8 UART driver asserts that both directions are
        # present in FPIOA. GPIO4 is the board's documented UART1 RX pin;
        # it is intentionally left electrically unconnected here.
        self._fpioa.set_function(4, FPIOA.UART1_RXD, ie=1, oe=1)
        self._uart = UART(
            UART.UART1,
            baudrate=baudrate,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )
        self._frame_lock = _thread.allocate_lock()
        self._latest_frame = None
        _thread.start_new_thread(self._tx_loop, ())

    def publish(self, sequence, position_mm, quality):
        # The vision/LCD loop only replaces one small shared value.  A stalled
        # UART driver can no longer freeze detection or RTSP.
        frame = _frame(sequence, position_mm, quality)
        self._frame_lock.acquire()
        try:
            self._latest_frame = frame
        finally:
            self._frame_lock.release()

    def _tx_loop(self):
        while True:
            self._frame_lock.acquire()
            try:
                frame = self._latest_frame
            finally:
                self._frame_lock.release()
            if frame is not None:
                try:
                    self._uart.write(frame)
                except BaseException:
                    pass
            time.sleep_ms(50)
