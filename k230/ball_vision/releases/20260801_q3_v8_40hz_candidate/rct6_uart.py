"""Full-duplex K230/RCT6 link: fresh ball samples out, RTSP commands in."""

import _thread
import time


TX_PERIOD_MS = 25
SERVICE_PERIOD_MS = 5


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


def _video_mode_from_line(line):
    """Return False/True only for a valid `$V,0/1*XOR` command."""
    line = line.strip()
    if len(line) != 7 or line[0] != "$" or line[4] != "*":
        return None
    payload = line[1:4]
    if payload != "V,0" and payload != "V,1":
        return None
    checksum = 0
    for character in payload:
        checksum ^= ord(character)
    try:
        received = int(line[5:7], 16)
    except ValueError:
        return None
    if received != checksum:
        return None
    return payload == "V,1"


class Rct6UartPublisher:
    """GPIO3 TX -> PA10 RX; PA9 TX -> GPIO4 RX, 115200 8N1.

    The vision thread replaces a one-frame mailbox.  The UART worker sends a
    mailbox generation at most once and no faster than 40 Hz.  It therefore
    cannot repeat an old sequence to keep the RCT6 watchdog alive.
    """

    def __init__(self, baudrate=115200):
        from machine import FPIOA, UART

        self._fpioa = FPIOA()
        self._fpioa.set_function(3, FPIOA.UART1_TXD, ie=1, oe=1)
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
        self._frame_generation = 0
        self._mode_lock = _thread.allocate_lock()
        self._pending_video_mode = None
        self._last_received_video_mode = None
        self._rx_buffer = ""
        _thread.start_new_thread(self._service_loop, ())

    def publish(self, sequence, position_mm, quality):
        """Replace the mailbox with exactly one newly captured sample."""
        frame = _frame(sequence, position_mm, quality)
        self._frame_lock.acquire()
        try:
            self._latest_frame = frame
            self._frame_generation += 1
        finally:
            self._frame_lock.release()

    def take_video_mode_request(self):
        """Called by the vision thread; RTSP APIs never run in UART thread."""
        self._mode_lock.acquire()
        try:
            requested = self._pending_video_mode
            self._pending_video_mode = None
            return requested
        finally:
            self._mode_lock.release()

    def _poll_rx(self):
        try:
            if self._uart.any() <= 0:
                return
            chunk = self._uart.read()
        except BaseException:
            return
        if not chunk:
            return
        try:
            text = chunk.decode("ascii")
        except BaseException:
            return
        self._rx_buffer += text
        if len(self._rx_buffer) > 256:
            self._rx_buffer = self._rx_buffer[-128:]
        while "\n" in self._rx_buffer:
            line, self._rx_buffer = self._rx_buffer.split("\n", 1)
            requested = _video_mode_from_line(line)
            if requested is None or requested == self._last_received_video_mode:
                continue
            self._last_received_video_mode = requested
            self._mode_lock.acquire()
            try:
                self._pending_video_mode = requested
            finally:
                self._mode_lock.release()

    def _service_loop(self):
        last_sent_generation = 0
        last_tx_ms = time.ticks_add(time.ticks_ms(), -TX_PERIOD_MS)
        while True:
            self._poll_rx()
            now_ms = time.ticks_ms()
            if time.ticks_diff(now_ms, last_tx_ms) >= TX_PERIOD_MS:
                self._frame_lock.acquire()
                try:
                    frame = self._latest_frame
                    generation = self._frame_generation
                finally:
                    self._frame_lock.release()
                if frame is not None and generation != last_sent_generation:
                    # Consume before attempting the write.  A failed write is
                    # never retried as though it were a new camera sample.
                    last_sent_generation = generation
                    last_tx_ms = now_ms
                    try:
                        self._uart.write(frame)
                    except BaseException:
                        pass
            time.sleep_ms(SERVICE_PERIOD_MS)
