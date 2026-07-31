"""One-way K230 vision publisher for the STM32F103RCT6 ball controller."""

from vision_contract import VisionStatus


def format_rct6_frame(sample):
    """Return the ASCII frame consumed by USER/main_d36a_test.c."""
    sequence = int(sample.seq) & 0xFF
    valid = sample.status == VisionStatus.VALID and sample.x_mm is not None
    position_mm = int(sample.x_mm) if valid else 0
    quality = int(sample.quality) if valid else 0
    quality = max(0, min(100, quality))
    payload = "B,%d,%d,%d" % (sequence, position_mm, quality)
    checksum = 0
    for character in payload:
        checksum ^= ord(character)
    return "$%s*%02X\r\n" % (payload, checksum)


class Rct6UartPublisher:
    """K230 GPIO3/TX1 -> RCT6 PA3/USART2_RX at 115200 8N1."""

    def __init__(self, baudrate=115200):
        from machine import FPIOA, UART

        fpioa = FPIOA()
        fpioa.set_function(3, FPIOA.UART1_TXD)
        self._uart = UART(
            UART.UART1,
            baudrate=baudrate,
            bits=UART.EIGHTBITS,
            parity=UART.PARITY_NONE,
            stop=UART.STOPBITS_ONE,
        )

    def publish(self, sample):
        self._uart.write(format_rct6_frame(sample))
