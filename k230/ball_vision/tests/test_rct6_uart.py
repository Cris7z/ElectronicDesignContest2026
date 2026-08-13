import unittest

from k230.ball_vision.rct6_uart import Rct6UartPublisher, format_rct6_frame
from k230.ball_vision.vision_contract import VisionSampleV1, VisionStatus


class FakeFpioa:
    UART1_TXD = "uart1_txd"
    UART1_RXD = "uart1_rxd"

    def __init__(self):
        self.calls = []

    def set_function(self, pin, function, **kwargs):
        self.calls.append((pin, function, kwargs))


class FakeUart:
    UART1 = 1
    EIGHTBITS = 8
    PARITY_NONE = 0
    STOPBITS_ONE = 1

    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs
        self.writes = []
        self.results = []
        self.deinit_count = 0

    def write(self, data):
        self.writes.append(data)
        result = self.results.pop(0) if self.results else len(data)
        if isinstance(result, BaseException):
            raise result
        return result

    def deinit(self):
        self.deinit_count += 1


class Clock:
    def __init__(self, now=0):
        self.now = now

    def __call__(self):
        return self.now


def sample(seq, position=-17, quality=73, status=VisionStatus.VALID):
    value = position if status == VisionStatus.VALID else None
    return VisionSampleV1(seq, 1000, value, quality, status)


def make_publisher(clock=None, ticks_diff=None):
    clock = clock or Clock()
    publisher = Rct6UartPublisher(
        clock=clock,
        ticks_diff=ticks_diff or (lambda newer, older: newer - older),
        fpioa_cls=FakeFpioa,
        uart_cls=FakeUart,
    )
    return publisher, clock


class Rct6UartFrameTest(unittest.TestCase):
    def test_valid_sample_uses_signed_millimetres_and_quality(self):
        self.assertEqual(format_rct6_frame(sample(258)), "$B,2,-17,73*73\r\n")

    def test_invalid_sample_cannot_refresh_the_rct6_watchdog(self):
        lost = sample(3, quality=0, status=VisionStatus.LOST)
        self.assertEqual(format_rct6_frame(lost), "$B,3,0,0*5D\r\n")

    def test_legacy_three_field_call_uses_the_same_frame_protocol(self):
        publisher, _ = make_publisher()
        self.assertTrue(publisher.publish(258, -17, 73))
        self.assertEqual(publisher._uart.writes, [b"$B,2,-17,73*73\r\n"])

    def test_uses_official_uart1_routes_without_forcing_io_flags(self):
        publisher, _ = make_publisher()
        self.assertEqual(
            publisher._fpioa.calls,
            [(3, FakeFpioa.UART1_TXD, {}), (4, FakeFpioa.UART1_RXD, {})],
        )
        self.assertEqual(publisher._uart.args, (FakeUart.UART1,))
        self.assertEqual(
            publisher._uart.kwargs,
            {
                "baudrate": 115200,
                "bits": FakeUart.EIGHTBITS,
                "parity": FakeUart.PARITY_NONE,
                "stop": FakeUart.STOPBITS_ONE,
            },
        )

    def test_disabled_mode_never_initialises_or_writes_uart(self):
        publisher = Rct6UartPublisher(enabled=False)
        self.assertIsNone(publisher._fpioa)
        self.assertIsNone(publisher._uart)
        self.assertFalse(publisher.publish(sample(1)))
        publisher.close()

    def test_rate_limits_and_sends_the_latest_sample_only(self):
        publisher, clock = make_publisher()
        self.assertTrue(publisher.publish(sample(1)))
        clock.now = 24
        self.assertFalse(publisher.publish(sample(2)))
        clock.now = 25
        self.assertTrue(publisher.publish(sample(3)))
        self.assertEqual(
            publisher._uart.writes,
            [b"$B,1,-17,73*70\r\n", b"$B,3,-17,73*72\r\n"],
        )

    def test_short_write_is_dropped_without_a_busy_retry(self):
        publisher, clock = make_publisher()
        expected = format_rct6_frame(sample(1)).encode("ascii")
        publisher._uart.results = [len(expected) - 1, len(expected)]
        self.assertFalse(publisher.publish(sample(1)))
        self.assertEqual(publisher.short_writes, 1)
        clock.now = 24
        self.assertFalse(publisher.publish(sample(2)))
        self.assertEqual(len(publisher._uart.writes), 1)
        clock.now = 25
        self.assertTrue(publisher.publish(sample(3)))
        self.assertEqual(len(publisher._uart.writes), 2)

    def test_write_exception_is_contained_and_a_new_frame_can_recover(self):
        publisher, clock = make_publisher()
        publisher._uart.results = [OSError("uart busy")]
        self.assertFalse(publisher.publish(sample(1)))
        self.assertEqual(publisher.write_failures, 1)
        clock.now = 25
        self.assertTrue(publisher.publish(sample(2)))
        self.assertEqual(publisher.frames_sent, 1)

    def test_ticks_diff_handles_clock_wrap(self):
        def wrapped_diff(newer, older):
            return (newer - older) % 128

        clock = Clock(120)
        publisher, _ = make_publisher(clock, wrapped_diff)
        self.assertTrue(publisher.publish(sample(1)))
        clock.now = 42
        self.assertTrue(publisher.publish(sample(2)))

    def test_close_is_idempotent_and_disables_future_publishes(self):
        publisher, _ = make_publisher()
        publisher.close()
        publisher.close()
        self.assertEqual(publisher._uart.deinit_count, 1)
        self.assertFalse(publisher.publish(sample(1)))


if __name__ == "__main__":
    unittest.main()
