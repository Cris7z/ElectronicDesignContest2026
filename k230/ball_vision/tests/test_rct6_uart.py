import unittest

from rct6_uart import format_rct6_frame
from vision_contract import VisionSampleV1, VisionStatus


class Rct6UartFrameTest(unittest.TestCase):
    def test_valid_sample_uses_signed_millimetres_and_quality(self):
        sample = VisionSampleV1(258, 1000, -17, 73, VisionStatus.VALID)
        self.assertEqual(format_rct6_frame(sample), "$B,2,-17,73*73\r\n")

    def test_invalid_sample_cannot_refresh_the_rct6_watchdog(self):
        sample = VisionSampleV1(3, 1000, None, 0, VisionStatus.LOST)
        self.assertEqual(format_rct6_frame(sample), "$B,3,0,0*5D\r\n")


if __name__ == "__main__":
    unittest.main()
