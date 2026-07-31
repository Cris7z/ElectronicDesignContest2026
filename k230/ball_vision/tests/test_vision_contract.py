import unittest

from k230.ball_vision.vision_contract import (
    MeasurementTracker,
    PiecewiseCalibration,
    VisionStatus,
)


def calibration():
    return PiecewiseCalibration.from_dict(
        {
            "schema_version": 1,
            "installation_id": "unit-test",
            "model_sha256": "0" * 64,
            "orientation": {
                "origin": "rod_center",
                "positive_direction": "hinge_to_actuator",
            },
            "roi": [0, 0, 1000, 200],
            "points": [
                {"pixel_x": 100, "x_mm": -100},
                {"pixel_x": 500, "x_mm": 0},
                {"pixel_x": 900, "x_mm": 100},
            ],
        }
    )


def candidate(pixel_x=500, confidence=0.95, box_w=20, box_h=20):
    return {
        "pixel_x": pixel_x,
        "pixel_y": 100,
        "confidence": confidence,
        "box_w": box_w,
        "box_h": box_h,
    }


def tracker():
    return MeasurementTracker(
        calibration(),
        [0, 0, 1000, 200],
        max_speed_mm_s=300,
        temporal_margin_mm=5,
    )


class CalibrationTests(unittest.TestCase):
    def test_interpolates_without_endpoint_clamp(self):
        cal = calibration()
        self.assertEqual(cal.map_pixel(500), 0)
        self.assertEqual(cal.map_pixel(300), -50)
        self.assertIsNone(cal.map_pixel(99))
        self.assertIsNone(cal.map_pixel(901))

    def test_rejects_non_monotonic_physical_axis(self):
        document = {
            "schema_version": 1,
            "installation_id": "bad",
            "model_sha256": "0" * 64,
            "orientation": {},
            "points": [
                {"pixel_x": 10, "x_mm": -10},
                {"pixel_x": 20, "x_mm": 10},
                {"pixel_x": 30, "x_mm": 0},
            ],
        }
        with self.assertRaises(ValueError):
            PiecewiseCalibration.from_dict(document)


class TrackerTests(unittest.TestCase):
    def test_requires_three_valid_frames_after_boot(self):
        subject = tracker()
        self.assertEqual(subject.process(0, candidate()).status, VisionStatus.TEMPORAL_REJECT)
        self.assertEqual(subject.process(20, candidate()).status, VisionStatus.TEMPORAL_REJECT)
        result = subject.process(40, candidate())
        self.assertEqual(result.status, VisionStatus.VALID)
        self.assertEqual(result.x_mm, 0)
        self.assertGreater(result.quality, 0)

    def test_invalid_never_reuses_prior_coordinate_and_enters_lost(self):
        subject = tracker()
        subject.process(0, candidate())
        subject.process(20, candidate())
        self.assertEqual(subject.process(40, candidate()).status, VisionStatus.VALID)
        first_missing = subject.process(60, None)
        self.assertEqual(first_missing.status, VisionStatus.NO_TARGET)
        self.assertIsNone(first_missing.x_mm)
        subject.process(80, None)
        lost = subject.process(100, None)
        self.assertEqual(lost.status, VisionStatus.LOST)
        self.assertIsNone(lost.x_mm)

    def test_temporal_jump_is_rejected(self):
        subject = tracker()
        subject.process(0, candidate(500))
        subject.process(20, candidate(500))
        self.assertEqual(subject.process(40, candidate(500)).status, VisionStatus.VALID)
        jump = subject.process(60, candidate(900))
        self.assertEqual(jump.status, VisionStatus.TEMPORAL_REJECT)
        self.assertIsNone(jump.x_mm)

    def test_sequence_wraps_u32(self):
        subject = tracker()
        subject.seq = 0xFFFFFFFF
        first = subject.process(0, candidate())
        second = subject.process(20, candidate())
        self.assertEqual(first.seq, 0xFFFFFFFF)
        self.assertEqual(second.seq, 0)

    def test_outside_calibration_is_invalid(self):
        subject = tracker()
        result = subject.process(0, candidate(50))
        self.assertEqual(result.status, VisionStatus.OUT_OF_RANGE)
        self.assertIsNone(result.x_mm)


if __name__ == "__main__":
    unittest.main()
