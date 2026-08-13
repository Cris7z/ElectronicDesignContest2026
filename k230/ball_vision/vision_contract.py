"""Host-testable Stage 6 measurement contract.

This module intentionally has no K230 imports.  The same code runs in CPython
unit tests and in CanMV MicroPython, so calibration and invalid-state behavior
cannot silently diverge between the workstation and the board.
"""

VERSION = 1


class VisionStatus:
    VALID = "VALID"
    NO_TARGET = "NO_TARGET"
    LOW_CONFIDENCE = "LOW_CONFIDENCE"
    OUT_OF_RANGE = "OUT_OF_RANGE"
    TEMPORAL_REJECT = "TEMPORAL_REJECT"
    LOST = "LOST"
    FAULT = "FAULT"


def _clamp(value, low, high):
    return max(low, min(high, value))


class VisionSampleV1:
    """One result for one captured camera frame.

    `x_mm` is None unless status is VALID.  The later Stage 7 wire protocol
    must carry this same semantic rule rather than inventing a stale-value
    fallback.
    """

    def __init__(self, seq, capture_ms, x_mm, quality, status):
        self.version = VERSION
        self.seq = int(seq) & 0xFFFFFFFF
        self.capture_ms = int(capture_ms) & 0xFFFFFFFF
        self.x_mm = None if x_mm is None else int(round(x_mm))
        self.quality = int(_clamp(int(round(quality)), 0, 100))
        self.status = status
        if self.status != VisionStatus.VALID:
            self.x_mm = None
            self.quality = 0

    def as_dict(self):
        return {
            "version": self.version,
            "seq": self.seq,
            "capture_ms": self.capture_ms,
            "x_mm": self.x_mm,
            "quality": self.quality,
            "status": self.status,
        }


class PiecewiseCalibration:
    """Monotonic pixel-to-millimetre table with no endpoint clamping."""

    def __init__(self, points, installation_id, model_sha256, orientation, roi=None):
        if len(points) < 2:
            raise ValueError("calibration needs at least two points")
        self.points = sorted(
            [(float(item["pixel_x"]), float(item["x_mm"])) for item in points],
            key=lambda item: item[0],
        )
        self.installation_id = installation_id
        self.model_sha256 = model_sha256
        self.orientation = orientation
        self.roi = roi
        self._validate()

    @classmethod
    def from_dict(cls, document):
        if document.get("schema_version") != 1:
            raise ValueError("unsupported calibration schema")
        if document.get("example_only"):
            raise ValueError("example calibration cannot be used on the vehicle")
        return cls(
            document["points"],
            document["installation_id"],
            document["model_sha256"],
            document["orientation"],
            document.get("roi"),
        )

    def _validate(self):
        x_direction = None
        previous_px, previous_mm = self.points[0]
        for pixel_x, x_mm in self.points[1:]:
            if pixel_x <= previous_px:
                raise ValueError("calibration pixel_x values must increase")
            delta = x_mm - previous_mm
            if delta == 0:
                raise ValueError("calibration x_mm values must be monotonic")
            direction = 1 if delta > 0 else -1
            if x_direction is None:
                x_direction = direction
            elif direction != x_direction:
                raise ValueError("calibration x_mm values must not reverse")
            previous_px, previous_mm = pixel_x, x_mm

    def map_pixel(self, pixel_x):
        """Return None outside the calibrated interval; never clamp."""
        pixel_x = float(pixel_x)
        if pixel_x < self.points[0][0] or pixel_x > self.points[-1][0]:
            return None
        for index in range(1, len(self.points)):
            left_px, left_mm = self.points[index - 1]
            right_px, right_mm = self.points[index]
            if pixel_x <= right_px:
                fraction = (pixel_x - left_px) / (right_px - left_px)
                return left_mm + fraction * (right_mm - left_mm)
        return self.points[-1][1]


class MeasurementTracker:
    """Applies deterministic validation, quality and lost/reacquire rules."""

    def __init__(
        self,
        calibration,
        roi,
        min_confidence=0.50,
        min_box_px=4,
        max_box_px=240,
        max_speed_mm_s=300,
        temporal_margin_mm=8,
        lost_after_frames=3,
        lost_after_ms=100,
        reacquire_frames=3,
    ):
        self.calibration = calibration
        self.roi = roi
        self.min_confidence = float(min_confidence)
        self.min_box_px = float(min_box_px)
        self.max_box_px = float(max_box_px)
        self.max_speed_mm_s = float(max_speed_mm_s)
        self.temporal_margin_mm = float(temporal_margin_mm)
        self.lost_after_frames = int(lost_after_frames)
        self.lost_after_ms = int(lost_after_ms)
        self.reacquire_frames = int(reacquire_frames)
        self.seq = 0
        self.invalid_frames = 0
        self.last_valid_x_mm = None
        self.last_valid_ms = None
        self.reacquire_count = 0
        self.is_lost = True

    def _next_seq(self):
        value = self.seq
        self.seq = (self.seq + 1) & 0xFFFFFFFF
        return value

    def _in_roi(self, pixel_x, pixel_y):
        x, y, width, height = self.roi
        return x <= pixel_x <= x + width and y <= pixel_y <= y + height

    def _quality(self, candidate, temporal_ratio):
        confidence = float(candidate["confidence"])
        confidence_score = 100.0 * _clamp(
            (confidence - self.min_confidence) / (1.0 - self.min_confidence), 0.0, 1.0
        )
        x, y, width, height = self.roi
        edge_distance = min(
            candidate["pixel_x"] - x,
            x + width - candidate["pixel_x"],
            candidate["pixel_y"] - y,
            y + height - candidate["pixel_y"],
        )
        margin_score = 100.0 * _clamp(edge_distance / max(1.0, min(width, height) * 0.10), 0.0, 1.0)
        # Size is a plausibility gate, not a reward for a large detector box.
        # A correctly detected 10 px ball must not receive a near-zero quality.
        size_score = 100.0
        temporal_score = 100.0 * _clamp(temporal_ratio, 0.0, 1.0)
        return min(confidence_score, margin_score, size_score, temporal_score)

    def _invalid(self, capture_ms, reason):
        self.invalid_frames += 1
        elapsed = 0 if self.last_valid_ms is None else max(0, int(capture_ms) - self.last_valid_ms)
        if self.invalid_frames >= self.lost_after_frames or elapsed >= self.lost_after_ms:
            self.is_lost = True
            self.reacquire_count = 0
            reason = VisionStatus.LOST
        return VisionSampleV1(self._next_seq(), capture_ms, None, 0, reason)

    def process(self, capture_ms, candidate):
        """Process the best detector candidate, or None if there is no target."""
        try:
            if candidate is None:
                return self._invalid(capture_ms, VisionStatus.NO_TARGET)
            pixel_x = float(candidate["pixel_x"])
            pixel_y = float(candidate["pixel_y"])
            confidence = float(candidate["confidence"])
            box_w = float(candidate["box_w"])
            box_h = float(candidate["box_h"])
        except (KeyError, TypeError, ValueError):
            return self._invalid(capture_ms, VisionStatus.FAULT)

        if confidence < self.min_confidence:
            return self._invalid(capture_ms, VisionStatus.LOW_CONFIDENCE)
        if not self._in_roi(pixel_x, pixel_y):
            return self._invalid(capture_ms, VisionStatus.OUT_OF_RANGE)
        if box_w < self.min_box_px or box_h < self.min_box_px or box_w > self.max_box_px or box_h > self.max_box_px:
            return self._invalid(capture_ms, VisionStatus.LOW_CONFIDENCE)

        x_mm = self.calibration.map_pixel(pixel_x)
        if x_mm is None:
            return self._invalid(capture_ms, VisionStatus.OUT_OF_RANGE)

        temporal_ratio = 1.0
        # A LOST result deliberately invalidates the old measurement.  Do not
        # keep using that stale location to reject a genuinely reappearing ball
        # at another point on the rod; the three-frame reacquire gate below is
        # the safety mechanism in this mode.
        if (not self.is_lost and self.last_valid_x_mm is not None
                and self.last_valid_ms is not None):
            dt_ms = max(1, int(capture_ms) - self.last_valid_ms)
            gate_mm = self.max_speed_mm_s * dt_ms / 1000.0 + self.temporal_margin_mm
            delta_mm = abs(x_mm - self.last_valid_x_mm)
            if delta_mm > gate_mm:
                return self._invalid(capture_ms, VisionStatus.TEMPORAL_REJECT)
            temporal_ratio = 1.0 - delta_mm / max(gate_mm, 0.001)

        self.invalid_frames = 0
        self.last_valid_x_mm = x_mm
        self.last_valid_ms = int(capture_ms)
        if self.is_lost:
            self.reacquire_count += 1
            if self.reacquire_count < self.reacquire_frames:
                return VisionSampleV1(
                    self._next_seq(), capture_ms, None, 0, VisionStatus.TEMPORAL_REJECT
                )
            self.is_lost = False
        quality = self._quality(candidate, temporal_ratio)
        return VisionSampleV1(self._next_seq(), capture_ms, x_mm, quality, VisionStatus.VALID)
