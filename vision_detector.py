"""
vision_detector.py

Challenge 3 -- the detector is a pluggable interface (TargetDetector),
not a hardcoded dependency. Same "optional, gracefully degrading"
pattern as formation_viz.py's rclpy import: everything else in this
project (vision_follow_controller.py, vision_executor.py) is written
against the Detection/TargetDetector shapes below and fully testable
with MockDetector, with zero dependency on any actual ML library. If
ultralytics isn't installed, YoloDetector simply can't be constructed --
nothing else breaks.

This split matters for the same reason schema.py is separate from
validator.py: swapping the detector (YOLO for a different model, a
cloud API, a different confidence calibration) should never require
touching the flight logic that consumes its output.
"""

from dataclasses import dataclass
from typing import List, Protocol


@dataclass(frozen=True)
class Detection:
    """A single detected object in one camera frame. Bounding box fields
    are normalized to [0, 1] -- fractions of frame width/height -- so
    downstream code (vision_follow_controller.py) never needs to know
    the actual camera resolution."""
    class_name: str
    confidence: float
    bbox_x_center: float  # 0 = left edge, 1 = right edge
    bbox_y_center: float  # 0 = top edge, 1 = bottom edge
    bbox_width: float     # fraction of frame width -- bigger == closer, the only depth proxy available
    bbox_height: float


class TargetDetector(Protocol):
    def detect(self, frame) -> List[Detection]: ...


class MockDetector:
    """Deterministic, scripted detector for tests -- returns a fixed
    sequence of detection lists regardless of the actual frame content
    (frame can be anything, even None, in tests using this). Each call
    to detect() advances to the next scripted entry; the last entry
    repeats once the script runs out, so a test doesn't need to know
    exactly how many polls will happen."""

    def __init__(self, scripted_detections: List[List[Detection]]):
        if not scripted_detections:
            raise ValueError("MockDetector needs at least one scripted detection list")
        self._script = scripted_detections
        self.call_count = 0

    def detect(self, frame) -> List[Detection]:
        index = min(self.call_count, len(self._script) - 1)
        self.call_count += 1
        return self._script[index]


class YoloDetector:
    """Real detector, backed by ultralytics YOLO. Lazy-imports so that
    every other file in this project works and is testable without
    ultralytics/torch installed at all -- only constructing a
    YoloDetector requires them.

    NOT verified live in this environment -- no GPU/camera stream was
    available while building this (same honesty note as fly_squad.sh's
    Gazebo spawn). The interface and the math in
    vision_follow_controller.py that consumes its output ARE fully
    tested, via MockDetector.
    """

    def __init__(self, model_name: str = "yolov8n.pt"):
        try:
            from ultralytics import YOLO
        except ImportError as e:
            raise RuntimeError(
                "ultralytics is not installed -- pip install ultralytics to use YoloDetector. "
                "Everything else in the vision pipeline works without it (see MockDetector)."
            ) from e
        self._model = YOLO(model_name)

    def detect(self, frame) -> List[Detection]:
        h, w = frame.shape[0], frame.shape[1]
        results = self._model(frame, verbose=False)[0]
        detections: List[Detection] = []
        for box in results.boxes:
            class_name = self._model.names[int(box.cls[0])]
            confidence = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            detections.append(Detection(
                class_name=class_name,
                confidence=confidence,
                bbox_x_center=(x1 + x2) / 2.0 / w,
                bbox_y_center=(y1 + y2) / 2.0 / h,
                bbox_width=(x2 - x1) / w,
                bbox_height=(y2 - y1) / h,
            ))
        return detections
