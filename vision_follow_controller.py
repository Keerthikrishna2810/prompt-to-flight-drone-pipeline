"""
vision_follow_controller.py

Challenge 3 -- pure geometry, same role formation.py plays for Challenge
1: this file has never heard of a camera, a drone, or MAVSDK. It only
ever converts (a Detection's normalized bounding box, a desired standoff
distance) into (how far forward/right to move). That boundary is what
makes it testable with plain asserts (see test_vision_follow_controller.py)
independent of any detector or simulator.

Honesty note on the approach: a single camera frame has no true depth
information. Bounding box WIDTH is used as the only available proxy for
distance -- a target that looks bigger is assumed closer. This is a
coarse, standard technique (used in real vision-servoing systems as a
starting point) but it assumes the target's real-world size roughly
matches `reference_bbox_width_at_follow_distance`'s calibration; a target
much larger or smaller than expected will be over/under-approached. A
depth camera or stereo pair would remove this assumption entirely --
noted as follow-up work rather than solved here, to keep scope honest.
"""

import math
from dataclasses import dataclass

from vision_detector import Detection


@dataclass(frozen=True)
class FollowCommand:
    forward_m: float          # positive = move toward the target, negative = back away
    right_m: float             # positive = move right to re-center the target in frame
    reached_standoff: bool     # True once close enough to distance AND centered enough to hold position


def compute_follow_offset(
    detection: Detection,
    follow_distance_m: float,
    camera_hfov_deg: float = 80.0,
    reference_bbox_width_at_follow_distance: float = 0.25,
    size_tolerance: float = 0.15,
    center_tolerance: float = 0.08,
) -> FollowCommand:
    """One control-loop tick's worth of steering, given the latest
    detection and how far away we want to stay.

    Horizontal steering: how far the target's box center sits from frame
    center (0.5) is converted to an angle using the camera's horizontal
    field of view, then to a lateral offset at the target's assumed
    range -- move right if the target is right of center, and vice versa.

    Distance steering: reference_bbox_width_at_follow_distance is "how
    wide, as a fraction of frame width, does this target look when it's
    exactly follow_distance_m away" -- a calibration constant. If the
    box is wider than that, the target looks closer than desired, so
    forward_m comes out negative (back away); narrower means forward_m
    is positive (approach).
    """
    if not (0.0 <= detection.bbox_x_center <= 1.0):
        raise ValueError(f"bbox_x_center must be normalized to [0, 1], got {detection.bbox_x_center}")
    if detection.bbox_width <= 0:
        raise ValueError(f"bbox_width must be positive, got {detection.bbox_width}")

    x_offset = detection.bbox_x_center - 0.5
    angle_offset_deg = x_offset * camera_hfov_deg
    right_m = follow_distance_m * math.tan(math.radians(angle_offset_deg))

    size_ratio = detection.bbox_width / reference_bbox_width_at_follow_distance
    forward_m = follow_distance_m * (1.0 - size_ratio)

    reached = abs(size_ratio - 1.0) < size_tolerance and abs(x_offset) < center_tolerance

    return FollowCommand(forward_m=forward_m, right_m=right_m, reached_standoff=reached)
