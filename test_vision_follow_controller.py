"""
test_vision_follow_controller.py

Tests for vision_follow_controller.py's pure geometry. No camera, no
drone, no detector -- just Detection objects built by hand.

Run with:  pytest test_vision_follow_controller.py -v
"""

import pytest

from vision_detector import Detection
from vision_follow_controller import compute_follow_offset


def _detection(x_center=0.5, width=0.25, y_center=0.5, height=0.4, confidence=0.9, class_name="person"):
    return Detection(class_name=class_name, confidence=confidence,
                      bbox_x_center=x_center, bbox_y_center=y_center,
                      bbox_width=width, bbox_height=height)


def test_centered_target_at_reference_size_has_zero_offset():
    det = _detection(x_center=0.5, width=0.25)
    cmd = compute_follow_offset(det, follow_distance_m=10.0, reference_bbox_width_at_follow_distance=0.25)
    assert cmd.forward_m == pytest.approx(0.0, abs=1e-6)
    assert cmd.right_m == pytest.approx(0.0, abs=1e-6)
    assert cmd.reached_standoff


def test_target_left_of_center_produces_negative_right_offset():
    det = _detection(x_center=0.3, width=0.25)
    cmd = compute_follow_offset(det, follow_distance_m=10.0, reference_bbox_width_at_follow_distance=0.25)
    assert cmd.right_m < 0


def test_target_right_of_center_produces_positive_right_offset():
    det = _detection(x_center=0.7, width=0.25)
    cmd = compute_follow_offset(det, follow_distance_m=10.0, reference_bbox_width_at_follow_distance=0.25)
    assert cmd.right_m > 0


def test_target_looking_bigger_than_reference_backs_away():
    """A bigger box than the reference means the target looks closer
    than the desired standoff -- the drone should back off (negative
    forward_m)."""
    det = _detection(width=0.5)  # twice the reference width
    cmd = compute_follow_offset(det, follow_distance_m=10.0, reference_bbox_width_at_follow_distance=0.25)
    assert cmd.forward_m < 0


def test_target_looking_smaller_than_reference_approaches():
    det = _detection(width=0.1)  # smaller than reference -- target is far
    cmd = compute_follow_offset(det, follow_distance_m=10.0, reference_bbox_width_at_follow_distance=0.25)
    assert cmd.forward_m > 0


def test_reached_standoff_false_when_off_center_even_at_right_distance():
    det = _detection(x_center=0.9, width=0.25)  # right size, but way off to the side
    cmd = compute_follow_offset(det, follow_distance_m=10.0, reference_bbox_width_at_follow_distance=0.25)
    assert not cmd.reached_standoff


def test_reached_standoff_false_when_centered_but_wrong_distance():
    det = _detection(x_center=0.5, width=0.6)  # centered, but far too close
    cmd = compute_follow_offset(det, follow_distance_m=10.0, reference_bbox_width_at_follow_distance=0.25)
    assert not cmd.reached_standoff


def test_rejects_out_of_range_bbox_x_center():
    det = _detection(x_center=1.5)
    with pytest.raises(ValueError):
        compute_follow_offset(det, follow_distance_m=10.0)


def test_rejects_non_positive_bbox_width():
    det = _detection(width=0.0)
    with pytest.raises(ValueError):
        compute_follow_offset(det, follow_distance_m=10.0)


if __name__ == "__main__":
    tests = [
        test_centered_target_at_reference_size_has_zero_offset,
        test_target_left_of_center_produces_negative_right_offset,
        test_target_right_of_center_produces_positive_right_offset,
        test_target_looking_bigger_than_reference_backs_away,
        test_target_looking_smaller_than_reference_approaches,
        test_reached_standoff_false_when_off_center_even_at_right_distance,
        test_reached_standoff_false_when_centered_but_wrong_distance,
        test_rejects_out_of_range_bbox_x_center,
        test_rejects_non_positive_bbox_width,
    ]
    passed, failed = 0, 0
    for t in tests:
        try:
            t()
            print(f"[PASS] {t.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"[FAIL] {t.__name__}: {e}")
            failed += 1
    print(f"\n{passed} passed, {failed} failed")
