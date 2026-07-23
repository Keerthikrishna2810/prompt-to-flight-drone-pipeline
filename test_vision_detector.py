"""
test_vision_detector.py

Tests for vision_detector.py's MockDetector -- proves the scripted-
sequence contract vision_executor.py's tests rely on. YoloDetector is
NOT tested here (it needs ultralytics installed, which is intentionally
optional -- see vision_detector.py's docstring).

Run with:  pytest test_vision_detector.py -v
"""

import pytest

from vision_detector import Detection, MockDetector


def _det(class_name="person"):
    return Detection(class_name=class_name, confidence=0.9,
                      bbox_x_center=0.5, bbox_y_center=0.5, bbox_width=0.2, bbox_height=0.4)


def test_returns_scripted_detections_in_order():
    mock = MockDetector([[], [_det("person")], []])
    assert mock.detect(None) == []
    assert mock.detect(None) == [_det("person")]
    assert mock.detect(None) == []


def test_repeats_last_entry_once_script_exhausted():
    last = [_det("car")]
    mock = MockDetector([[], last])
    mock.detect(None)
    mock.detect(None)
    assert mock.detect(None) == last
    assert mock.detect(None) == last


def test_call_count_tracks_number_of_calls():
    mock = MockDetector([[]])
    mock.detect(None)
    mock.detect(None)
    mock.detect(None)
    assert mock.call_count == 3


def test_rejects_empty_script():
    with pytest.raises(ValueError):
        MockDetector([])


if __name__ == "__main__":
    tests = [
        test_returns_scripted_detections_in_order,
        test_repeats_last_entry_once_script_exhausted,
        test_call_count_tracks_number_of_calls,
        test_rejects_empty_script,
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
