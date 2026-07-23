"""
test_vision_interpreter.py

Tests for vision_interpreter.py's retry/validation logic, Ollama mocked
out entirely -- same philosophy as test_squad_interpreter.py.

Run with:  pytest test_vision_interpreter.py -v
"""

import json
from unittest.mock import patch

from vision_interpreter import interpret_vision_prompt
from validator import SafetyConfig

CFG = SafetyConfig(
    home_lat=10.5270, home_lon=76.2140,
    max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
    max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
)

VALID_VISION_MISSION = json.dumps({
    "target_class": "person",
    "search_waypoints": [
        {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
        {"lat": 10.5272, "lon": 76.2145, "alt_m": 10},
    ],
    "search_speed_mps": 4,
    "follow_distance_m": 8,
    "follow_altitude_m": 10,
    "max_follow_duration_s": 60,
    "detection_confidence_threshold": 0.5,
})

# empty target_class -- schema-legal-looking JSON shape, fails vision schema validation
EMPTY_TARGET_CLASS = json.dumps({
    "target_class": "",
    "search_waypoints": [
        {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
        {"lat": 10.5272, "lon": 76.2145, "alt_m": 10},
    ],
    "search_speed_mps": 4,
    "follow_distance_m": 8,
    "follow_altitude_m": 10,
    "max_follow_duration_s": 60,
    "detection_confidence_threshold": 0.5,
})


def test_accepts_on_first_valid_draft():
    with patch("vision_interpreter._call_ollama", return_value=VALID_VISION_MISSION) as mock_call:
        result = interpret_vision_prompt("follow the person walking down this path", CFG, max_attempts=2)
    assert result.ok
    assert result.attempts == 1
    assert result.plan.target_class == "person"
    assert mock_call.call_count == 1


def test_retries_once_then_succeeds():
    with patch("vision_interpreter._call_ollama",
               side_effect=[EMPTY_TARGET_CLASS, VALID_VISION_MISSION]) as mock_call:
        result = interpret_vision_prompt("go find something and follow it", CFG, max_attempts=2)
    assert result.ok
    assert result.attempts == 2
    assert mock_call.call_count == 2


def test_gives_up_after_max_attempts():
    with patch("vision_interpreter._call_ollama", return_value=EMPTY_TARGET_CLASS) as mock_call:
        result = interpret_vision_prompt("find nothing in particular", CFG, max_attempts=2)
    assert not result.ok
    assert result.attempts == 2
    assert result.plan is None
    assert mock_call.call_count == 2


def test_malformed_json_triggers_retry():
    with patch("vision_interpreter._call_ollama", side_effect=["not json", VALID_VISION_MISSION]):
        result = interpret_vision_prompt("follow the dog", CFG, max_attempts=2)
    assert result.ok
    assert result.attempts == 2


def test_target_class_is_configurable_from_the_prompt():
    """The core Challenge 3 requirement -- the target type comes from
    whatever the operator said, not a hardcoded value."""
    car_mission = json.dumps({**json.loads(VALID_VISION_MISSION), "target_class": "car"})
    with patch("vision_interpreter._call_ollama", return_value=car_mission):
        result = interpret_vision_prompt("follow that red car", CFG, max_attempts=1)
    assert result.ok
    assert result.plan.target_class == "car"


if __name__ == "__main__":
    tests = [
        test_accepts_on_first_valid_draft,
        test_retries_once_then_succeeds,
        test_gives_up_after_max_attempts,
        test_malformed_json_triggers_retry,
        test_target_class_is_configurable_from_the_prompt,
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
