"""
test_llm_interpreter.py

Day 4 tests for llm_interpreter.py's retry/validation logic, with Ollama
itself mocked out -- these run with no live model and no network call,
same philosophy as Day 2/3's tests: prove the logic before trusting it
against something genuinely unpredictable.

Run with:  pytest test_llm_interpreter.py -v
Or directly:  python3 test_llm_interpreter.py
"""

import json
from unittest.mock import patch

from llm_interpreter import interpret_prompt
from validator import SafetyConfig

CFG = SafetyConfig(
    home_lat=10.5270, home_lon=76.2140,
    max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
    max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
)

VALID_ROUTE = json.dumps({
    "mission_type": "route", "repetitions": 1, "speed_mps": 5,
    "waypoints": [
        {"lat": 10.5270, "lon": 76.2140, "alt_m": 8},
        {"lat": 10.5272, "lon": 76.2142, "alt_m": 10},
    ],
})

# speed_mps=12 is schema-legal (ceiling 15) but exceeds the safety cap of 8
# -- exercises Stage 2 rejection, not just Stage 1.
INVALID_SPEED = json.dumps({
    "mission_type": "route", "repetitions": 1, "speed_mps": 12,
    "waypoints": [
        {"lat": 10.5270, "lon": 76.2140, "alt_m": 8},
        {"lat": 10.5272, "lon": 76.2142, "alt_m": 10},
    ],
})


def test_accepts_on_first_valid_draft():
    with patch("llm_interpreter._call_ollama", return_value=VALID_ROUTE) as mock_call:
        result = interpret_prompt("fly a short route", CFG, max_attempts=2)
    assert result.ok
    assert result.attempts == 1
    assert mock_call.call_count == 1


def test_retries_once_then_succeeds():
    with patch("llm_interpreter._call_ollama", side_effect=[INVALID_SPEED, VALID_ROUTE]) as mock_call:
        result = interpret_prompt("fly fast around the block", CFG, max_attempts=2)
    assert result.ok
    assert result.attempts == 2
    assert mock_call.call_count == 2


def test_gives_up_after_max_attempts():
    with patch("llm_interpreter._call_ollama", return_value=INVALID_SPEED) as mock_call:
        result = interpret_prompt("fly very fast", CFG, max_attempts=2)
    assert not result.ok
    assert result.attempts == 2
    assert len(result.errors) > 0
    assert mock_call.call_count == 2  # bounded -- must not retry forever


def test_malformed_json_triggers_retry():
    with patch("llm_interpreter._call_ollama", side_effect=["not json at all", VALID_ROUTE]):
        result = interpret_prompt("fly somewhere", CFG, max_attempts=2)
    assert result.ok
    assert result.attempts == 2


def test_retry_prompt_includes_validation_errors():
    """The retry message sent back to the model must actually contain the
    validation errors -- otherwise 'bounded retry with feedback' is just
    a relabeled blind retry."""
    with patch("llm_interpreter._call_ollama", side_effect=[INVALID_SPEED, VALID_ROUTE]) as mock_call:
        interpret_prompt("fly fast", CFG, max_attempts=2)

    second_call_messages = mock_call.call_args_list[1].args[0]
    retry_feedback = second_call_messages[-1]["content"]
    assert "exceeds configured max" in retry_feedback


if __name__ == "__main__":
    tests = [
        test_accepts_on_first_valid_draft,
        test_retries_once_then_succeeds,
        test_gives_up_after_max_attempts,
        test_malformed_json_triggers_retry,
        test_retry_prompt_includes_validation_errors,
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
