"""
test_squad_interpreter.py

Tests for squad_interpreter.py's retry/validation logic, Ollama mocked
out entirely -- same philosophy as test_llm_interpreter.py: prove the
retry-with-feedback logic before trusting it against a real model.

Run with:  pytest test_squad_interpreter.py -v
"""

import json
from unittest.mock import patch

from squad_interpreter import interpret_squad_prompt
from validator import SafetyConfig

CFG = SafetyConfig(
    home_lat=10.5270, home_lon=76.2140,
    max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
    max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
)

VALID_FORMATION_SQUAD = json.dumps({
    "drone_count": 3, "mode": "formation", "formation": "wedge", "spacing_m": 10.0,
    "mission_type": "route", "repetitions": 1, "speed_mps": 5,
    "waypoints": [
        {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
        {"lat": 10.5272, "lon": 76.2145, "alt_m": 10},
    ],
})

# drone_count=8 is schema-legal-looking but exceeds SquadPlan's le=6 cap --
# exercises squad-schema rejection, not per-drone safety rejection.
TOO_MANY_DRONES = json.dumps({
    "drone_count": 8, "mode": "formation", "formation": "line", "spacing_m": 10.0,
    "mission_type": "route", "repetitions": 1, "speed_mps": 5,
    "waypoints": [
        {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
        {"lat": 10.5272, "lon": 76.2145, "alt_m": 10},
    ],
})


def test_accepts_on_first_valid_draft():
    with patch("squad_interpreter._call_ollama", return_value=VALID_FORMATION_SQUAD) as mock_call:
        result = interpret_squad_prompt("send three drones in a wedge down this route", CFG, max_attempts=2)
    assert result.ok
    assert result.attempts == 1
    assert len(result.plans) == 3
    assert mock_call.call_count == 1


def test_retries_once_then_succeeds():
    with patch("squad_interpreter._call_ollama", side_effect=[TOO_MANY_DRONES, VALID_FORMATION_SQUAD]) as mock_call:
        result = interpret_squad_prompt("send a big squad down this route", CFG, max_attempts=2)
    assert result.ok
    assert result.attempts == 2
    assert mock_call.call_count == 2


def test_gives_up_after_max_attempts():
    with patch("squad_interpreter._call_ollama", return_value=TOO_MANY_DRONES) as mock_call:
        result = interpret_squad_prompt("send eight drones", CFG, max_attempts=2)
    assert not result.ok
    assert result.attempts == 2
    assert result.plans is None
    assert mock_call.call_count == 2  # bounded, must not retry forever


def test_malformed_json_triggers_retry():
    with patch("squad_interpreter._call_ollama", side_effect=["not json at all", VALID_FORMATION_SQUAD]):
        result = interpret_squad_prompt("send the squad somewhere", CFG, max_attempts=2)
    assert result.ok
    assert result.attempts == 2


def test_retry_prompt_includes_validation_errors():
    with patch("squad_interpreter._call_ollama", side_effect=[TOO_MANY_DRONES, VALID_FORMATION_SQUAD]) as mock_call:
        interpret_squad_prompt("send a big squad", CFG, max_attempts=2)

    second_call_messages = mock_call.call_args_list[1].args[0]
    retry_feedback = second_call_messages[-1]["content"]
    assert "squad schema validation failed" in retry_feedback


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
