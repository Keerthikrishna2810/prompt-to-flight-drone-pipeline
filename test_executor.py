"""
test_executor.py

Day 3 tests for mission_executor.py that need no live PX4/Gazebo session --
everything here runs via `dry_run=True`, which produces the exact same
audit log a real run would (same lat/lon/alt math, same command sequence)
without touching a vehicle. That means these tests run in CI / inside the
Docker build itself, same as Day 2's test_validator.py.

Run with:  pytest test_executor.py -v
Or directly:  python3 test_executor.py
"""

import asyncio
import json
import pathlib

import pytest

from mission_executor import MissionExecutor, _offset_from_reference, _apply_offset
from schema import MissionPlan
from validator import SafetyConfig, validate_mission_json

FIXTURES_DIR = pathlib.Path(__file__).parent / "fixtures"

# Same reference position as test_validator.py's SAFETY_CFG, so the
# existing Day 2 fixtures validate identically here.
SAFETY_CFG = SafetyConfig(
    home_lat=10.5270,
    home_lon=76.2140,
    max_geofence_radius_m=200.0,
    min_alt_m=3.0,
    max_alt_m=25.0,
    max_speed_mps=8.0,
    max_leg_distance_m=150.0,
    loop_closure_tolerance_m=15.0,
)


def _load_valid_plan(name: str) -> MissionPlan:
    raw = json.loads((FIXTURES_DIR / name).read_text())
    raw.pop("_expect", None)
    raw.pop("_note", None)
    result = validate_mission_json(raw, SAFETY_CFG)
    assert result.ok, f"{name} unexpectedly failed validation: {result.errors}"
    return MissionPlan.model_validate(raw)


def _dry_run_executor() -> MissionExecutor:
    return MissionExecutor(
        drone=None,
        reference_lat=SAFETY_CFG.home_lat,
        reference_lon=SAFETY_CFG.home_lon,
        dry_run=True,
    )


# -- coordinate re-homing math ----------------------------------------------

def test_offset_roundtrip():
    """apply_offset(offset_from_reference(p)) should return p, within
    floating-point tolerance. This is the math the executor relies on to
    re-anchor a mission onto wherever the sim actually spawns."""
    lat, lon = 10.5300, 76.2200
    north_m, east_m = _offset_from_reference(SAFETY_CFG.home_lat, SAFETY_CFG.home_lon, lat, lon)
    lat2, lon2 = _apply_offset(SAFETY_CFG.home_lat, SAFETY_CFG.home_lon, north_m, east_m)
    assert lat2 == pytest.approx(lat, abs=1e-7)
    assert lon2 == pytest.approx(lon, abs=1e-7)


def test_rehoming_preserves_pattern_shape_on_different_world():
    """The whole point of re-homing: flying the same plan from a totally
    different real-world home position should produce waypoints with the
    identical local offset pattern, just anchored elsewhere -- proving
    the same mission JSON is portable across simulator worlds."""
    plan = _load_valid_plan("valid_patrol_loop.json")

    executor_a = _dry_run_executor()  # dry-run "sim home" == authored reference (Kerala)
    log_a = asyncio.run(executor_a.execute(plan))

    # Simulate a completely different spawn point (e.g. PX4's default
    # Zurich world) by pointing this executor's dry-run stand-in home
    # position at Zurich, while the plan's authored reference frame
    # (Kerala) stays fixed -- this is exactly what happens on a real run.
    executor_b = _dry_run_executor()
    executor_b.reference_lat, executor_b.reference_lon = 47.3977508, 8.5456074  # Zurich
    log_b = asyncio.run(executor_b.execute(plan))

    gotos_a = [e for e in log_a if e.action == "goto_location"]
    gotos_b = [e for e in log_b if e.action == "goto_location"]

    assert len(gotos_a) == len(gotos_b) == len(plan.waypoints) * plan.repetitions
    # Leg-to-leg deltas (in degrees) should match between the two runs even
    # though absolute positions differ -- same shape, different anchor.
    for i in range(1, len(gotos_a)):
        d_lat_a = gotos_a[i].params["lat"] - gotos_a[i - 1].params["lat"]
        d_lat_b = gotos_b[i].params["lat"] - gotos_b[i - 1].params["lat"]
        assert d_lat_a == pytest.approx(d_lat_b, abs=1e-5)


# -- command sequence shape ---------------------------------------------

def test_dry_run_command_sequence_route():
    plan = _load_valid_plan("valid_route_single_pass.json")
    log = asyncio.run(_dry_run_executor().execute(plan))

    actions = [e.action for e in log]
    assert "arm" in actions
    assert "set_takeoff_altitude" in actions
    assert "takeoff" in actions
    assert actions.count("goto_location") == len(plan.waypoints) * plan.repetitions
    assert actions[-1] == "return_to_launch"
    # arm must happen before any goto_location
    assert actions.index("arm") < actions.index("goto_location")


def test_dry_run_command_sequence_loop_repeats():
    plan = _load_valid_plan("valid_patrol_loop.json")
    log = asyncio.run(_dry_run_executor().execute(plan))

    goto_count = sum(1 for e in log if e.action == "goto_location")
    assert goto_count == len(plan.waypoints) * plan.repetitions


def test_only_loop_missions_repeat():
    route_plan = _load_valid_plan("valid_route_single_pass.json")
    log = asyncio.run(_dry_run_executor().execute(route_plan))
    goto_count = sum(1 for e in log if e.action == "goto_location")
    assert goto_count == len(route_plan.waypoints)  # exactly one pass, regardless of `repetitions`


# -- determinism, the actual Day 3 exit criterion ------------------------

def test_determinism_same_plan_same_sequence():
    """Same MissionPlan executed twice must produce identical command
    sequences (action + params). This is the concrete, checkable version
    of the 'deterministic executor' claim in the architecture write-up."""
    plan = _load_valid_plan("valid_patrol_loop.json")

    log_a = asyncio.run(_dry_run_executor().execute(plan))
    log_b = asyncio.run(_dry_run_executor().execute(plan))

    seq_a = [(e.action, e.params) for e in log_a]
    seq_b = [(e.action, e.params) for e in log_b]
    assert seq_a == seq_b


if __name__ == "__main__":
    tests = [
        test_offset_roundtrip,
        test_rehoming_preserves_pattern_shape_on_different_world,
        test_dry_run_command_sequence_route,
        test_dry_run_command_sequence_loop_repeats,
        test_only_loop_missions_repeat,
        test_determinism_same_plan_same_sequence,
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
