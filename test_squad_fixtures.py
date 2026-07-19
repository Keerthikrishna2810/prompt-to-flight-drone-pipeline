"""
test_squad_fixtures.py

Proves the example squad fixtures in fixtures/ actually validate the way
their _note says, same role as test_validator.py plays for the core
task's single-drone fixtures.

Run with:  pytest test_squad_fixtures.py -v
"""

import json
import pathlib

from validator import SafetyConfig
from squad_validator import validate_squad_mission_json

FIXTURES_DIR = pathlib.Path(__file__).parent / "fixtures"

CFG = SafetyConfig(
    home_lat=10.5270, home_lon=76.2140,
    max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
    max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
)

VALID_SQUAD_FIXTURES = [
    "valid_wedge_formation.json",
    "valid_line_abreast_loop.json",
    "valid_split_sweep.json",
]


def _load(name: str) -> dict:
    raw = json.loads((FIXTURES_DIR / name).read_text())
    raw.pop("_expect", None)
    raw.pop("_note", None)
    return raw


def test_all_valid_squad_fixtures_are_accepted():
    for name in VALID_SQUAD_FIXTURES:
        raw = _load(name)
        result = validate_squad_mission_json(raw, CFG)
        assert result.ok, f"{name} unexpectedly rejected: {result.errors}"
        assert len(result.plans) == raw["drone_count"]


def test_wedge_fixture_leader_flies_unmodified_route():
    raw = _load("valid_wedge_formation.json")
    result = validate_squad_mission_json(raw, CFG)
    leader_plan = result.plans[0]
    for flown, original in zip(leader_plan.waypoints, raw["waypoints"]):
        assert flown.lat == original["lat"]
        assert flown.lon == original["lon"]


def test_split_fixture_produces_contiguous_lanes():
    raw = _load("valid_split_sweep.json")
    result = validate_squad_mission_json(raw, CFG)
    plans = result.plans
    for i in range(len(plans) - 1):
        assert plans[i].waypoints[-1].lat == plans[i + 1].waypoints[0].lat
        assert plans[i].waypoints[-1].lon == plans[i + 1].waypoints[0].lon


if __name__ == "__main__":
    tests = [
        test_all_valid_squad_fixtures_are_accepted,
        test_wedge_fixture_leader_flies_unmodified_route,
        test_split_fixture_produces_contiguous_lanes,
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
