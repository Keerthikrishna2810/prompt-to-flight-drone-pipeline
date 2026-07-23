"""
test_vision_fixtures.py

Proves the example vision fixture in fixtures/vision/ actually validates
the way its _note says. Lives in its own subfolder (not fixtures/*.json
directly) so test_validator.py's generic single-drone fixture glob
doesn't pick it up and fail on its different schema shape -- same reason
squad fixtures use different field names that happen to coincidentally
overlap with MissionPlan's; vision's don't, so it needs the subfolder
separation to avoid a false failure there.

Run with:  pytest test_vision_fixtures.py -v
"""

import json
import pathlib

from validator import SafetyConfig
from vision_validator import validate_vision_mission_json

FIXTURES_DIR = pathlib.Path(__file__).parent / "fixtures" / "vision"

CFG = SafetyConfig(
    home_lat=10.5270, home_lon=76.2140,
    max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
    max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
)


def _load(name: str) -> dict:
    raw = json.loads((FIXTURES_DIR / name).read_text())
    raw.pop("_expect", None)
    raw.pop("_note", None)
    return raw


def test_valid_vision_fixture_is_accepted():
    raw = _load("valid_vision_follow_person.json")
    result = validate_vision_mission_json(raw, CFG)
    assert result.ok, result.errors
    assert result.plan.target_class == "person"
    assert len(result.search_plan.waypoints) == len(raw["search_waypoints"])


if __name__ == "__main__":
    try:
        test_valid_vision_fixture_is_accepted()
        print("[PASS] test_valid_vision_fixture_is_accepted")
    except AssertionError as e:
        print(f"[FAIL] test_valid_vision_fixture_is_accepted: {e}")
