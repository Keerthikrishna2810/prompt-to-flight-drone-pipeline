"""
test_vision_validator.py

Tests for vision_validator.py -- VisionFollowPlan -> validated search
MissionPlan via the unchanged core safety pipeline, plus vision-specific
bounds checks.

Run with:  pytest test_vision_validator.py -v
"""

from validator import SafetyConfig
from vision_validator import validate_vision_mission_json

CFG = SafetyConfig(
    home_lat=10.5270, home_lon=76.2140,
    max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
    max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
)

VALID_WAYPOINTS = [
    {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
    {"lat": 10.5272, "lon": 76.2145, "alt_m": 10},
    {"lat": 10.5274, "lon": 76.2148, "alt_m": 10},
]


def _valid_mission(**overrides):
    base = {
        "target_class": "person",
        "search_waypoints": VALID_WAYPOINTS,
        "search_speed_mps": 4,
        "follow_distance_m": 8,
        "follow_altitude_m": 10,
        "max_follow_duration_s": 60,
    }
    base.update(overrides)
    return base


def test_valid_mission_is_accepted():
    result = validate_vision_mission_json(_valid_mission(), CFG)
    assert result.ok, result.errors
    assert result.plan.target_class == "person"
    assert len(result.search_plan.waypoints) == 3


def test_empty_target_class_is_rejected():
    result = validate_vision_mission_json(_valid_mission(target_class=""), CFG)
    assert not result.ok


def test_search_route_outside_geofence_is_rejected():
    far_waypoints = [
        {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
        {"lat": 10.60, "lon": 76.30, "alt_m": 10},  # far outside 200m geofence
    ]
    result = validate_vision_mission_json(_valid_mission(search_waypoints=far_waypoints), CFG)
    assert not result.ok
    assert any("search route" in e for e in result.errors)


def test_follow_altitude_outside_configured_bounds_is_rejected():
    tight_cfg = SafetyConfig(
        home_lat=CFG.home_lat, home_lon=CFG.home_lon, max_geofence_radius_m=CFG.max_geofence_radius_m,
        min_alt_m=5.0, max_alt_m=15.0, max_speed_mps=CFG.max_speed_mps,
        max_leg_distance_m=CFG.max_leg_distance_m, loop_closure_tolerance_m=CFG.loop_closure_tolerance_m,
    )
    result = validate_vision_mission_json(_valid_mission(follow_altitude_m=20.0), tight_cfg)
    assert not result.ok
    assert any("follow_altitude_m" in e for e in result.errors)


def test_search_speed_over_limit_is_rejected():
    result = validate_vision_mission_json(_valid_mission(search_speed_mps=50), CFG)
    assert not result.ok


if __name__ == "__main__":
    tests = [
        test_valid_mission_is_accepted,
        test_empty_target_class_is_rejected,
        test_search_route_outside_geofence_is_rejected,
        test_follow_altitude_outside_configured_bounds_is_rejected,
        test_search_speed_over_limit_is_rejected,
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
