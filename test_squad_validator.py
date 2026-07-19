"""
test_squad_validator.py

Tests for squad_validator.py -- SquadPlan -> N per-drone MissionPlans,
each validated through the unchanged core-task pipeline, plus the new
minimum-separation check. No PX4, no LLM.

Run with:  pytest test_squad_validator.py -v
"""

import pytest

from validator import SafetyConfig
from squad_validator import validate_squad_mission_json, check_min_separation
from schema import MissionPlan

CFG = SafetyConfig(
    home_lat=10.5270, home_lon=76.2140,
    max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
    max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
)

VALID_ROUTE_WAYPOINTS = [
    {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
    {"lat": 10.5272, "lon": 76.2145, "alt_m": 10},
]

VALID_LOOP_WAYPOINTS = [
    {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
    {"lat": 10.5273, "lon": 76.2140, "alt_m": 10},
    {"lat": 10.5273, "lon": 76.2144, "alt_m": 10},
    {"lat": 10.5270, "lon": 76.2144, "alt_m": 10},
    {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
]


def _formation_squad(drone_count=3, formation="wedge", spacing_m=10.0, waypoints=None, mission_type="route"):
    return {
        "drone_count": drone_count,
        "mode": "formation",
        "formation": formation,
        "spacing_m": spacing_m,
        "mission_type": mission_type,
        "waypoints": waypoints or VALID_ROUTE_WAYPOINTS,
        "repetitions": 1,
        "speed_mps": 5,
    }


def _split_squad(drone_count=3, waypoints=None):
    return {
        "drone_count": drone_count,
        "mode": "split",
        "mission_type": "route",
        "waypoints": waypoints or VALID_LOOP_WAYPOINTS,
        "repetitions": 1,
        "speed_mps": 5,
    }


# -- happy paths -----------------------------------------------------------

def test_valid_formation_squad_produces_one_plan_per_drone():
    result = validate_squad_mission_json(_formation_squad(drone_count=3), CFG)
    assert result.ok, result.errors
    assert len(result.plans) == 3
    for plan in result.plans:
        assert isinstance(plan, MissionPlan)


def test_valid_split_squad_produces_one_plan_per_drone():
    result = validate_squad_mission_json(_split_squad(drone_count=3), CFG)
    assert result.ok, result.errors
    assert len(result.plans) == 3


def test_formation_plans_each_pass_normal_per_drone_safety_rules():
    """Every drone's route must independently satisfy the exact same
    geofence/altitude/speed rules a single drone would -- nothing about
    being in a squad relaxes them."""
    result = validate_squad_mission_json(_formation_squad(drone_count=2, spacing_m=15.0), CFG)
    assert result.ok, result.errors
    for plan in result.plans:
        for wp in plan.waypoints:
            assert CFG.min_alt_m <= wp.alt_m <= CFG.max_alt_m


# -- squad schema rejection --------------------------------------------

def test_rejects_too_few_drones():
    result = validate_squad_mission_json(_formation_squad(drone_count=1), CFG)
    assert not result.ok
    assert any("squad schema" in e for e in result.errors)


def test_rejects_too_many_drones():
    result = validate_squad_mission_json(_formation_squad(drone_count=10), CFG)
    assert not result.ok


def test_rejects_unknown_formation():
    raw = _formation_squad()
    raw["formation"] = "diamond"  # not a real FormationType
    result = validate_squad_mission_json(raw, CFG)
    assert not result.ok


# -- per-drone rejection propagates with drone index -----------------------

def test_per_drone_geofence_violation_is_reported_with_drone_index():
    # spacing_m is schema-capped at 50 (a sane per-drone bound, same
    # "hard ceiling regardless of config" idea as schema.py's altitude
    # cap) so a wing drone's max possible reach here is ~77m from home --
    # inside the project's usual 200m geofence but outside a tighter one,
    # which is exactly what's used here to force a deterministic, drone-
    # index-specific rejection without needing an unrealistic spacing.
    tight_geofence_cfg = SafetyConfig(
        home_lat=CFG.home_lat, home_lon=CFG.home_lon,
        max_geofence_radius_m=70.0, min_alt_m=CFG.min_alt_m, max_alt_m=CFG.max_alt_m,
        max_speed_mps=CFG.max_speed_mps, max_leg_distance_m=CFG.max_leg_distance_m,
        loop_closure_tolerance_m=CFG.loop_closure_tolerance_m,
    )
    result = validate_squad_mission_json(
        _formation_squad(drone_count=3, formation="line", spacing_m=50.0), tight_geofence_cfg
    )
    assert not result.ok
    # drone[0] (unmodified leader) and drone[1] (mid, offset 0) stay well
    # inside 70m; drone[2] (offset +50m) is the one pushed out.
    assert any(e.startswith("drone[2]:") for e in result.errors)


def test_split_route_too_few_waypoints_for_drone_count_is_rejected():
    result = validate_squad_mission_json(
        _split_squad(drone_count=5, waypoints=VALID_ROUTE_WAYPOINTS), CFG  # only 2 waypoints
    )
    assert not result.ok


# -- minimum separation -----------------------------------------------

def test_check_min_separation_flags_drones_too_close():
    # spacing_m's schema floor is 3m, so to force a violation the
    # requested minimum separation just needs to be set higher than that
    # floor -- exactly the situation an operator asking for a tight
    # formation with a conservative safety margin would hit.
    tight = validate_squad_mission_json(
        _formation_squad(drone_count=2, spacing_m=3.0), CFG, min_separation_m=5.0
    )
    assert not tight.ok
    assert any("below minimum separation" in e for e in tight.errors)


def test_check_min_separation_passes_with_adequate_spacing():
    roomy = validate_squad_mission_json(_formation_squad(drone_count=2, spacing_m=10.0), CFG, min_separation_m=3.0)
    assert roomy.ok


def test_check_min_separation_direct_call_with_two_identical_plans():
    """Directly exercises check_min_separation() with two plans that are
    (deliberately) identical -- i.e. zero separation -- the most obvious
    possible violation, independent of formation math."""
    result = validate_squad_mission_json(_formation_squad(drone_count=2, spacing_m=10.0), CFG)
    plans = result.plans
    errors = check_min_separation([plans[0], plans[0]], min_separation_m=3.0)
    assert len(errors) > 0
    assert "0.0m apart" in errors[0]


def test_split_mode_skips_separation_check_by_design():
    """Split-mode lanes intentionally sit in different places and don't
    share waypoint indices meaningfully -- separation checking is a
    formation-mode-only concept, this proves split mode isn't blocked by
    it."""
    result = validate_squad_mission_json(_split_squad(drone_count=3), CFG, min_separation_m=1000.0)
    assert result.ok, result.errors


if __name__ == "__main__":
    tests = [
        test_valid_formation_squad_produces_one_plan_per_drone,
        test_valid_split_squad_produces_one_plan_per_drone,
        test_formation_plans_each_pass_normal_per_drone_safety_rules,
        test_rejects_too_few_drones,
        test_rejects_too_many_drones,
        test_rejects_unknown_formation,
        test_per_drone_geofence_violation_is_reported_with_drone_index,
        test_split_route_too_few_waypoints_for_drone_count_is_rejected,
        test_check_min_separation_flags_drones_too_close,
        test_check_min_separation_passes_with_adequate_spacing,
        test_check_min_separation_direct_call_with_two_identical_plans,
        test_split_mode_skips_separation_check_by_design,
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
