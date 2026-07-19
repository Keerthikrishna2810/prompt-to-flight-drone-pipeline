"""
test_formation.py

Tests for formation.py's pure geometry -- no PX4, no MAVSDK, no LLM.
Same philosophy as test_executor.py's coordinate re-homing tests: prove
the math with plain asserts before trusting it anywhere near a simulator.

Run with:  pytest test_formation.py -v
"""

import math

import pytest

from schema import Waypoint
from validator import _haversine_m
from formation import (
    FormationType,
    slot_offsets,
    apply_formation,
    split_route_across_drones,
    _bearing_deg,
)

# A simple two-point route heading due east, near the project's usual
# reference point -- easy to reason about by hand.
EAST_ROUTE = [
    Waypoint(lat=10.5270, lon=76.2140, alt_m=10),
    Waypoint(lat=10.5270, lon=76.2160, alt_m=10),
]

SQUARE_LOOP = [
    Waypoint(lat=10.5270, lon=76.2140, alt_m=10),
    Waypoint(lat=10.5274, lon=76.2140, alt_m=10),
    Waypoint(lat=10.5274, lon=76.2144, alt_m=10),
    Waypoint(lat=10.5270, lon=76.2144, alt_m=10),
    Waypoint(lat=10.5270, lon=76.2140, alt_m=10),
]


def test_bearing_due_east_is_90():
    b = _bearing_deg(10.5270, 76.2140, 10.5270, 76.2160)
    assert b == pytest.approx(90.0, abs=0.5)


# -- slot_offsets: shape correctness, no lat/lon involved --------------

def test_line_formation_is_symmetric_about_center():
    slots = slot_offsets(FormationType.LINE, 3, spacing_m=10.0)
    assert slots[0] == (0.0, -10.0)
    assert slots[1] == (0.0, 0.0)
    assert slots[2] == (0.0, 10.0)


def test_column_formation_falls_straight_back():
    slots = slot_offsets(FormationType.COLUMN, 3, spacing_m=5.0)
    assert slots[0] == (0.0, 0.0)
    assert slots[1] == (-5.0, 0.0)
    assert slots[2] == (-10.0, 0.0)


def test_wedge_formation_point_leads_wings_fall_back_and_out():
    slots = slot_offsets(FormationType.WEDGE, 3, spacing_m=8.0)
    assert slots[0] == (0.0, 0.0)  # point of the wedge
    # wing 1 back-left, wing 2 back-right, same rank
    assert slots[1] == (-8.0, -8.0)
    assert slots[2] == (-8.0, 8.0)


def test_box_formation_fills_row_major():
    slots = slot_offsets(FormationType.BOX, 4, spacing_m=6.0)
    assert len(slots) == 4
    # 4 drones -> 2 columns -> 2 rows of 2
    assert slots[0] == (0.0, 0.0)
    assert slots[1] == (0.0, 6.0)
    assert slots[2] == (-6.0, 0.0)
    assert slots[3] == (-6.0, 6.0)


def test_slot_offsets_rejects_single_drone():
    with pytest.raises(ValueError):
        slot_offsets(FormationType.LINE, 1, spacing_m=10.0)


# -- apply_formation: full lat/lon route expansion -----------------------

def test_apply_formation_returns_one_route_per_drone():
    routes = apply_formation(EAST_ROUTE, FormationType.LINE, 3, spacing_m=10.0)
    assert len(routes) == 3
    for route in routes:
        assert len(route) == len(EAST_ROUTE)


def test_leader_drone_flies_original_route_unmodified():
    """Drone 0 (index 0, the 'lead') should fly exactly the input route --
    formations are defined relative to it, not the other way around."""
    routes = apply_formation(EAST_ROUTE, FormationType.WEDGE, 3, spacing_m=8.0)
    leader_route = routes[0]
    for original, flown in zip(EAST_ROUTE, leader_route):
        assert flown.lat == pytest.approx(original.lat, abs=1e-9)
        assert flown.lon == pytest.approx(original.lon, abs=1e-9)


def test_line_formation_spacing_is_correct_on_real_earth():
    """For an east-heading route, a LINE formation's offset is
    perpendicular to travel -- i.e. north/south. Spacing between adjacent
    drones, measured with the real haversine distance, should match
    spacing_m closely at this scale."""
    routes = apply_formation(EAST_ROUTE, FormationType.LINE, 3, spacing_m=12.0)
    wp_a = routes[0][0]
    wp_b = routes[1][0]
    dist = _haversine_m(wp_a.lat, wp_a.lon, wp_b.lat, wp_b.lon)
    assert dist == pytest.approx(12.0, abs=0.1)


def test_formation_offsets_are_consistent_across_all_waypoints():
    """The whole squad should keep its shape for the ENTIRE route, not
    just the first waypoint -- i.e. every drone's route should be a
    rigid translation of the leader's route."""
    routes = apply_formation(SQUARE_LOOP, FormationType.WEDGE, 3, spacing_m=10.0)
    leader, wing = routes[0], routes[1]
    dists = [
        _haversine_m(l.lat, l.lon, w.lat, w.lon)
        for l, w in zip(leader, wing)
    ]
    for d in dists[1:]:
        assert d == pytest.approx(dists[0], abs=0.5)


def test_apply_formation_preserves_altitude_per_waypoint():
    varied_alt_route = [
        Waypoint(lat=10.5270, lon=76.2140, alt_m=8),
        Waypoint(lat=10.5270, lon=76.2160, alt_m=15),
    ]
    routes = apply_formation(varied_alt_route, FormationType.COLUMN, 2, spacing_m=10.0)
    for route in routes:
        assert route[0].alt_m == 8
        assert route[1].alt_m == 15


def test_apply_formation_rejects_single_waypoint_route():
    with pytest.raises(ValueError):
        apply_formation([EAST_ROUTE[0]], FormationType.LINE, 2, spacing_m=10.0)


# -- split_route_across_drones -------------------------------------------

def test_split_route_covers_full_original_route():
    lanes = split_route_across_drones(SQUARE_LOOP, 4)
    assert len(lanes) == 4
    # First lane starts where the original route starts, last lane ends
    # where the original route ends -- no gap at either edge of coverage.
    assert lanes[0][0] == SQUARE_LOOP[0]
    assert lanes[-1][-1] == SQUARE_LOOP[-1]


def test_split_route_lanes_overlap_at_seams():
    """Adjacent lanes should share their boundary waypoint -- otherwise
    there's a gap in area coverage between two drones' lanes."""
    lanes = split_route_across_drones(SQUARE_LOOP, 4)
    for i in range(len(lanes) - 1):
        assert lanes[i][-1] == lanes[i + 1][0]


def test_split_route_rejects_too_few_waypoints_for_drone_count():
    with pytest.raises(ValueError):
        split_route_across_drones(EAST_ROUTE, 5)  # 2 waypoints, 5 drones


def test_split_route_every_lane_has_at_least_two_points():
    lanes = split_route_across_drones(SQUARE_LOOP, 4)
    for lane in lanes:
        assert len(lane) >= 2


if __name__ == "__main__":
    tests = [
        test_bearing_due_east_is_90,
        test_line_formation_is_symmetric_about_center,
        test_column_formation_falls_straight_back,
        test_wedge_formation_point_leads_wings_fall_back_and_out,
        test_box_formation_fills_row_major,
        test_slot_offsets_rejects_single_drone,
        test_apply_formation_returns_one_route_per_drone,
        test_leader_drone_flies_original_route_unmodified,
        test_line_formation_spacing_is_correct_on_real_earth,
        test_formation_offsets_are_consistent_across_all_waypoints,
        test_apply_formation_preserves_altitude_per_waypoint,
        test_apply_formation_rejects_single_waypoint_route,
        test_split_route_covers_full_original_route,
        test_split_route_lanes_overlap_at_seams,
        test_split_route_rejects_too_few_waypoints_for_drone_count,
        test_split_route_every_lane_has_at_least_two_points,
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
