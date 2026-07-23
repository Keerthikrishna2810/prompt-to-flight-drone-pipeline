"""
test_vision_executor.py

Tests for vision_executor.py's search-then-follow logic, all via
dry_run=True with MockDetector/NullCameraSource -- same philosophy as
test_squad_executor.py: dry-run exercises the identical branching and
math a live run would, with no drone or camera needed.

Run with:  pytest test_vision_executor.py -v
"""

import pathlib

import pytest

from schema import MissionPlan, Waypoint, MissionType
from vision_schema import VisionFollowPlan
from vision_detector import Detection, MockDetector
from vision_executor import VisionFollowExecutor, NullCameraSource

REFERENCE_LAT, REFERENCE_LON = 10.5270, 76.2140

SEARCH_PLAN = MissionPlan(
    mission_type=MissionType.ROUTE,
    waypoints=[
        Waypoint(lat=10.5270, lon=76.2140, alt_m=10),
        Waypoint(lat=10.5272, lon=76.2143, alt_m=10),
        Waypoint(lat=10.5274, lon=76.2146, alt_m=10),
    ],
    repetitions=1,
    speed_mps=4,
)

VISION_PLAN = VisionFollowPlan(
    target_class="person",
    search_waypoints=SEARCH_PLAN.waypoints,
    search_speed_mps=4,
    follow_distance_m=8,
    follow_altitude_m=10,
    max_follow_duration_s=10.0,  # with follow_poll_interval_s=3.0 below -> 3 follow iterations max
    detection_confidence_threshold=0.5,
)

MATCH = Detection(class_name="person", confidence=0.9,
                   bbox_x_center=0.5, bbox_y_center=0.5, bbox_width=0.25, bbox_height=0.4)


def _executor(tmp_path, detector, max_geofence_radius_m=None):
    return VisionFollowExecutor(
        drone=None,
        reference_lat=REFERENCE_LAT,
        reference_lon=REFERENCE_LON,
        camera=NullCameraSource(),
        detector=detector,
        snapshot_dir=tmp_path / "snapshots",
        max_geofence_radius_m=max_geofence_radius_m,
        rtl_return_alt_m=25.0,
        follow_poll_interval_s=3.0,
        dry_run=True,
    )


# -- target never found ----------------------------------------------

def test_target_never_found_returns_to_launch_without_following(tmp_path):
    detector = MockDetector([[]])  # never matches, for every search waypoint
    executor = _executor(tmp_path, detector)

    import asyncio
    result = asyncio.run(executor.execute(VISION_PLAN, SEARCH_PLAN))

    assert result.ok
    assert not result.target_found
    assert result.snapshot_path is None
    actions = [e.action for e in result.audit_log]
    assert "target_detected" not in actions
    assert "search_complete_no_target" in actions
    assert actions[-2] == "return_to_launch" or "return_to_launch" in actions
    # exactly 3 search goto_location calls, one per waypoint, none followed
    assert actions.count("goto_location") == 3
    assert "follow_goto" not in actions


# -- target found, then followed until max duration ----------------------

def test_target_found_and_followed_until_max_duration(tmp_path):
    # no match at waypoint 0, match at waypoint 1 (search breaks there),
    # then matches for every follow-loop poll after that
    detector = MockDetector([[], [MATCH], [MATCH], [MATCH], [MATCH]])
    executor = _executor(tmp_path, detector)

    import asyncio
    result = asyncio.run(executor.execute(VISION_PLAN, SEARCH_PLAN))

    assert result.ok
    assert result.target_found
    assert result.snapshot_path is not None
    assert pathlib.Path(result.snapshot_path).exists()

    actions = [e.action for e in result.audit_log]
    assert actions.count("goto_location") == 2  # stopped searching after waypoint[1]
    assert "target_detected" in actions
    # max_follow_duration_s=6 / follow_poll_interval_s=2 -> exactly 3 follow iterations
    assert actions.count("follow_goto") == 3
    follow_ended = next(e for e in result.audit_log if e.action == "follow_ended")
    assert follow_ended.params["reason"] == "max_follow_duration_reached"


def test_target_lost_mid_follow_stops_early(tmp_path):
    # matches at both search waypoints checked, then lost immediately in follow phase
    detector = MockDetector([[MATCH], [], []])
    executor = _executor(tmp_path, detector)

    import asyncio
    result = asyncio.run(executor.execute(VISION_PLAN, SEARCH_PLAN))

    assert result.target_found  # found during search
    actions = [e.action for e in result.audit_log]
    assert actions.count("goto_location") == 1  # found on the very first waypoint
    assert actions.count("follow_goto") == 0    # lost before any follow move was made
    follow_ended = next(e for e in result.audit_log if e.action == "follow_ended")
    assert follow_ended.params["reason"] == "target_lost"


def test_geofence_breach_stops_follow_before_any_follow_move(tmp_path):
    detector = MockDetector([[], [MATCH], [MATCH], [MATCH]])
    # max_geofence_radius_m=0 guarantees the very first follow-loop check
    # (evaluated before polling the camera) trips immediately, since the
    # search route has already moved some distance from home.
    executor = _executor(tmp_path, detector, max_geofence_radius_m=0.0)

    import asyncio
    result = asyncio.run(executor.execute(VISION_PLAN, SEARCH_PLAN))

    assert result.target_found
    actions = [e.action for e in result.audit_log]
    assert actions.count("follow_goto") == 0
    follow_ended = next(e for e in result.audit_log if e.action == "follow_ended")
    assert follow_ended.params["reason"] == "geofence_exceeded"


# -- provably bounded, never an infinite loop ------------------------

def test_follow_loop_is_bounded_even_with_permanent_detections(tmp_path):
    """The core safety property: however long max_follow_duration_s /
    follow_poll_interval_s implies, the loop executes AT MOST that many
    iterations -- structurally, not just 'usually because the clock
    behaves'. This test would hang forever with a while-True design if
    something upstream were wrong; here it can't, by construction."""
    detector = MockDetector([[], [MATCH]])  # after search, every subsequent call returns MATCH forever
    executor = _executor(tmp_path, detector)

    import asyncio
    result = asyncio.run(executor.execute(VISION_PLAN, SEARCH_PLAN))

    follow_goto_count = sum(1 for e in result.audit_log if e.action == "follow_goto")
    max_iterations = int(VISION_PLAN.max_follow_duration_s / executor.follow_poll_interval_s)
    assert follow_goto_count <= max_iterations


if __name__ == "__main__":
    tests = [
        test_target_never_found_returns_to_launch_without_following,
        test_target_found_and_followed_until_max_duration,
        test_target_lost_mid_follow_stops_early,
        test_geofence_breach_stops_follow_before_any_follow_move,
        test_follow_loop_is_bounded_even_with_permanent_detections,
    ]
    import tempfile
    passed, failed = 0, 0
    for t in tests:
        try:
            with tempfile.TemporaryDirectory() as d:
                t(pathlib.Path(d))
            print(f"[PASS] {t.__name__}")
            passed += 1
        except AssertionError as e:
            print(f"[FAIL] {t.__name__}: {e}")
            failed += 1
    print(f"\n{passed} passed, {failed} failed")
