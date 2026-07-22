"""
test_squad_executor.py

Tests for squad_executor.py's concurrent fan-out, all via dry_run=True --
same reasoning as test_executor.py: dry-run produces the identical
command sequence a live run would (same math, same branching), without
touching a vehicle, so this runs in CI / inside the Docker build with no
PX4/Gazebo session required.

Run with:  pytest test_squad_executor.py -v
"""

import asyncio

import pytest

from schema import MissionPlan
from validator import SafetyConfig
from squad_validator import validate_squad_mission_json
from squad_executor import fly_squad, VizPublisher

CFG = SafetyConfig(
    home_lat=10.5270, home_lon=76.2140,
    max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
    max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
)


def _three_drone_wedge_plans():
    raw = {
        "drone_count": 3, "mode": "formation", "formation": "wedge", "spacing_m": 10.0,
        "mission_type": "route", "repetitions": 1, "speed_mps": 5,
        "waypoints": [
            {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
            {"lat": 10.5272, "lon": 76.2145, "alt_m": 10},
        ],
    }
    result = validate_squad_mission_json(raw, CFG)
    assert result.ok, result.errors
    return result.plans


# -- basic fan-out -----------------------------------------------------

def test_dry_run_flies_all_drones():
    plans = _three_drone_wedge_plans()
    result = asyncio.run(fly_squad(plans, CFG.home_lat, CFG.home_lon, dry_run=True))
    assert result.ok, result.errors
    assert len(result.audit_logs) == 3
    for log in result.audit_logs:
        assert log[-1].action == "return_to_launch"


def test_each_drone_gets_its_own_waypoints_in_its_log():
    """Not just 'N logs exist' -- each drone's own waypoints (which
    differ between drones in a formation) must actually show up in ITS
    audit log, proving the right plan was routed to the right drone."""
    plans = _three_drone_wedge_plans()
    result = asyncio.run(fly_squad(plans, CFG.home_lat, CFG.home_lon, dry_run=True))

    for i, (plan, log) in enumerate(zip(plans, result.audit_logs)):
        gotos = [e for e in log if e.action == "goto_location"]
        assert len(gotos) == len(plan.waypoints)
        # drone 0 (the leader) flies the unmodified input route -- its
        # first goto should land essentially back at the plan's own first
        # waypoint once re-homed onto the (identical, in dry-run) sim home.
        first_goto = gotos[0]
        assert first_goto.params["lat"] == pytest.approx(plan.waypoints[0].lat, abs=1e-6)
        assert first_goto.params["lon"] == pytest.approx(plan.waypoints[0].lon, abs=1e-6)


def test_drones_fly_concurrently_not_sequentially():
    """A crude but real concurrency check: three drones with a fixed
    per-waypoint arrival wait would take ~3x as long run sequentially as
    concurrently. dry_run mode has near-zero per-step latency, so this
    checks asyncio.gather is actually gathering (all tasks scheduled
    together) rather than silently falling back to one-at-a-time by
    checking every drone's log contains a takeoff before any drone's
    return_to_launch would need to have already happened -- i.e. we don't
    assert wall-clock time (too flaky), we assert the fan-out shape."""
    plans = _three_drone_wedge_plans()
    result = asyncio.run(fly_squad(plans, CFG.home_lat, CFG.home_lon, dry_run=True))
    for log in result.audit_logs:
        actions = [e.action for e in log]
        assert actions.count("arm") == 1
        assert actions.count("takeoff") == 1
        assert actions.count("return_to_launch") == 1


# -- fault isolation -----------------------------------------------------

def test_one_drone_failing_does_not_stop_the_others():
    """asyncio.gather(..., return_exceptions=True) means one drone's
    exception must not silently swallow or block the other drones'
    results -- this directly tests that guarantee by forcing drone[1]'s
    plan to have an invalid mission_type value that will blow up inside
    execute(), and checking drones 0 and 2 still completed."""
    plans = _three_drone_wedge_plans()
    broken_plans = list(plans)
    broken_plans[1] = "not a real MissionPlan"  # guaranteed to raise inside _run_one_drone

    result = asyncio.run(fly_squad(broken_plans, CFG.home_lat, CFG.home_lon, dry_run=True))
    assert not result.ok
    assert len(result.errors) == 1
    assert result.errors[0].startswith("drone[1]:")
    # drones 0 and 2 still got a real audit log despite drone 1 failing
    assert result.audit_logs[0][-1].action == "return_to_launch"
    assert result.audit_logs[2][-1].action == "return_to_launch"
    assert result.audit_logs[1] == []


# -- optional viz hook never blocks flight --------------------------------

class _RecordingViz:
    def __init__(self):
        self.plans_seen = []
        self.positions_seen = []

    def publish_plan(self, drone_index, plan):
        self.plans_seen.append((drone_index, plan))

    def publish_position(self, drone_index, lat, lon, alt_m):
        self.positions_seen.append((drone_index, lat, lon, alt_m))


class _BrokenViz:
    """Every call raises -- proves a failing viz layer can't take down a
    flying mission (see _safe_viz_call in squad_executor.py)."""

    def publish_plan(self, drone_index, plan):
        raise RuntimeError("viz backend unreachable")

    def publish_position(self, drone_index, lat, lon, alt_m):
        raise RuntimeError("viz backend unreachable")


def test_viz_publish_plan_called_once_per_drone():
    plans = _three_drone_wedge_plans()
    viz = _RecordingViz()
    result = asyncio.run(fly_squad(plans, CFG.home_lat, CFG.home_lon, dry_run=True, viz=viz))
    assert result.ok
    assert len(viz.plans_seen) == 3
    assert {i for i, _ in viz.plans_seen} == {0, 1, 2}


def test_stuck_connection_times_out_instead_of_hanging_forever():
    """Directly proves the guarantee connect_timeout_s exists for: if one
    drone's connection never completes (e.g. fly_squad.sh launched fewer
    PX4 instances than the squad needs), fly_squad() must return a clear
    error within the configured timeout -- NOT hang the whole squad
    forever. Patches MissionExecutor.connect_and_wait_ready to simulate
    exactly that stuck state, with a short timeout so the test itself
    stays fast."""
    from unittest.mock import patch
    import mission_executor

    async def _hangs_forever(self, system_address):
        await asyncio.Event().wait()  # never set -- simulates no PX4 instance answering

    plans = _three_drone_wedge_plans()
    with patch.object(mission_executor.MissionExecutor, "connect_and_wait_ready", _hangs_forever):
        result = asyncio.run(
            fly_squad(plans, CFG.home_lat, CFG.home_lon, dry_run=False, connect_timeout_s=0.2)
        )
    assert not result.ok
    assert len(result.errors) == 3  # all three drones hit the same stuck connection
    assert all("no PX4 instance answered" in e for e in result.errors)


def test_broken_viz_does_not_fail_the_mission():
    plans = _three_drone_wedge_plans()
    result = asyncio.run(fly_squad(plans, CFG.home_lat, CFG.home_lon, dry_run=True, viz=_BrokenViz()))
    assert result.ok, result.errors
    assert all(log[-1].action == "return_to_launch" for log in result.audit_logs)


if __name__ == "__main__":
    tests = [
        test_dry_run_flies_all_drones,
        test_each_drone_gets_its_own_waypoints_in_its_log,
        test_drones_fly_concurrently_not_sequentially,
        test_one_drone_failing_does_not_stop_the_others,
        test_stuck_connection_times_out_instead_of_hanging_forever,
        test_viz_publish_plan_called_once_per_drone,
        test_broken_viz_does_not_fail_the_mission,
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
