"""
squad_executor.py

Challenge 1 -- runs N already-validated MissionPlans concurrently, one per
drone, each through the EXACT SAME MissionExecutor the core task already
built and tested. This file adds no new flight logic at all -- its only
job is fan-out: one MAVSDK connection per drone, one asyncio task per
drone, gathered together, with a per-drone-prefixed audit log at the end.

Same "boring on purpose" philosophy mission_executor.py states for
itself: if this file is deleted, every individual MissionExecutor still
works and is still independently testable exactly as it was for the core
task. Squad behaviour is squad_validator.py (what's safe to fly) plus
this file (how to fly N things at once) -- neither one touches the other's
job.

-------------------------------------------------------------------------
Why separate MAVSDK connections, not separate processes
-------------------------------------------------------------------------
Each simulated drone is its own PX4 SITL instance (instance 0, 1, 2, ...),
each exposing MAVSDK on its own UDP port -- PX4's own convention is
14540 + instance number, which is exactly what --system-address already
parameterizes in mission_executor.py's CLI, so squad_executor.py just
calls that pattern N times instead of once. Running all N inside a single
asyncio event loop (rather than N separate OS processes) keeps the audit
log simple to assemble and keeps this file able to be dry-run tested with
no live PX4 instance at all -- see test_squad_executor.py.

-------------------------------------------------------------------------
Optional live visualization hook (Gazebo is separate; this is for RViz)
-------------------------------------------------------------------------
Gazebo already shows the simulated drones directly -- no extra code
needed for that, same as the core task (see README's "Watching the
simulation" section, extended for multiple vehicles in fly_squad.sh).
RViz is not part of the core task's stack at all, so it's wired in here
as a single optional interface (`viz`, see formation_viz.py) with exactly
two methods: publish_plan() and publish_position(). If `viz` is None
(the default), none of this code runs -- squad flight behaviour is
completely unaffected by whether RViz is even installed. If `viz` is
provided but a call into it raises, that failure is logged and swallowed,
never allowed to abort or stall a flying mission -- visualization must
never be able to get a live drone stuck.
"""

import argparse
import asyncio
import json
import pathlib
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Protocol

from mavsdk import System

from schema import MissionPlan
from mission_executor import MissionExecutor
from validator import SafetyConfig
from squad_validator import validate_squad_mission_json


class VizPublisher(Protocol):
    """See formation_viz.py for the RViz implementation of this. Any
    object with these two methods works -- squad_executor.py never
    imports rclpy or anything ROS-specific itself."""

    def publish_plan(self, drone_index: int, plan: MissionPlan) -> None: ...

    def publish_position(self, drone_index: int, lat: float, lon: float, alt_m: float) -> None: ...


@dataclass
class SquadExecutionResult:
    ok: bool
    audit_logs: List[list] = field(default_factory=list)  # audit_logs[i] = drone i's ExecutedCommand list
    errors: List[str] = field(default_factory=list)


def _safe_viz_call(fn_name: str, drone_index: int, fn, *args) -> None:
    """Visualization must never be able to stall or crash a live flight --
    see module docstring. Any exception from the viz layer is printed and
    dropped."""
    try:
        fn(*args)
    except Exception as e:
        print(f"-- [viz] drone[{drone_index}] {fn_name} failed (non-fatal): {e}")


async def _poll_position_for_viz(
    drone: Optional[System], drone_index: int, viz: VizPublisher, dry_run: bool, stop_event: asyncio.Event
) -> None:
    """Background task, one per drone, purely for RViz -- publishes live
    telemetry position at roughly 2Hz until the drone's mission finishes.
    Runs alongside (not inside) MissionExecutor.execute(), so a slow or
    failing viz publish can never block a waypoint arrival check."""
    if dry_run or drone is None:
        return
    async for position in drone.telemetry.position():
        if stop_event.is_set():
            return
        _safe_viz_call(
            "publish_position", drone_index, viz.publish_position,
            drone_index, position.latitude_deg, position.longitude_deg, position.relative_altitude_m,
        )
        await asyncio.sleep(0.5)


async def _run_one_drone(
    drone_index: int,
    plan: MissionPlan,
    reference_lat: float,
    reference_lon: float,
    system_address: str,
    rtl_return_alt_m: Optional[float],
    dry_run: bool,
    viz: Optional[VizPublisher],
    connect_timeout_s: float = 30.0,
) -> list:
    """One drone's full lifecycle: connect, fly, disconnect -- wrapping
    the same MissionExecutor the core task uses for a single drone,
    unchanged.

    connect_timeout_s matters specifically for squads: mission_executor.py's
    connect_and_wait_ready() has no timeout at all (fine for a single
    drone you're watching directly), but a squad launched from
    fly_squad.sh depends on the right NUMBER of PX4 instances already
    being up on the right ports before this runs -- get that count wrong
    (or one instance fails to boot) and, without a bound here, that one
    drone would hang forever, silently stalling the whole squad's
    asyncio.gather() with it. A clear timeout error is the fast-fail
    version of that story."""
    if viz is not None:
        _safe_viz_call("publish_plan", drone_index, viz.publish_plan, drone_index, plan)

    # A plain System() per drone, exactly like mission_executor.py's own
    # CLI entry point -- MAVSDK manages a separate mavsdk_server subprocess
    # per System() instance automatically, so N drones just means N of
    # these, each pointed at its own PX4 instance's system_address below.
    drone = None if dry_run else System()
    executor = MissionExecutor(
        drone, reference_lat, reference_lon,
        rtl_return_alt_m=rtl_return_alt_m,
        dry_run=dry_run,
    )
    print(f"-- drone[{drone_index}]: connecting on {system_address} (timeout {connect_timeout_s}s) ...")
    try:
        await asyncio.wait_for(executor.connect_and_wait_ready(system_address), timeout=connect_timeout_s)
    except asyncio.TimeoutError:
        raise TimeoutError(
            f"no PX4 instance answered on {system_address} within {connect_timeout_s}s -- "
            f"check fly_squad.sh launched at least {drone_index + 1} instance(s), "
            f"and that instance {drone_index}'s log doesn't show a startup failure"
        )

    stop_event = asyncio.Event()
    viz_task = None
    if viz is not None:
        viz_task = asyncio.create_task(_poll_position_for_viz(drone, drone_index, viz, dry_run, stop_event))

    try:
        log = await executor.execute(plan)
    finally:
        stop_event.set()
        if viz_task is not None:
            viz_task.cancel()

    print(f"-- drone[{drone_index}]: mission complete ({len(log)} commands)")
    return log


async def fly_squad(
    plans: List[MissionPlan],
    reference_lat: float,
    reference_lon: float,
    rtl_return_alt_m: Optional[float] = None,
    base_port: int = 14540,
    dry_run: bool = False,
    viz: Optional[VizPublisher] = None,
    connect_timeout_s: float = 30.0,
) -> SquadExecutionResult:
    """Flies every plan concurrently, one MissionExecutor per drone. Each
    drone connects to udpin://0.0.0.0:{base_port + drone_index} -- PX4's
    own instance-numbering convention (instance 0 -> 14540, instance 1 ->
    14541, ...), see fly_squad.sh for how the matching PX4 instances are
    launched.

    Uses asyncio.gather with return_exceptions=True on purpose: one
    drone's connection failure must not silently strand the others mid-
    mission with no report -- every drone's outcome (success or the
    exception it raised) is collected and reported, not just the first
    one to fail."""
    tasks = [
        _run_one_drone(
            i, plan, reference_lat, reference_lon,
            f"udpin://0.0.0.0:{base_port + i}",
            rtl_return_alt_m, dry_run, viz,
            connect_timeout_s=connect_timeout_s,
        )
        for i, plan in enumerate(plans)
    ]
    results = await asyncio.gather(*tasks, return_exceptions=True)

    audit_logs: List[list] = []
    errors: List[str] = []
    for i, r in enumerate(results):
        if isinstance(r, Exception):
            errors.append(f"drone[{i}]: {type(r).__name__}: {r}")
            audit_logs.append([])
        else:
            audit_logs.append(r)

    return SquadExecutionResult(ok=(len(errors) == 0), audit_logs=audit_logs, errors=errors)


# -- CLI entry point --------------------------------------------------------

def _default_safety_config() -> SafetyConfig:
    return SafetyConfig(
        home_lat=10.5270, home_lon=76.2140,
        max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
        max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
    )


async def run_squad_from_file(
    squad_mission_path: pathlib.Path,
    safety_cfg: SafetyConfig,
    base_port: int = 14540,
    dry_run: bool = False,
    min_separation_m: float = 3.0,
    use_rviz: bool = False,
) -> None:
    raw = json.loads(squad_mission_path.read_text())
    raw.pop("_expect", None)
    raw.pop("_note", None)

    result = validate_squad_mission_json(raw, safety_cfg, min_separation_m)
    if not result.ok:
        print("-- Squad mission REJECTED, not executing. Errors:")
        for err in result.errors:
            print(f"   - {err}")
        sys.exit(1)

    print(f"-- Squad mission validated OK: {len(result.plans)} drone(s)")

    viz = None
    if use_rviz:
        try:
            from formation_viz import RvizFormationPublisher
            viz = RvizFormationPublisher()
            print("-- RViz publisher connected")
        except Exception as e:
            print(f"-- WARNING: could not start RViz publisher, continuing without it: {e}")

    exec_result = await fly_squad(
        result.plans, safety_cfg.home_lat, safety_cfg.home_lon,
        rtl_return_alt_m=safety_cfg.max_alt_m, base_port=base_port,
        dry_run=dry_run, viz=viz,
    )

    print("\n-- Squad audit logs:")
    for i, log in enumerate(exec_result.audit_logs):
        print(f"\n-- drone[{i}] --")
        print(json.dumps([{"step": e.step, "action": e.action, "params": e.params} for e in log], indent=2))

    if not exec_result.ok:
        print("\n-- One or more drones FAILED:")
        for err in exec_result.errors:
            print(f"   - {err}")
        sys.exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a validated squad mission JSON against N PX4 SITL instances.")
    parser.add_argument("squad_mission_file", type=pathlib.Path,
                         help="Path to a squad mission JSON file, e.g. fixtures/valid_wedge_formation.json")
    parser.add_argument("--base-port", type=int, default=14540)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--min-separation-m", type=float, default=3.0)
    parser.add_argument("--rviz", action="store_true", help="Publish live positions + planned routes to RViz")
    args = parser.parse_args()

    asyncio.run(run_squad_from_file(
        args.squad_mission_file, _default_safety_config(),
        args.base_port, args.dry_run, args.min_separation_m, args.rviz,
    ))
