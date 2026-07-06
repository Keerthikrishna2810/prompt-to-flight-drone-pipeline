"""
mission_executor.py

Day 3 -- the deterministic executor.

Takes an already-validated MissionPlan (see schema.py / validator.py) and
turns it into a MAVSDK command sequence. This file has ZERO knowledge of
where the MissionPlan came from -- hand-written fixture, future LLM output,
anything else. It is deliberately the most boring file in the project:
same MissionPlan in, same command sequence out, every single time.

No LLM imports here, on purpose. If this file ever needs to import
anything from an `llm_*` module, that's a sign the architecture boundary
between "decides what to fly" and "flies it" has been broken.

--------------------------------------------------------------------------
Why there's coordinate re-homing math in here
--------------------------------------------------------------------------
The Day 2 fixtures (and SafetyConfig.home_lat/home_lon) use an arbitrary
reference position for the geofence/distance math -- it does NOT have to
match wherever the simulator actually spawns the vehicle (PX4 SITL's
default world spawns near Zurich; the fixtures were authored against a
Kerala reference point). Sending fixture lat/lon straight to goto_location
would therefore try to fly the drone thousands of km across the planet
instead of the intended local pattern.

The fix: treat SafetyConfig's home position as the frame the *pattern* was
authored in, compute each waypoint as a north/east metre offset from that
frame, then re-apply the same offsets onto the sim's *actual* home
position (read from telemetry at runtime). The flown pattern is identical
either way -- only its real-world anchor point changes. This also means
the exact same mission JSON works unmodified against any simulator world,
not just the one it happened to be authored against.
"""

import argparse
import asyncio
import json
import math
import pathlib
import sys
from dataclasses import dataclass, field
from typing import Callable, List, Optional

from mavsdk import System

from schema import MissionPlan, MissionType
from validator import SafetyConfig, validate_mission_json


_EARTH_RADIUS_M = 6371000.0


def _offset_from_reference(ref_lat: float, ref_lon: float, lat: float, lon: float):
    """Small-distance equirectangular approximation: (north_m, east_m) of
    (lat, lon) relative to (ref_lat, ref_lon). Accurate to well under 1%
    at the scales this project operates at (tens to low hundreds of
    metres) -- not meant for anything beyond local pattern geometry."""
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)
    north_m = d_lat * _EARTH_RADIUS_M
    east_m = d_lon * _EARTH_RADIUS_M * math.cos(math.radians(ref_lat))
    return north_m, east_m


def _apply_offset(base_lat: float, base_lon: float, north_m: float, east_m: float):
    """Inverse of _offset_from_reference: apply a north/east metre offset
    to a base lat/lon, returning the resulting (lat, lon)."""
    d_lat = north_m / _EARTH_RADIUS_M
    d_lon = east_m / (_EARTH_RADIUS_M * math.cos(math.radians(base_lat)))
    return base_lat + math.degrees(d_lat), base_lon + math.degrees(d_lon)


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres. Same formula as validator.py's
    version, duplicated here rather than imported so this file has no
    dependency on validator internals -- keeps the executor's only real
    coupling to schema.py's MissionPlan, which is the actual contract."""
    r = _EARTH_RADIUS_M
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlambda / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


@dataclass
class ExecutedCommand:
    """One entry in the audit log -- proof of exactly what was commanded,
    in order. Same MissionPlan should produce byte-identical sequences of
    these (params only, ignore wall-clock timing) across separate runs --
    that's the determinism claim, and this log is what lets you actually
    check it rather than just assert it in a write-up."""
    step: int
    action: str
    params: dict


class MissionExecutor:
    def __init__(
        self,
        drone: Optional[System],
        reference_lat: float,
        reference_lon: float,
        takeoff_alt_m: float = 10.0,
        rtl_return_alt_m: Optional[float] = None,
        dry_run: bool = False,
    ):
        self.drone = drone
        self.reference_lat = reference_lat
        self.reference_lon = reference_lon
        self.takeoff_alt_m = takeoff_alt_m
        self.rtl_return_alt_m = rtl_return_alt_m
        self.dry_run = dry_run
        self.audit_log: List[ExecutedCommand] = []
        self._step = 0

    # -- bookkeeping ---------------------------------------------------

    def _log(self, action: str, **params) -> None:
        self._step += 1
        entry = ExecutedCommand(step=self._step, action=action, params=params)
        self.audit_log.append(entry)
        print(f"   [{entry.step:02d}] {action} {params}")

    async def _do(self, action: str, call: Optional[Callable] = None, **params) -> None:
        """Log the action, then actually perform it -- unless dry_run,
        in which case only the log entry is produced. `call` is a
        zero-arg callable returning the awaitable (a lambda), so nothing
        is ever awaited/constructed in dry-run mode."""
        self._log(action, **params)
        if not self.dry_run and call is not None:
            await call()

    async def _wait_until_altitude(self, target_agl_m: float, label: str,
                                    tolerance_m: float = 1.0, timeout_s: float = 20.0,
                                    poll_interval_s: float = 2.0) -> None:
        """Poll until relative (AGL) altitude is within tolerance of the
        target, or give up after timeout_s. Used after takeoff, where
        there's no lat/lon target -- only altitude matters."""
        if self.dry_run:
            return
        print(f"-- Waiting for altitude: {label} (target {target_agl_m}m AGL, "
              f"tolerance {tolerance_m}m, timeout {timeout_s}s)")
        elapsed = 0.0
        async for position in self.drone.telemetry.position():
            alt = position.relative_altitude_m
            print(f"   t+{elapsed:>4.1f}s  alt={alt:.1f}m AGL")
            if abs(alt - target_agl_m) <= tolerance_m:
                print(f"   -- Reached target altitude at t+{elapsed:.1f}s")
                return
            elapsed += poll_interval_s
            if elapsed >= timeout_s:
                print(f"   -- WARNING: altitude not reached within {timeout_s}s "
                      f"(last alt={alt:.1f}m), proceeding anyway")
                return
            await asyncio.sleep(poll_interval_s)

    async def _wait_until_arrived(self, target_lat: float, target_lon: float,
                                   target_alt_amsl: float, label: str,
                                   tolerance_m: float = 3.0, timeout_s: float = 25.0,
                                   poll_interval_s: float = 2.0) -> None:
        """Poll position until within `tolerance_m` (horizontal + vertical)
        of the target, or give up after timeout_s and proceed anyway
        (logged loudly as a warning -- a stuck/unreachable waypoint should
        be visible in the log, not silently swallowed)."""
        if self.dry_run:
            return
        print(f"-- Waiting for arrival: {label} (tolerance {tolerance_m}m, timeout {timeout_s}s)")
        elapsed = 0.0
        async for position in self.drone.telemetry.position():
            dist = _haversine_m(position.latitude_deg, position.longitude_deg, target_lat, target_lon)
            alt_diff = abs(position.absolute_altitude_m - target_alt_amsl)
            print(f"   t+{elapsed:>4.1f}s  lat={position.latitude_deg:.6f}  "
                  f"lon={position.longitude_deg:.6f}  alt={position.relative_altitude_m:.1f}m AGL  "
                  f"dist_to_target={dist:.1f}m  alt_diff={alt_diff:.1f}m")
            if dist <= tolerance_m and alt_diff <= tolerance_m:
                print(f"   -- Arrived within tolerance at t+{elapsed:.1f}s")
                return
            elapsed += poll_interval_s
            if elapsed >= timeout_s:
                print(f"   -- WARNING: did not reach tolerance within {timeout_s}s "
                      f"(last dist={dist:.1f}m) -- proceeding to next command anyway")
                return
            await asyncio.sleep(poll_interval_s)

    async def _wait_until_landed(self, label: str, timeout_s: float = 40.0,
                                  poll_interval_s: float = 2.0) -> None:
        """Poll the `in_air` flag until it goes False (landed + disarmed
        territory), rather than guessing a fixed RTL duration."""
        if self.dry_run:
            return
        print(f"-- Waiting for landing: {label} (timeout {timeout_s}s)")
        elapsed = 0.0
        async for in_air in self.drone.telemetry.in_air():
            print(f"   t+{elapsed:>4.1f}s  in_air={in_air}")
            if not in_air:
                print(f"   -- Landed at t+{elapsed:.1f}s")
                return
            elapsed += poll_interval_s
            if elapsed >= timeout_s:
                print(f"   -- WARNING: still airborne after {timeout_s}s, proceeding anyway")
                return
            await asyncio.sleep(poll_interval_s)

    # -- connection ------------------------------------------------------

    async def connect_and_wait_ready(self, system_address: str = "udpin://0.0.0.0:14540") -> None:
        if self.dry_run:
            print("-- [DRY RUN] Skipping live connection")
            return

        print(f"-- Connecting to drone on {system_address} ...")
        await self.drone.connect(system_address=system_address)

        async for state in self.drone.core.connection_state():
            if state.is_connected:
                print("-- Connected")
                break

        print("-- Waiting for global position + home position lock ...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("-- Global position estimate OK")
                break

    async def _get_sim_home_position(self):
        """Returns (lat, lon, absolute_altitude_m) of the vehicle's actual
        current position in the simulator. In dry-run mode, uses the
        plan's own reference position as a stand-in -- good enough to
        audit the command sequence without a live sim."""
        if self.dry_run:
            return self.reference_lat, self.reference_lon, 0.0
        async for position in self.drone.telemetry.position():
            return position.latitude_deg, position.longitude_deg, position.absolute_altitude_m

    # -- the deterministic core -----------------------------------------

    async def execute(self, plan: MissionPlan) -> List[ExecutedCommand]:
        """Same `plan` -> same sequence of actions below, every time. The
        only branch point is `plan.mission_type`, which is itself part of
        the input -- there is no other source of nondeterminism here."""
        sim_home_lat, sim_home_lon, sim_home_alt = await self._get_sim_home_position()
        self._log("record_sim_home", lat=sim_home_lat, lon=sim_home_lon, alt_amsl=sim_home_alt)

        # Align PX4's own RTL behavior with the app-level safety ceiling.
        # RTL is PX4's native failsafe maneuver -- it does NOT go through
        # this executor's goto_location path, so validator.py's max_alt_m
        # has no effect on it unless we explicitly set PX4's own
        # RTL_RETURN_ALT parameter to match. Without this, RTL climbs to
        # whatever PX4's default is (30m) regardless of what the mission's
        # safety config says.
        if self.rtl_return_alt_m is not None:
            await self._do(
                "set_rtl_return_altitude",
                lambda: self.drone.param.set_param_float("RTL_RETURN_ALT", self.rtl_return_alt_m),
                alt_m=self.rtl_return_alt_m,
            )

        await self._do("arm", lambda: self.drone.action.arm())

        await self._do(
            "set_takeoff_altitude",
            lambda: self.drone.action.set_takeoff_altitude(self.takeoff_alt_m),
            alt_m=self.takeoff_alt_m,
        )

        await self._do("takeoff", lambda: self.drone.action.takeoff())
        await self._wait_until_altitude(self.takeoff_alt_m, label="takeoff")

        # repetitions applies to loops; a plain route is always flown once.
        loop_count = plan.repetitions if plan.mission_type == MissionType.LOOP else 1

        for rep in range(1, loop_count + 1):
            if plan.mission_type == MissionType.LOOP:
                print(f"-- Loop pass {rep}/{loop_count}")

            for i, wp in enumerate(plan.waypoints):
                # Re-home: waypoint's offset from the plan's reference
                # frame, re-applied onto the sim's actual home position.
                north_m, east_m = _offset_from_reference(
                    self.reference_lat, self.reference_lon, wp.lat, wp.lon
                )
                target_lat, target_lon = _apply_offset(sim_home_lat, sim_home_lon, north_m, east_m)
                target_alt = sim_home_alt + wp.alt_m  # AGL -> AMSL for MAVSDK

                await self._do(
                    "goto_location",
                    lambda lat=target_lat, lon=target_lon, alt=target_alt: (
                        self.drone.action.goto_location(lat, lon, alt, 0.0)
                    ),
                    rep=rep, waypoint_index=i,
                    lat=round(target_lat, 7), lon=round(target_lon, 7),
                    alt_amsl=round(target_alt, 2),
                )
                await self._wait_until_arrived(
                    target_lat, target_lon, target_alt, label=f"rep {rep} -> waypoint[{i}]"
                )

        await self._do("return_to_launch", lambda: self.drone.action.return_to_launch())
        await self._wait_until_landed(label="returning to launch")

        print("-- Mission execution complete."
              + (" [DRY RUN -- nothing was actually sent to a vehicle]" if self.dry_run else ""))
        return self.audit_log

    def audit_log_as_json(self) -> str:
        return json.dumps(
            [{"step": e.step, "action": e.action, "params": e.params} for e in self.audit_log],
            indent=2,
        )


# -- CLI entry point ------------------------------------------------------

def _default_safety_config() -> SafetyConfig:
    # Matches test_validator.py's SAFETY_CFG so the Day 2 fixtures
    # validate identically here.
    return SafetyConfig(
        home_lat=10.5270,
        home_lon=76.2140,
        max_geofence_radius_m=200.0,
        min_alt_m=3.0,
        max_alt_m=25.0,
        max_speed_mps=8.0,
        max_leg_distance_m=150.0,
        loop_closure_tolerance_m=15.0,
    )


async def run_mission_from_file(
    mission_path: pathlib.Path,
    safety_cfg: SafetyConfig,
    system_address: str = "udpin://0.0.0.0:14540",
    dry_run: bool = False,
) -> None:
    """Load a JSON mission file, validate it (Stage 1 + Stage 2), and only
    execute it if validation passes. This is deliberately the same
    validate-then-execute path Day 4's LLM output will go through -- the
    executor never sees anything that hasn't already cleared
    validate_mission_json()."""
    raw = json.loads(mission_path.read_text())
    raw.pop("_expect", None)
    raw.pop("_note", None)

    result = validate_mission_json(raw, safety_cfg)
    if not result.ok:
        print("-- Mission REJECTED, not executing. Errors:")
        for err in result.errors:
            print(f"   - {err}")
        sys.exit(1)

    plan = MissionPlan.model_validate(raw)
    print(f"-- Mission validated OK: {plan.mission_type.value}, "
          f"{len(plan.waypoints)} waypoint(s), {plan.repetitions} repetition(s)")

    drone = None if dry_run else System()
    executor = MissionExecutor(
        drone, safety_cfg.home_lat, safety_cfg.home_lon,
        rtl_return_alt_m=safety_cfg.max_alt_m,
        dry_run=dry_run,
    )
    await executor.connect_and_wait_ready(system_address)
    await executor.execute(plan)

    print("\n-- Audit log (this run's exact command sequence):")
    print(executor.audit_log_as_json())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Run a validated mission JSON against PX4 SITL.")
    parser.add_argument("mission_file", type=pathlib.Path,
                         help="Path to a mission JSON file, e.g. fixtures/valid_patrol_loop.json")
    parser.add_argument("--system-address", default="udpin://0.0.0.0:14540")
    parser.add_argument("--dry-run", action="store_true",
                         help="Print the command sequence without connecting to a live vehicle")
    args = parser.parse_args()

    asyncio.run(run_mission_from_file(
        args.mission_file, _default_safety_config(), args.system_address, args.dry_run
    ))
