"""
vision_executor.py

Challenge 3 -- flies a search route, polling a camera after each leg;
on a qualifying detection, saves a snapshot (the "send a picture to the
operator" requirement) and switches into a closed-loop follow behavior
using vision_follow_controller.py's pure geometry.

Duplicates mission_executor.py's small connect/arm/takeoff/coordinate-
offset helpers rather than importing its internals -- same "duplicate a
few small functions, keep the module boundary clean" choice this project
already makes for validator.py's haversine (in mission_executor.py) and
mission_executor.py's offset math (in formation.py). mission_executor's
execute() has no hook for injecting camera polling mid-flight, and this
is a genuinely different flight pattern (search-then-react, not a fixed
fly-through) -- not worth editing the one file this project treats as
"boring on purpose, never touched" for.

--------------------------------------------------------------------------
Bounded by design, not by trust in a clock
--------------------------------------------------------------------------
The follow loop is a `for` loop over a fixed, precomputed iteration
count -- not a `while True` guarded only by a timeout check. This is a
direct lesson from this project's own multi-agent formation work: a
timing assumption that quietly doesn't hold (a paused sim clock, a stuck
telemetry stream) turns "loop until timeout" into "loop forever". Here,
even if every timing assumption inside the loop is wrong, the loop
still physically cannot execute more than max_iterations times.

--------------------------------------------------------------------------
Known simplification: camera-forward assumed to face geographic north
--------------------------------------------------------------------------
compute_follow_offset() returns a (forward, right) steering command
relative to the CAMERA's current facing direction. Converting that into
a real-world (north, east) offset correctly requires the vehicle's
current yaw/heading. This executor does not track yaw -- it applies the
steering command as if forward==north, right==east, unconditionally.
That's a real, named limitation (not a hidden bug): the follow behavior
will steer toward the target's LEFT/RIGHT and NEAR/FAR correctly only if
the vehicle happens to be facing geographic north. Tracking yaw via
`telemetry.attitude_euler()` and rotating the offset accordingly (the
same rotation formation.py already does for a route's bearing) is the
direct fix, noted as follow-up work rather than solved here.
"""

import asyncio
import math
import pathlib
import time
from dataclasses import dataclass, field
from typing import Callable, List, Optional, Protocol

from mavsdk import System

from schema import MissionPlan
from vision_schema import VisionFollowPlan
from vision_detector import TargetDetector
from vision_follow_controller import compute_follow_offset

_EARTH_RADIUS_M = 6371000.0


def _offset_from_reference(ref_lat: float, ref_lon: float, lat: float, lon: float):
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)
    north_m = d_lat * _EARTH_RADIUS_M
    east_m = d_lon * _EARTH_RADIUS_M * math.cos(math.radians(ref_lat))
    return north_m, east_m


def _apply_offset(base_lat: float, base_lon: float, north_m: float, east_m: float):
    d_lat = north_m / _EARTH_RADIUS_M
    d_lon = east_m / (_EARTH_RADIUS_M * math.cos(math.radians(base_lat)))
    return base_lat + math.degrees(d_lat), base_lon + math.degrees(d_lon)


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    r = _EARTH_RADIUS_M
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlambda / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


class CameraSource(Protocol):
    def get_frame(self): ...


class NullCameraSource:
    """Dry-run / test stand-in -- returns None forever. Pair with a
    MockDetector whose scripted output doesn't depend on frame content."""

    def get_frame(self):
        return None


@dataclass
class ExecutedCommand:
    step: int
    action: str
    params: dict


@dataclass
class VisionExecutionResult:
    ok: bool
    target_found: bool
    snapshot_path: Optional[str] = None
    audit_log: List[ExecutedCommand] = field(default_factory=list)
    errors: List[str] = field(default_factory=list)


class VisionFollowExecutor:
    def __init__(
        self,
        drone: Optional[System],
        reference_lat: float,
        reference_lon: float,
        camera: CameraSource,
        detector: TargetDetector,
        snapshot_dir: pathlib.Path,
        max_geofence_radius_m: Optional[float] = None,
        takeoff_alt_m: float = 10.0,
        rtl_return_alt_m: Optional[float] = None,
        follow_poll_interval_s: float = 2.0,
        dry_run: bool = False,
    ):
        self.drone = drone
        self.reference_lat = reference_lat
        self.reference_lon = reference_lon
        self.camera = camera
        self.detector = detector
        self.snapshot_dir = snapshot_dir
        self.max_geofence_radius_m = max_geofence_radius_m
        self.takeoff_alt_m = takeoff_alt_m
        self.rtl_return_alt_m = rtl_return_alt_m
        self.follow_poll_interval_s = follow_poll_interval_s
        self.dry_run = dry_run
        self.audit_log: List[ExecutedCommand] = []
        self._step = 0

    # -- bookkeeping, same pattern as MissionExecutor -------------------

    def _log(self, action: str, **params) -> None:
        self._step += 1
        entry = ExecutedCommand(step=self._step, action=action, params=params)
        self.audit_log.append(entry)
        print(f"   [{entry.step:02d}] {action} {params}")

    async def _do(self, action: str, call: Optional[Callable] = None, **params) -> None:
        self._log(action, **params)
        if not self.dry_run and call is not None:
            await call()

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
        if self.dry_run:
            return self.reference_lat, self.reference_lon, 0.0
        async for position in self.drone.telemetry.position():
            return position.latitude_deg, position.longitude_deg, position.absolute_altitude_m

    async def _wait_until_arrived(self, target_lat, target_lon, target_alt_amsl, label,
                                   tolerance_m=3.0, timeout_s=25.0, poll_interval_s=2.0) -> None:
        if self.dry_run:
            return
        print(f"-- Waiting for arrival: {label} (tolerance {tolerance_m}m, timeout {timeout_s}s)")
        elapsed = 0.0
        async for position in self.drone.telemetry.position():
            dist = _haversine_m(position.latitude_deg, position.longitude_deg, target_lat, target_lon)
            if dist <= tolerance_m:
                print(f"   -- Arrived within tolerance at t+{elapsed:.1f}s")
                return
            elapsed += poll_interval_s
            if elapsed >= timeout_s:
                print(f"   -- WARNING: did not reach tolerance within {timeout_s}s (last dist={dist:.1f}m), "
                      f"proceeding anyway")
                return
            await asyncio.sleep(poll_interval_s)

    async def _wait_until_landed(self, timeout_s=40.0, poll_interval_s=2.0) -> None:
        if self.dry_run:
            return
        elapsed = 0.0
        async for in_air in self.drone.telemetry.in_air():
            if not in_air:
                return
            elapsed += poll_interval_s
            if elapsed >= timeout_s:
                print(f"   -- WARNING: still airborne after {timeout_s}s, proceeding anyway")
                return
            await asyncio.sleep(poll_interval_s)

    # -- operator notification -------------------------------------------

    def _save_snapshot(self, frame, target_class: str) -> str:
        """'Sends a picture to the human operator' -- MVP implementation:
        saves the frame to a watched snapshot directory with a clear
        filename, and prints an operator-facing notification. A real
        deployment would swap this one function for an actual push
        (email, Slack webhook, MQTT) -- everything upstream (detection,
        follow control) is unaffected by how the picture actually
        reaches the operator, that's the whole point of isolating it
        here."""
        self.snapshot_dir.mkdir(parents=True, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        path = self.snapshot_dir / f"{timestamp}_{target_class}.jpg"
        if frame is not None:
            try:
                from PIL import Image
                Image.fromarray(frame).save(path)
            except Exception as e:
                print(f"-- WARNING: could not encode snapshot ({e}), writing placeholder file")
                path.write_bytes(b"")
        else:
            path.write_bytes(b"")  # dry-run / no-camera stand-in -- path still reported correctly
        print(f"-- TARGET ACQUIRED: '{target_class}' -- snapshot saved to {path} -- notify operator")
        return str(path)

    # -- the deterministic core -------------------------------------------

    async def execute(self, vision_plan: VisionFollowPlan, search_plan: MissionPlan) -> VisionExecutionResult:
        sim_home_lat, sim_home_lon, sim_home_alt = await self._get_sim_home_position()
        self._log("record_sim_home", lat=sim_home_lat, lon=sim_home_lon, alt_amsl=sim_home_alt)

        if self.rtl_return_alt_m is not None:
            await self._do(
                "set_rtl_return_altitude",
                lambda: self.drone.param.set_param_float("RTL_RETURN_ALT", self.rtl_return_alt_m),
                alt_m=self.rtl_return_alt_m,
            )

        await self._do("arm", lambda: self.drone.action.arm())
        await self._do("set_takeoff_altitude",
                        lambda: self.drone.action.set_takeoff_altitude(self.takeoff_alt_m),
                        alt_m=self.takeoff_alt_m)
        await self._do("takeoff", lambda: self.drone.action.takeoff())

        target_found = False
        snapshot_path = None
        current_lat, current_lon = sim_home_lat, sim_home_lon

        # -- search phase: fly the route, poll the camera after each leg --
        for i, wp in enumerate(search_plan.waypoints):
            north_m, east_m = _offset_from_reference(self.reference_lat, self.reference_lon, wp.lat, wp.lon)
            target_lat, target_lon = _apply_offset(sim_home_lat, sim_home_lon, north_m, east_m)
            target_alt = sim_home_alt + wp.alt_m

            await self._do(
                "goto_location",
                lambda lat=target_lat, lon=target_lon, alt=target_alt: (
                    self.drone.action.goto_location(lat, lon, alt, 0.0)
                ),
                waypoint_index=i, lat=round(target_lat, 7), lon=round(target_lon, 7), alt_amsl=round(target_alt, 2),
            )
            await self._wait_until_arrived(target_lat, target_lon, target_alt, label=f"search waypoint[{i}]")
            current_lat, current_lon = target_lat, target_lon

            frame = self.camera.get_frame()
            detections = self.detector.detect(frame)
            match = next(
                (d for d in detections
                 if d.class_name == vision_plan.target_class
                 and d.confidence >= vision_plan.detection_confidence_threshold),
                None,
            )
            if match is not None:
                target_found = True
                snapshot_path = self._save_snapshot(frame, vision_plan.target_class)
                self._log("target_detected", waypoint_index=i, confidence=round(match.confidence, 3))
                break

        # -- no target found: RTL and stop, same as any other completed mission --
        if not target_found:
            self._log("search_complete_no_target", target_class=vision_plan.target_class)
            await self._do("return_to_launch", lambda: self.drone.action.return_to_launch())
            await self._wait_until_landed()
            print("-- Vision mission complete: target not found."
                  + (" [DRY RUN]" if self.dry_run else ""))
            return VisionExecutionResult(ok=True, target_found=False, audit_log=self.audit_log)

        # -- follow phase: bounded for-loop, never an unbounded while-True --
        max_iterations = max(1, int(vision_plan.max_follow_duration_s / self.follow_poll_interval_s))
        follow_stop_reason = "max_follow_duration_reached"

        for iteration in range(max_iterations):
            if self.max_geofence_radius_m is not None:
                dist_from_home = _haversine_m(current_lat, current_lon, sim_home_lat, sim_home_lon)
                if dist_from_home > self.max_geofence_radius_m:
                    follow_stop_reason = "geofence_exceeded"
                    break

            frame = self.camera.get_frame()
            detections = self.detector.detect(frame)
            match = next(
                (d for d in detections
                 if d.class_name == vision_plan.target_class
                 and d.confidence >= vision_plan.detection_confidence_threshold),
                None,
            )
            if match is None:
                follow_stop_reason = "target_lost"
                break

            cmd = compute_follow_offset(match, vision_plan.follow_distance_m)
            # NOTE: forward/right applied directly as north/east -- see
            # module docstring's "known simplification" section.
            new_lat, new_lon = _apply_offset(current_lat, current_lon, cmd.forward_m, cmd.right_m)
            target_alt = sim_home_alt + vision_plan.follow_altitude_m

            await self._do(
                "follow_goto",
                lambda lat=new_lat, lon=new_lon, alt=target_alt: (
                    self.drone.action.goto_location(lat, lon, alt, 0.0)
                ),
                iteration=iteration, lat=round(new_lat, 7), lon=round(new_lon, 7),
                alt_amsl=round(target_alt, 2), reached_standoff=cmd.reached_standoff,
            )
            current_lat, current_lon = new_lat, new_lon

            if not self.dry_run:
                await asyncio.sleep(self.follow_poll_interval_s)

        self._log("follow_ended", reason=follow_stop_reason)
        await self._do("return_to_launch", lambda: self.drone.action.return_to_launch())
        await self._wait_until_landed()

        print("-- Vision mission complete: target followed then RTL."
              + (" [DRY RUN]" if self.dry_run else ""))
        return VisionExecutionResult(
            ok=True, target_found=True, snapshot_path=snapshot_path, audit_log=self.audit_log,
        )
