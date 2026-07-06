"""
Safety / sanity validator — Day 2, Stage 2.

Runs only on a MissionPlan that has already passed Pydantic (Stage 1).
This layer encodes *operational* rules that don't belong in the schema
itself because they depend on runtime context (where home is, what
airspace is allowed today) rather than the shape of the data.

Design note for the write-up: Stage 1 answers "is this well-formed?".
Stage 2 answers "is this safe to actually fly, right now, from here?".
Keeping them separate means the geofence/limits can change per deployment
without touching the schema contract the LLM is prompted against.
"""

import math
from dataclasses import dataclass, field
from typing import List

from schema import MissionPlan


@dataclass
class SafetyConfig:
    home_lat: float
    home_lon: float
    max_geofence_radius_m: float = 200.0
    min_alt_m: float = 3.0
    max_alt_m: float = 25.0          # tighter than the schema's 30m ceiling
    max_speed_mps: float = 8.0       # tighter than the schema's 15 m/s ceiling
    max_leg_distance_m: float = 150.0
    loop_closure_tolerance_m: float = 15.0


@dataclass
class ValidationResult:
    ok: bool
    errors: List[str] = field(default_factory=list)


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres between two lat/lon points."""
    r = 6371000.0
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlambda / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def validate_safety(plan: MissionPlan, cfg: SafetyConfig) -> ValidationResult:
    errors: List[str] = []

    for i, wp in enumerate(plan.waypoints):
        dist_from_home = _haversine_m(cfg.home_lat, cfg.home_lon, wp.lat, wp.lon)
        if dist_from_home > cfg.max_geofence_radius_m:
            errors.append(
                f"waypoint[{i}] is {dist_from_home:.1f}m from home, "
                f"exceeds geofence radius {cfg.max_geofence_radius_m}m"
            )
        if not (cfg.min_alt_m <= wp.alt_m <= cfg.max_alt_m):
            errors.append(
                f"waypoint[{i}] altitude {wp.alt_m}m outside configured "
                f"safe range [{cfg.min_alt_m}, {cfg.max_alt_m}]"
            )

    if plan.speed_mps > cfg.max_speed_mps:
        errors.append(
            f"speed_mps {plan.speed_mps} exceeds configured max {cfg.max_speed_mps}"
        )

    for i in range(len(plan.waypoints) - 1):
        a, b = plan.waypoints[i], plan.waypoints[i + 1]
        leg = _haversine_m(a.lat, a.lon, b.lat, b.lon)
        if leg > cfg.max_leg_distance_m:
            errors.append(
                f"leg waypoint[{i}]->waypoint[{i+1}] is {leg:.1f}m, "
                f"exceeds max_leg_distance_m {cfg.max_leg_distance_m}"
            )

    if plan.mission_type.value == "loop":
        first, last = plan.waypoints[0], plan.waypoints[-1]
        closure = _haversine_m(first.lat, first.lon, last.lat, last.lon)
        if closure > cfg.loop_closure_tolerance_m:
            errors.append(
                f"mission_type 'loop' but first/last waypoints are {closure:.1f}m "
                f"apart, exceeds closure tolerance {cfg.loop_closure_tolerance_m}m"
            )

    return ValidationResult(ok=(len(errors) == 0), errors=errors)


def validate_mission_json(raw: dict, cfg: SafetyConfig) -> ValidationResult:
    """Full two-stage entry point: Stage 1 (schema) then Stage 2 (safety)."""
    try:
        plan = MissionPlan.model_validate(raw)
    except Exception as e:
        return ValidationResult(ok=False, errors=[f"schema validation failed: {e}"])
    return validate_safety(plan, cfg)
