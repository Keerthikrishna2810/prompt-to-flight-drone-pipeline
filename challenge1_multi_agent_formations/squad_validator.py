"""
squad_validator.py

Challenge 1 -- turns one SquadPlan into N per-drone MissionPlans, then
runs EVERY SINGLE ONE through the exact same validate_mission_json() /
validate_safety() two-stage pipeline the core task already built. No new
safety logic is invented for individual drones -- geofence, altitude,
speed, leg length, loop closure all apply per drone, unchanged.

The one genuinely new safety rule here is minimum separation between
drones -- meaningless for a single drone, essential for a squad. It's
kept in its own function, run only after every individual plan has
already passed, so a formation with a collision risk is rejected the
same way a single drone's out-of-bounds waypoint always has been:
outright refusal, nothing executed, no silent correction (see
WRITEUP.md's "Refuse completely, don't auto-correct" for why that
policy was chosen for the core task -- it applies here without change).
"""

from dataclasses import dataclass, field
from typing import List

from schema import MissionPlan
from validator import SafetyConfig, validate_mission_json, _haversine_m
from squad_schema import SquadPlan, SquadMode
from formation import apply_formation, split_route_across_drones


@dataclass
class SquadValidationResult:
    ok: bool
    plans: List[MissionPlan] = field(default_factory=list)  # one per drone, in drone-index order
    errors: List[str] = field(default_factory=list)          # drone-prefixed, e.g. "drone[1]: ..."


def _expand_to_per_drone_routes(squad: SquadPlan) -> List[list]:
    """Squad-level geometry only -- no validation happens here, on
    purpose, so this function's output can be unit-tested against
    formation.py alone (see test_formation.py) independently of
    validator behaviour."""
    if squad.mode == SquadMode.FORMATION:
        return apply_formation(squad.waypoints, squad.formation, squad.drone_count, squad.spacing_m)
    if squad.mode == SquadMode.SPLIT:
        return split_route_across_drones(squad.waypoints, squad.drone_count)
    raise ValueError(f"unknown squad mode: {squad.mode}")


def check_min_separation(plans: List[MissionPlan], min_separation_m: float) -> List[str]:
    """Compares every drone pair at every matching waypoint INDEX. This is
    only meaningful for SquadMode.FORMATION, where every drone's route has
    the same length and index i across all routes represents 'the same
    moment' in the shared shape (they all fly the same number of legs in
    lockstep). SquadMode.SPLIT routes intentionally don't share indices
    the same way -- lanes are meant to be in different places, that IS
    the safety property there, so this check is skipped for SPLIT plans
    by the caller (see validate_squad_mission_json)."""
    errors: List[str] = []
    if len(plans) < 2:
        return errors

    n_waypoints = len(plans[0].waypoints)
    for p in plans[1:]:
        if len(p.waypoints) != n_waypoints:
            errors.append(
                "cannot check separation: drone routes have different waypoint counts "
                f"({n_waypoints} vs {len(p.waypoints)}) -- formation expansion should "
                "never produce this, this indicates a bug upstream"
            )
            return errors

    for i in range(n_waypoints):
        for a in range(len(plans)):
            for b in range(a + 1, len(plans)):
                wp_a, wp_b = plans[a].waypoints[i], plans[b].waypoints[i]
                dist = _haversine_m(wp_a.lat, wp_a.lon, wp_b.lat, wp_b.lon)
                if dist < min_separation_m:
                    errors.append(
                        f"drone[{a}] and drone[{b}] are only {dist:.1f}m apart at "
                        f"waypoint[{i}], below minimum separation {min_separation_m}m"
                    )
    return errors


def validate_squad_mission_json(raw: dict, cfg: SafetyConfig, min_separation_m: float = 3.0) -> SquadValidationResult:
    """Full entry point, mirroring validator.py's validate_mission_json()
    shape so callers (squad_interpreter.py, squad_executor.py) can use it
    the same way the core task's callers use the single-drone version."""
    try:
        squad = SquadPlan.model_validate(raw)
    except Exception as e:
        return SquadValidationResult(ok=False, errors=[f"squad schema validation failed: {e}"])

    try:
        per_drone_routes = _expand_to_per_drone_routes(squad)
    except ValueError as e:
        return SquadValidationResult(ok=False, errors=[f"formation expansion failed: {e}"])

    plans: List[MissionPlan] = []
    errors: List[str] = []
    for i, route in enumerate(per_drone_routes):
        drone_plan_raw = {
            "mission_type": squad.mission_type.value,
            "waypoints": [wp.model_dump() for wp in route],
            "repetitions": squad.repetitions,
            "speed_mps": squad.speed_mps,
        }
        result = validate_mission_json(drone_plan_raw, cfg)
        if not result.ok:
            errors.extend(f"drone[{i}]: {e}" for e in result.errors)
        else:
            plans.append(MissionPlan.model_validate(drone_plan_raw))

    if errors:
        return SquadValidationResult(ok=False, errors=errors)

    if squad.mode == SquadMode.FORMATION:
        sep_errors = check_min_separation(plans, min_separation_m)
        if sep_errors:
            return SquadValidationResult(ok=False, errors=sep_errors)

    return SquadValidationResult(ok=True, plans=plans)
