"""
vision_validator.py

Challenge 3 -- validates a VisionFollowPlan the same way
squad_validator.py validates a SquadPlan: the search route is converted
to an ordinary MissionPlan and run through validator.py's
validate_mission_json() completely unchanged -- geofence, altitude,
speed, loop closure, all identical to a normal single-drone mission.

Field-level bounds on the vision-specific parameters (target_class
non-empty, follow_distance_m/follow_altitude_m ranges, confidence
threshold range) are enforced by VisionFollowPlan's own pydantic
constraints in vision_schema.py -- this file's job is just wiring that
model to the existing per-drone safety pipeline, plus one extra check:
follow_altitude_m must itself sit inside the configured altitude bounds
(pydantic's static 3-25 range is a sane global default, but the actual
per-deployment SafetyConfig might be tighter).
"""

from dataclasses import dataclass, field
from typing import List

from schema import MissionPlan, MissionType
from validator import SafetyConfig, validate_mission_json
from vision_schema import VisionFollowPlan


@dataclass
class VisionValidationResult:
    ok: bool
    search_plan: MissionPlan | None = None
    plan: VisionFollowPlan | None = None
    errors: List[str] = field(default_factory=list)


def validate_vision_mission_json(raw: dict, cfg: SafetyConfig) -> VisionValidationResult:
    try:
        vision_plan = VisionFollowPlan.model_validate(raw)
    except Exception as e:
        return VisionValidationResult(ok=False, errors=[f"vision schema validation failed: {e}"])

    errors: List[str] = []

    if not (cfg.min_alt_m <= vision_plan.follow_altitude_m <= cfg.max_alt_m):
        errors.append(
            f"follow_altitude_m {vision_plan.follow_altitude_m} is outside the configured "
            f"altitude bounds [{cfg.min_alt_m}, {cfg.max_alt_m}]"
        )

    search_route_raw = {
        "mission_type": MissionType.ROUTE.value,
        "waypoints": [wp.model_dump() for wp in vision_plan.search_waypoints],
        "repetitions": 1,
        "speed_mps": vision_plan.search_speed_mps,
    }
    search_result = validate_mission_json(search_route_raw, cfg)
    if not search_result.ok:
        errors.extend(f"search route: {e}" for e in search_result.errors)

    if errors:
        return VisionValidationResult(ok=False, errors=errors)

    return VisionValidationResult(
        ok=True,
        search_plan=MissionPlan.model_validate(search_route_raw),
        plan=vision_plan,
    )
