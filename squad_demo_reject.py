"""
squad_demo_reject.py

Squad-level counterpart to demo_reject.py -- deterministic proof that
squad_validator.py refuses an unsafe SQUAD mission, independent of any
LLM output. Two separate, structurally-valid-looking squad missions are
run through validate_squad_mission_json(), each hitting a different new
rule this project's squad layer adds on top of the unchanged single-drone
checks:

  1. Too many drones for the squad schema (drone_count above the cap).
  2. Individually-safe drones that are too close TO EACH OTHER once
     formation.py expands them -- the new rule that has no single-drone
     equivalent at all.

Run with:  python3 squad_demo_reject.py
Safe to run any time, in any state -- makes no MAVSDK/drone connection.
"""

import json

from validator import SafetyConfig
from squad_validator import validate_squad_mission_json

cfg = SafetyConfig(
    home_lat=10.5270, home_lon=76.2140,
    max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
    max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
)

too_many_drones = {
    "drone_count": 9,  # SquadPlan caps at 6
    "mode": "formation",
    "formation": "box",
    "spacing_m": 10.0,
    "mission_type": "route",
    "repetitions": 1,
    "speed_mps": 5,
    "waypoints": [
        {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
        {"lat": 10.5272, "lon": 76.2145, "alt_m": 10},
    ],
}

# Every individual drone here is well within the geofence and every other
# single-drone rule -- this is rejected ONLY by the squad-level minimum
# separation check, run with a stricter-than-default 5m minimum (an
# operator asking for a tighter formation safety margin than the schema's
# bare 3m spacing floor guarantees on its own).
too_close_formation = {
    "drone_count": 2,
    "mode": "formation",
    "formation": "line",
    "spacing_m": 3.0,  # schema floor
    "mission_type": "route",
    "repetitions": 1,
    "speed_mps": 5,
    "waypoints": [
        {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
        {"lat": 10.5272, "lon": 76.2145, "alt_m": 10},
    ],
}


def _run_case(title: str, mission: dict, min_separation_m: float = 3.0) -> None:
    print(f"\n== {title} ==")
    print(json.dumps(mission, indent=2))
    result = validate_squad_mission_json(mission, cfg, min_separation_m=min_separation_m)
    print()
    if result.ok:
        print("-- UNEXPECTED: squad mission was accepted. This should not happen. --")
    else:
        print("-- REJECTED. Nothing was sent to any drone. Errors: --")
        for err in result.errors:
            print(f"   - {err}")


if __name__ == "__main__":
    _run_case("Case 1: drone_count above squad schema cap", too_many_drones)
    _run_case(
        "Case 2: every drone individually safe, but too close to each other",
        too_close_formation, min_separation_m=5.0,
    )
