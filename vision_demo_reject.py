"""
vision_demo_reject.py

Vision-follow counterpart to demo_reject.py / squad_demo_reject.py --
deterministic proof that vision_validator.py refuses unsafe vision
missions, independent of any LLM output.

Run with:  python3 vision_demo_reject.py
Safe to run any time -- makes no MAVSDK/camera/drone connection.
"""

import json

from validator import SafetyConfig
from vision_validator import validate_vision_mission_json

cfg = SafetyConfig(
    home_lat=10.5270, home_lon=76.2140,
    max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
    max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
)

empty_target = {
    "target_class": "",  # schema requires non-empty
    "search_waypoints": [
        {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
        {"lat": 10.5272, "lon": 76.2145, "alt_m": 10},
    ],
    "search_speed_mps": 4,
    "follow_distance_m": 8,
    "follow_altitude_m": 10,
    "max_follow_duration_s": 60,
}

search_route_outside_geofence = {
    "target_class": "person",
    "search_waypoints": [
        {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
        {"lat": 10.60, "lon": 76.30, "alt_m": 10},  # ~10km away, way outside 200m geofence
    ],
    "search_speed_mps": 4,
    "follow_distance_m": 8,
    "follow_altitude_m": 10,
    "max_follow_duration_s": 60,
}


def _run_case(title: str, mission: dict) -> None:
    print(f"\n== {title} ==")
    print(json.dumps(mission, indent=2))
    result = validate_vision_mission_json(mission, cfg)
    print()
    if result.ok:
        print("-- UNEXPECTED: mission was accepted. This should not happen. --")
    else:
        print("-- REJECTED. Nothing was sent to any drone. Errors: --")
        for err in result.errors:
            print(f"   - {err}")


if __name__ == "__main__":
    _run_case("Case 1: empty target_class", empty_target)
    _run_case("Case 2: search route outside the geofence", search_route_outside_geofence)
