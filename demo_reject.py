"""
demo_reject.py

A deterministic demonstration of Stage 2 (safety) validation refusing an
out-of-bounds mission -- independent of any LLM output, so this is 100%
reproducible for a demo recording. No model variability, no retry loop,
no drone connection: just proof that validate_mission_json() -- the exact
same function every prompt in the live pipeline goes through -- refuses
to pass through something unsafe.

This mission is deliberately STRUCTURALLY PERFECT (passes Stage 1 schema
checks cleanly) but ~5.5km from home, well beyond the configured 200m
geofence -- proving the point from Day 2: well-formed and safe are
different questions, answered by different code.

Run with:  python3 demo_reject.py
Safe to run any time, in any state -- makes no MAVSDK/drone connection.
"""

import json

from validator import SafetyConfig, validate_mission_json

cfg = SafetyConfig(
    home_lat=10.5270, home_lon=76.2140,
    max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
    max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
)

unsafe_mission = {
    "mission_type": "route",
    "waypoints": [
        {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
        {"lat": 10.5770, "lon": 76.2140, "alt_m": 10},  # ~5.5km north of home
    ],
    "repetitions": 1,
    "speed_mps": 5,
}

print("-- Mission JSON (schema-valid, but far outside the safety envelope) --")
print(json.dumps(unsafe_mission, indent=2))

result = validate_mission_json(unsafe_mission, cfg)

print()
if result.ok:
    print("-- UNEXPECTED: mission was accepted. This should not happen. --")
else:
    print("-- REJECTED. Nothing was sent to the drone. Errors: --")
    for err in result.errors:
        print(f"   - {err}")