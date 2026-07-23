"""
vision_interpreter.py

Challenge 3 -- natural language vision-follow instruction -> validated
VisionFollowPlan, using the same bounded-retry-with-feedback pattern as
llm_interpreter.py and squad_interpreter.py.

Same architectural boundary as the rest of this project: the LLM only
ever proposes WHAT to look for and WHERE to search -- target_class and
search_waypoints. It never computes follow-steering math (that's
vision_follow_controller.py, deterministic) and never decides camera
mechanics (that's vision_executor.py). This is what makes "the target
type should be configurable by the user" a property of the schema
(target_class is free text, validated only for non-emptiness) rather
than something hardcoded per target.
"""

import json
import math
import re
import sys
from dataclasses import dataclass
from typing import Optional

import requests

from validator import SafetyConfig
from vision_schema import VisionFollowPlan
from vision_validator import validate_vision_mission_json

OLLAMA_URL = "http://localhost:11434/api/chat"
DEFAULT_MODEL = "qwen2.5:7b-instruct"

_METERS_PER_DEG_LAT = 111320.0


def _build_vision_system_prompt(cfg: SafetyConfig) -> str:
    meters_per_deg_lon = _METERS_PER_DEG_LAT * math.cos(math.radians(cfg.home_lat))
    delta_lat_50m = 50.0 / _METERS_PER_DEG_LAT
    delta_lon_30m = 30.0 / meters_per_deg_lon
    example_lat = cfg.home_lat + delta_lat_50m
    example_lon = cfg.home_lon + delta_lon_30m
    max_delta_deg = cfg.max_geofence_radius_m / _METERS_PER_DEG_LAT

    return f"""You are a mission planner for a single quadcopter drone doing a \
search-and-follow task. Convert the operator's natural-language instruction \
into a single JSON object. Output ONLY the JSON object -- no explanation, no \
markdown fences, nothing else.

The JSON object must have exactly these fields:

{{
  "target_class": <string -- what to look for, e.g. "person", "car", "backpack">,
  "search_waypoints": [ {{"lat": <float>, "lon": <float>, "alt_m": <float>}}, ... ],
  "search_speed_mps": <float>,
  "follow_distance_m": <float>,
  "follow_altitude_m": <float>,
  "max_follow_duration_s": <float>,
  "detection_confidence_threshold": <float>
}}

"target_class" should be a short, common object category matching how the \
operator described it -- if they say "follow my dog", use "dog"; if they say \
"look for the red backpack", use "backpack" (colors/descriptors beyond the \
base category are usually not something a detector can filter on, so keep it \
to the object category).

"search_waypoints" is the route to patrol while looking for the target -- \
needs at least 2 waypoints.

The home / reference point is latitude {cfg.home_lat}, longitude {cfg.home_lon}.

IMPORTANT -- every "lat" and "lon" you output must be an ABSOLUTE coordinate \
close to this reference point. Do NOT output small numbers near zero.

Scale, at this location: 0.0001 degrees of latitude is about 11 metres; \
0.0001 degrees of longitude is about {meters_per_deg_lon / 10000:.1f} metres.

Worked example -- a waypoint 50 metres north and 30 metres east of home:
  lat = {cfg.home_lat} + {delta_lat_50m:.6f}  =  {example_lat:.6f}
  lon = {cfg.home_lon} + {delta_lon_30m:.6f}  =  {example_lon:.6f}

Hard rules:
- alt_m is altitude in metres ABOVE GROUND LEVEL, between 3 and 25.
- search_speed_mps must be between 0.5 and 8.
- follow_distance_m must be between 3 and 30 -- pick a sensible standoff for \
the target type (closer for a small object, farther for something you \
shouldn't startle or collide with).
- follow_altitude_m must be between 3 and 25.
- max_follow_duration_s must be between 10 and 300 -- how long to keep \
following before giving up and returning home, even if the target is still \
visible. Pick something reasonable (60-120) unless the operator specifies.
- detection_confidence_threshold must be between 0.3 and 0.99 -- default to \
0.5 unless the operator asks for stricter or looser detection.
- Keep every waypoint within about {max_delta_deg:.4f} degrees \
(~{cfg.max_geofence_radius_m:.0f} metres) of the reference point.
- If the instruction is ambiguous about a value, pick a reasonable default \
rather than omitting the field.
"""


@dataclass
class VisionInterpretResult:
    ok: bool
    plan: Optional[VisionFollowPlan]
    search_plan: Optional[object]  # MissionPlan, kept loosely typed to avoid a schema.py import cycle here
    raw_json: Optional[dict]
    errors: list
    attempts: int


def _extract_json(text: str) -> dict:
    text = text.strip()
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        pass
    match = re.search(r"\{.*\}", text, re.DOTALL)
    if match:
        return json.loads(match.group(0))
    raise ValueError(f"could not extract JSON from model output: {text[:200]!r}")


def _call_ollama(messages: list, model: str = DEFAULT_MODEL) -> str:
    resp = requests.post(
        OLLAMA_URL,
        json={"model": model, "messages": messages, "format": "json", "stream": False},
        timeout=120,
    )
    resp.raise_for_status()
    return resp.json()["message"]["content"]


def interpret_vision_prompt(
    user_prompt: str,
    safety_cfg: SafetyConfig,
    model: str = DEFAULT_MODEL,
    max_attempts: int = 2,
) -> VisionInterpretResult:
    system = _build_vision_system_prompt(safety_cfg)
    messages = [
        {"role": "system", "content": system},
        {"role": "user", "content": user_prompt},
    ]

    last_errors: list = []
    last_raw: Optional[dict] = None

    for attempt in range(1, max_attempts + 1):
        print(f"-- Vision attempt {attempt}/{max_attempts}: querying {model} ...")
        raw_text = _call_ollama(messages, model=model)

        try:
            raw = _extract_json(raw_text)
        except ValueError as e:
            last_errors = [f"model did not return valid JSON: {e}"]
            messages.append({"role": "assistant", "content": raw_text})
            messages.append({"role": "user", "content":
                "That was not valid JSON. Reply with ONLY the JSON object, no other text."})
            continue

        last_raw = raw
        print(f"-- Vision draft JSON:\n{json.dumps(raw, indent=2)}")

        result = validate_vision_mission_json(raw, safety_cfg)
        if result.ok:
            return VisionInterpretResult(
                ok=True, plan=result.plan, search_plan=result.search_plan,
                raw_json=raw, errors=[], attempts=attempt,
            )

        last_errors = result.errors
        print(f"-- Vision validation FAILED (attempt {attempt}): {result.errors}")

        if attempt < max_attempts:
            error_text = "\n".join(f"- {e}" for e in result.errors)
            messages.append({"role": "assistant", "content": raw_text})
            messages.append({"role": "user", "content":
                f"That mission failed validation with these errors:\n{error_text}\n\n"
                "Reply with a corrected JSON object that fixes all of these issues. "
                "Output ONLY the JSON object."})

    return VisionInterpretResult(
        ok=False, plan=None, search_plan=None, raw_json=last_raw, errors=last_errors, attempts=max_attempts,
    )


def _default_safety_config() -> SafetyConfig:
    return SafetyConfig(
        home_lat=10.5270, home_lon=76.2140,
        max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
        max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
    )


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print('Usage: python3 vision_interpreter.py "<natural language vision-follow instruction>"')
        sys.exit(1)

    result = interpret_vision_prompt(sys.argv[1], _default_safety_config())

    if not result.ok:
        print(f"\n-- FAILED after {result.attempts} attempt(s). Not executing. Errors:")
        for err in result.errors:
            print(f"   - {err}")
        sys.exit(1)

    print(f"\n-- SUCCESS after {result.attempts} attempt(s).")
    print(result.plan.model_dump_json(indent=2))
