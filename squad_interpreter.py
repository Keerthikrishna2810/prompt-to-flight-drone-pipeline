"""
squad_interpreter.py

Challenge 1 -- natural language squad instruction -> validated per-drone
MissionPlans, using the exact same pattern llm_interpreter.py already
proved out for a single drone:

    prompt -> system prompt (SquadPlan shape + constraints)
        -> Ollama /api/chat, format="json"
        -> raw JSON draft (untrusted)
        -> validate_squad_mission_json() [squad schema + per-drone
           schema/safety + separation]
        -> if invalid: retry ONCE with the errors fed back
        -> if valid: N MissionPlans, ready for squad_executor.py

Same architectural boundary as the core task: the LLM only ever proposes
SQUAD-LEVEL intent (how many drones, what formation, how far apart, the
shared route) -- it never computes an individual drone's waypoints. The
per-drone geometry is entirely formation.py's deterministic math, run
inside validate_squad_mission_json(). This is a stronger version of the
same "LLM proposes, code disposes" principle the core task's write-up
argues for: here the LLM cannot even attempt the fiddly multi-drone
offset arithmetic, so there's no geometry mistake for it to make in the
first place.
"""

import json
import math
import re
import sys
from dataclasses import dataclass
from typing import List, Optional

import requests

from schema import MissionPlan
from validator import SafetyConfig
from squad_validator import validate_squad_mission_json

OLLAMA_URL = "http://localhost:11434/api/chat"
DEFAULT_MODEL = "qwen2.5:7b-instruct"

_METERS_PER_DEG_LAT = 111320.0


def _build_squad_system_prompt(cfg: SafetyConfig) -> str:
    """Deliberately reuses llm_interpreter.py's worked-example approach
    (absolute lat/lon computed for the actual configured reference point,
    not an abstract local origin) -- that's what fixed real drift-to-zero
    hallucinations in the single-drone prompt, so the same fix is applied
    here rather than re-discovering it."""
    meters_per_deg_lon = _METERS_PER_DEG_LAT * math.cos(math.radians(cfg.home_lat))
    delta_lat_50m = 50.0 / _METERS_PER_DEG_LAT
    delta_lon_30m = 30.0 / meters_per_deg_lon
    example_lat = cfg.home_lat + delta_lat_50m
    example_lon = cfg.home_lon + delta_lon_30m
    max_delta_deg = cfg.max_geofence_radius_m / _METERS_PER_DEG_LAT

    return f"""You are a mission planner for a SQUAD of 2 to 6 quadcopter drones. \
Convert the operator's natural-language instruction into a single JSON object \
describing the squad's mission. Output ONLY the JSON object -- no explanation, \
no markdown fences, nothing else.

The JSON object must have exactly these fields:

{{
  "drone_count": <int, 2 to 6>,
  "mode": "formation" | "split",
  "formation": "line" | "wedge" | "column" | "box",
  "spacing_m": <float, 3 to 50>,
  "mission_type": "loop" | "route",
  "waypoints": [ {{"lat": <float>, "lon": <float>, "alt_m": <float>}}, ... ],
  "repetitions": <int>,
  "speed_mps": <float>
}}

Use mode "formation" when the operator wants the squad to fly TOGETHER as a \
shape (e.g. "in a wedge", "line abreast", "one behind the other") -- every \
drone flies the SAME route, offset into that shape. The "formation" field is \
only used in this mode; still include it either way with a sensible default.

Use mode "split" when the operator wants the squad to DIVIDE UP a route or \
area between drones (e.g. "split this into lanes", "each drone take a third \
of the area", "sweep this zone with 3 drones"). In this mode "waypoints" is \
the FULL route/perimeter to be divided -- it will be split into drone_count \
contiguous lanes automatically, you do not need to divide it yourself.

The home / reference point is latitude {cfg.home_lat}, longitude {cfg.home_lon}.

IMPORTANT -- every "lat" and "lon" you output must be an ABSOLUTE coordinate \
close to this reference point. Do NOT output small numbers near zero -- those \
would place the squad thousands of kilometres away and will always be \
rejected.

Scale, at this location: 0.0001 degrees of latitude is about 11 metres; \
0.0001 degrees of longitude is about {meters_per_deg_lon / 10000:.1f} metres.

Worked example -- a waypoint 50 metres north and 30 metres east of home:
  lat = {cfg.home_lat} + {delta_lat_50m:.6f}  =  {example_lat:.6f}
  lon = {cfg.home_lon} + {delta_lon_30m:.6f}  =  {example_lon:.6f}
Always ADD small deltas like this to the reference point above -- never write \
coordinates near (0, 0).

Hard rules:
- "waypoints" describes ONE shared route (the formation's shape/path, or the \
full area perimeter to split) -- never write separate routes per drone, that \
is computed automatically from this one list.
- "loop" missions need at least 3 DISTINCT corner waypoints, PLUS one more \
waypoint at the end that repeats the very first waypoint's lat/lon/alt \
exactly, to close the shape.
- "route" missions are flown once, start to end -- no closure needed.
- In "split" mode, the route needs at least drone_count + 1 waypoints so it \
can actually be divided into that many lanes.
- alt_m is altitude in metres ABOVE GROUND LEVEL, between 3 and 25.
- speed_mps must be between 0.5 and 8.
- spacing_m is the intended gap between drones -- pick at least 8 unless the \
operator asks for tighter formation flying.
- Keep every waypoint within about {max_delta_deg:.4f} degrees \
(~{cfg.max_geofence_radius_m:.0f} metres) of the reference point.
- If the instruction is ambiguous about a value, pick a reasonable default \
rather than omitting the field.
"""


@dataclass
class SquadInterpretResult:
    ok: bool
    plans: Optional[List[MissionPlan]]  # one per drone, drone-index order
    raw_json: Optional[dict]
    errors: List[str]
    attempts: int


def _extract_json(text: str) -> dict:
    """Identical defensive fallback to llm_interpreter.py's version --
    duplicated rather than imported, same reasoning as formation.py's
    duplicated offset math: this file's only real contract is the JSON
    shape, not llm_interpreter internals."""
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
    """Isolated exactly like llm_interpreter.py's _call_ollama, so tests
    can mock this one call and exercise the retry/validation logic with
    no live model -- see test_squad_interpreter.py."""
    resp = requests.post(
        OLLAMA_URL,
        json={"model": model, "messages": messages, "format": "json", "stream": False},
        timeout=120,
    )
    resp.raise_for_status()
    return resp.json()["message"]["content"]


def interpret_squad_prompt(
    user_prompt: str,
    safety_cfg: SafetyConfig,
    model: str = DEFAULT_MODEL,
    max_attempts: int = 2,
    min_separation_m: float = 3.0,
) -> SquadInterpretResult:
    """Squad-level counterpart to llm_interpreter.interpret_prompt() --
    same bounded-retry-with-feedback shape, never raises on a bad draft."""

    system = _build_squad_system_prompt(safety_cfg)
    messages = [
        {"role": "system", "content": system},
        {"role": "user", "content": user_prompt},
    ]

    last_errors: List[str] = []
    last_raw: Optional[dict] = None

    for attempt in range(1, max_attempts + 1):
        print(f"-- Squad attempt {attempt}/{max_attempts}: querying {model} ...")
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
        print(f"-- Squad draft JSON:\n{json.dumps(raw, indent=2)}")

        result = validate_squad_mission_json(raw, safety_cfg, min_separation_m)
        if result.ok:
            return SquadInterpretResult(ok=True, plans=result.plans, raw_json=raw, errors=[], attempts=attempt)

        last_errors = result.errors
        print(f"-- Squad validation FAILED (attempt {attempt}): {result.errors}")

        if attempt < max_attempts:
            error_text = "\n".join(f"- {e}" for e in result.errors)
            messages.append({"role": "assistant", "content": raw_text})
            messages.append({"role": "user", "content":
                f"That squad mission failed validation with these errors:\n{error_text}\n\n"
                "Reply with a corrected JSON object that fixes all of these issues. "
                "Output ONLY the JSON object."})

    return SquadInterpretResult(ok=False, plans=None, raw_json=last_raw, errors=last_errors, attempts=max_attempts)


def _default_safety_config() -> SafetyConfig:
    return SafetyConfig(
        home_lat=10.5270, home_lon=76.2140,
        max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
        max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
    )


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print('Usage: python3 squad_interpreter.py "<natural language squad instruction>"')
        sys.exit(1)

    result = interpret_squad_prompt(sys.argv[1], _default_safety_config())

    if not result.ok:
        print(f"\n-- FAILED after {result.attempts} attempt(s). Not executing. Errors:")
        for err in result.errors:
            print(f"   - {err}")
        sys.exit(1)

    print(f"\n-- SUCCESS after {result.attempts} attempt(s). {len(result.plans)} per-drone plan(s):")
    for i, plan in enumerate(result.plans):
        print(f"\n-- drone[{i}] --")
        print(plan.model_dump_json(indent=2))
