"""
llm_interpreter.py

Day 4 -- the LLM stage: natural-language prompt -> JSON mission draft,
using a locally-hosted Ollama model.

This file's ONLY job is producing a JSON draft. It never imports MAVSDK,
never talks to PX4/Gazebo, and never decides whether a mission is safe to
fly -- that's still entirely validator.py's job, unchanged since Day 2.
The whole project is built around this boundary: the LLM proposes, the
validator disposes, and mission_executor.py (Day 3) only ever sees output
that has already passed both validation stages.

Flow:
    natural language prompt
        -> system prompt (schema description + constraints, in plain English)
        -> Ollama /api/chat, format="json" (constrained decoding)
        -> raw JSON draft (untrusted)
        -> validate_mission_json() [Stage 1 schema + Stage 2 safety]
        -> if invalid: retry ONCE, feeding the validation errors back into
           the conversation so the model can self-correct
        -> if still invalid after the retry: reject, do not execute,
           surface the errors
        -> if valid: MissionPlan, ready for mission_executor.py
"""

import json
import math
import re
import sys
from dataclasses import dataclass
from typing import List, Optional

import requests

from schema import MissionPlan
from validator import SafetyConfig, validate_mission_json


OLLAMA_URL = "http://localhost:11434/api/chat"
DEFAULT_MODEL = "qwen2.5:7b-instruct"

_METERS_PER_DEG_LAT = 111320.0


def _build_system_prompt(cfg: SafetyConfig) -> str:
    """Builds the system prompt with a worked, numerically-correct example
    computed for the actual configured reference point -- rather than
    telling the model to think in an abstract local (0,0) frame, which
    empirically caused it to output waypoints literally near (0,0) on
    Earth (many thousands of km from home). Schema/validator expect
    absolute lat/lon, so the prompt has to ask for exactly that."""
    meters_per_deg_lon = _METERS_PER_DEG_LAT * math.cos(math.radians(cfg.home_lat))
    delta_lat_50m = 50.0 / _METERS_PER_DEG_LAT
    delta_lon_30m = 30.0 / meters_per_deg_lon
    example_lat = cfg.home_lat + delta_lat_50m
    example_lon = cfg.home_lon + delta_lon_30m
    max_delta_deg = cfg.max_geofence_radius_m / _METERS_PER_DEG_LAT

    return f"""You are a mission planner for a quadcopter drone. Convert the \
operator's natural-language instruction into a single JSON object describing a \
flight mission. Output ONLY the JSON object -- no explanation, no markdown \
fences, nothing else.

The JSON object must have exactly these fields:

{{
  "mission_type": "loop" | "route",
  "waypoints": [ {{"lat": <float>, "lon": <float>, "alt_m": <float>}}, ... ],
  "repetitions": <int>,
  "speed_mps": <float>
}}

The home / reference point is latitude {cfg.home_lat}, longitude {cfg.home_lon}.

IMPORTANT -- every "lat" and "lon" you output must be an ABSOLUTE coordinate \
close to this reference point. Do NOT output small numbers near zero (like \
0.05 or -0.03) -- those would place the drone thousands of kilometres away \
and will always be rejected.

Scale, at this location: 0.0001 degrees of latitude is about 11 metres; \
0.0001 degrees of longitude is about {meters_per_deg_lon / 10000:.1f} metres.

Worked example -- a waypoint 50 metres north and 30 metres east of home:
  lat = {cfg.home_lat} + {delta_lat_50m:.6f}  =  {example_lat:.6f}
  lon = {cfg.home_lon} + {delta_lon_30m:.6f}  =  {example_lon:.6f}
Always ADD small deltas like this to the reference point above -- never write \
coordinates near (0, 0).

Hard rules:
- "loop" missions need at least 3 DISTINCT corner waypoints, PLUS one more \
waypoint at the end that repeats the very first waypoint's lat/lon/alt \
exactly -- this is what closes the shape. In other words: a loop with N \
distinct corners needs N+1 waypoints listed (the N corners, in order, then \
the first corner again). For example, a 4-corner square patrol needs 5 \
waypoints total: corner1, corner2, corner3, corner4, corner1 (repeated).
- Do NOT stop at just the distinct corners -- always add that final \
repeated waypoint, or the path will not close and will be rejected.
- "route" missions are flown once, start to end -- no closure needed, do \
NOT repeat the first waypoint at the end.
- alt_m is altitude in metres ABOVE GROUND LEVEL at the launch point, between \
3 and 25.
- speed_mps must be between 0.5 and 8.
- Keep every waypoint within about {max_delta_deg:.4f} degrees \
(~{cfg.max_geofence_radius_m:.0f} metres) of the reference point, in both lat \
and lon.
- If the instruction is ambiguous about a value, pick a reasonable default \
rather than omitting the field.
"""


@dataclass
class InterpretResult:
    ok: bool
    plan: Optional[MissionPlan]
    raw_json: Optional[dict]
    errors: List[str]
    attempts: int


def _extract_json(text: str) -> dict:
    """Ollama's format="json" mode should return pure JSON, but local
    models occasionally still wrap it in prose or markdown fences despite
    instructions -- this is a defensive fallback, not the primary path."""
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
    """Isolated in its own function so tests can mock exactly this call
    and exercise the retry/validation logic without a live model."""
    resp = requests.post(
        OLLAMA_URL,
        json={"model": model, "messages": messages, "format": "json", "stream": False},
        timeout=120,
    )
    resp.raise_for_status()
    return resp.json()["message"]["content"]


def interpret_prompt(
    user_prompt: str,
    safety_cfg: SafetyConfig,
    model: str = DEFAULT_MODEL,
    max_attempts: int = 2,
) -> InterpretResult:
    """Turn a natural-language prompt into a validated MissionPlan, with
    one bounded retry if the first draft fails validation. Never raises on
    a bad LLM draft -- returns ok=False with the errors instead, so
    callers decide what to do rather than the whole process crashing on
    a model hallucination."""

    system = _build_system_prompt(safety_cfg)
    messages = [
        {"role": "system", "content": system},
        {"role": "user", "content": user_prompt},
    ]

    last_errors: List[str] = []
    last_raw: Optional[dict] = None

    for attempt in range(1, max_attempts + 1):
        print(f"-- Attempt {attempt}/{max_attempts}: querying {model} ...")
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
        print(f"-- Draft JSON:\n{json.dumps(raw, indent=2)}")

        result = validate_mission_json(raw, safety_cfg)
        if result.ok:
            plan = MissionPlan.model_validate(raw)
            return InterpretResult(ok=True, plan=plan, raw_json=raw, errors=[], attempts=attempt)

        last_errors = result.errors
        print(f"-- Validation FAILED (attempt {attempt}): {result.errors}")

        if attempt < max_attempts:
            error_text = "\n".join(f"- {e}" for e in result.errors)
            messages.append({"role": "assistant", "content": raw_text})
            messages.append({"role": "user", "content":
                f"That mission failed validation with these errors:\n{error_text}\n\n"
                "Reply with a corrected JSON object that fixes all of these issues. "
                "Output ONLY the JSON object."})

    return InterpretResult(ok=False, plan=None, raw_json=last_raw, errors=last_errors, attempts=max_attempts)


def _default_safety_config() -> SafetyConfig:
    # Same reference position used throughout the project's fixtures.
    return SafetyConfig(
        home_lat=10.5270, home_lon=76.2140,
        max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
        max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
    )


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print('Usage: python3 llm_interpreter.py "<natural language mission prompt>"')
        sys.exit(1)

    result = interpret_prompt(sys.argv[1], _default_safety_config())

    if not result.ok:
        print(f"\n-- FAILED after {result.attempts} attempt(s). Not executing. Errors:")
        for err in result.errors:
            print(f"   - {err}")
        sys.exit(1)

    print(f"\n-- SUCCESS after {result.attempts} attempt(s). Validated MissionPlan:")
    print(result.plan.model_dump_json(indent=2))
