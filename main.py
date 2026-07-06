"""
main.py

Day 4 completion: the full pipeline, end to end.

    natural language prompt
        -> llm_interpreter.interpret_prompt()   [Ollama + validate_mission_json, bounded retry]
        -> mission_executor.MissionExecutor      [the same executor used for fixtures since Day 3]
        -> PX4 SITL / Gazebo Harmonic

This file contains no new logic of its own -- it only wires together
pieces that were each already built and independently tested on their own
day. That's deliberate: the LLM's output goes through the exact same
validate-then-execute path a hand-written fixture JSON always has. If this
file is deleted, every other file in the project still works and is still
independently testable -- that's the sign the architecture boundary held.
"""

import argparse
import asyncio
import sys

from mavsdk import System

from llm_interpreter import interpret_prompt, DEFAULT_MODEL
from mission_executor import MissionExecutor
from validator import SafetyConfig


def _default_safety_config() -> SafetyConfig:
    # Same reference position used throughout the project's fixtures and CLIs.
    return SafetyConfig(
        home_lat=10.5270, home_lon=76.2140,
        max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
        max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
    )


async def fly_from_prompt(
    prompt: str,
    system_address: str = "udpin://0.0.0.0:14540",
    model: str = DEFAULT_MODEL,
    dry_run: bool = False,
) -> None:
    cfg = _default_safety_config()

    print(f'-- Interpreting prompt: "{prompt}"')
    result = interpret_prompt(prompt, cfg, model=model)

    if not result.ok:
        print(f"\n-- Interpretation FAILED after {result.attempts} attempt(s). Not executing. Errors:")
        for err in result.errors:
            print(f"   - {err}")
        sys.exit(1)

    print(f"\n-- Interpretation SUCCEEDED after {result.attempts} attempt(s). Executing mission...\n")

    drone = None if dry_run else System()
    executor = MissionExecutor(
        drone, cfg.home_lat, cfg.home_lon,
        rtl_return_alt_m=cfg.max_alt_m,
        dry_run=dry_run,
    )
    await executor.connect_and_wait_ready(system_address)
    await executor.execute(result.plan)

    print("\n-- Full pipeline run complete. Audit log:")
    print(executor.audit_log_as_json())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Full pipeline: natural language prompt -> validated mission -> live flight."
    )
    parser.add_argument("prompt", help='e.g. "fly a small square patrol loop twice at 10 meters altitude"')
    parser.add_argument("--system-address", default="udpin://0.0.0.0:14540")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--dry-run", action="store_true",
                         help="Interpret and validate, but don't connect to a live vehicle")
    args = parser.parse_args()

    asyncio.run(fly_from_prompt(args.prompt, args.system_address, args.model, args.dry_run))
