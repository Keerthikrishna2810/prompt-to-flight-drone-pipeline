"""
squad_main.py

Challenge 1 completion -- the full squad pipeline, end to end, mirroring
main.py exactly:

    natural language squad instruction
        -> squad_interpreter.interpret_squad_prompt()   [Ollama + validate_squad_mission_json, bounded retry]
        -> squad_executor.fly_squad()                    [N MissionExecutors, concurrent]
        -> N PX4 SITL instances + one shared Gazebo world, via MAVSDK

Same "no new logic of its own" principle main.py states for itself: this
file only wires together squad_interpreter.py and squad_executor.py, each
already independently tested on its own. If this file is deleted, both
still work and are still independently testable.
"""

import argparse
import asyncio
import sys

from squad_interpreter import interpret_squad_prompt, DEFAULT_MODEL
from squad_executor import fly_squad
from validator import SafetyConfig


def _default_safety_config() -> SafetyConfig:
    return SafetyConfig(
        home_lat=10.5270, home_lon=76.2140,
        max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
        max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
    )


async def fly_squad_from_prompt(
    prompt: str,
    base_port: int = 14540,
    model: str = DEFAULT_MODEL,
    dry_run: bool = False,
    min_separation_m: float = 3.0,
    use_rviz: bool = False,
) -> None:
    cfg = _default_safety_config()

    print(f'-- Interpreting squad prompt: "{prompt}"')
    result = interpret_squad_prompt(prompt, cfg, model=model, min_separation_m=min_separation_m)

    if not result.ok:
        print(f"\n-- Interpretation FAILED after {result.attempts} attempt(s). Not executing. Errors:")
        for err in result.errors:
            print(f"   - {err}")
        sys.exit(1)

    print(f"\n-- Interpretation SUCCEEDED after {result.attempts} attempt(s). "
          f"{len(result.plans)} drone(s). Executing...\n")

    viz = None
    if use_rviz:
        try:
            from formation_viz import RvizFormationPublisher
            viz = RvizFormationPublisher(reference_lat=cfg.home_lat, reference_lon=cfg.home_lon)
            print("-- RViz publisher connected")
        except Exception as e:
            print(f"-- WARNING: could not start RViz publisher, continuing without it: {e}")

    exec_result = await fly_squad(
        result.plans, cfg.home_lat, cfg.home_lon,
        rtl_return_alt_m=cfg.max_alt_m, base_port=base_port,
        dry_run=dry_run, viz=viz,
    )

    print("\n-- Full squad pipeline run complete. Audit logs:")
    import json
    for i, log in enumerate(exec_result.audit_logs):
        print(f"\n-- drone[{i}] --")
        print(json.dumps([{"step": e.step, "action": e.action, "params": e.params} for e in log], indent=2))

    if not exec_result.ok:
        print("\n-- One or more drones FAILED:")
        for err in exec_result.errors:
            print(f"   - {err}")
        sys.exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Full squad pipeline: natural language instruction -> validated squad -> live formation flight."
    )
    parser.add_argument("prompt", help='e.g. "send three drones in a wedge to sweep this route"')
    parser.add_argument("--base-port", type=int, default=14540,
                         help="MAVSDK port for drone 0; drone N connects on base_port + N")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--min-separation-m", type=float, default=3.0)
    parser.add_argument("--dry-run", action="store_true",
                         help="Interpret and validate, but don't connect to any live vehicle")
    parser.add_argument("--rviz", action="store_true", help="Publish live positions + planned routes to RViz")
    args = parser.parse_args()

    asyncio.run(fly_squad_from_prompt(
        args.prompt, args.base_port, args.model, args.dry_run, args.min_separation_m, args.rviz
    ))
