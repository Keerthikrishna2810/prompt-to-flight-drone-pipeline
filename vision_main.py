"""
vision_main.py

Challenge 3 completion -- the full vision pipeline, mirroring
squad_main.py and main.py:

    natural language vision-follow instruction
        -> vision_interpreter.interpret_vision_prompt()  [Ollama + validate_vision_mission_json, bounded retry]
        -> vision_executor.VisionFollowExecutor.execute()  [search route, poll camera, snapshot + follow on detection]
        -> one PX4 SITL instance + camera-equipped Gazebo model, via MAVSDK

Same "no new logic of its own" principle main.py/squad_main.py state for
themselves: this file only wires together vision_interpreter.py and
vision_executor.py, each already independently tested.
"""

import argparse
import asyncio
import pathlib
import sys

from mavsdk import System

from vision_interpreter import interpret_vision_prompt, DEFAULT_MODEL
from vision_executor import VisionFollowExecutor, NullCameraSource
from vision_detector import MockDetector
from validator import SafetyConfig


def _default_safety_config() -> SafetyConfig:
    return SafetyConfig(
        home_lat=10.5270, home_lon=76.2140,
        max_geofence_radius_m=200.0, min_alt_m=3.0, max_alt_m=25.0,
        max_speed_mps=8.0, max_leg_distance_m=150.0, loop_closure_tolerance_m=15.0,
    )


def _build_camera_and_detector(use_real_camera: bool, use_real_detector: bool):
    """Both real integrations (Gazebo camera stream, YOLO) are optional
    and lazily imported -- see vision_camera_gz.py's and
    vision_detector.YoloDetector's own docstrings for exactly what's
    verified versus best-effort here. Falls back to the same
    NullCameraSource/MockDetector the test suite uses if either real
    piece isn't available, rather than crashing the whole pipeline."""
    camera = NullCameraSource()
    detector = MockDetector([[]])

    if use_real_camera:
        try:
            from vision_camera_gz import GzCameraSource
            camera = GzCameraSource()
            print("-- Using live Gazebo camera source")
        except Exception as e:
            print(f"-- WARNING: could not start Gazebo camera source, falling back to no camera: {e}")

    if use_real_detector:
        try:
            from vision_detector import YoloDetector
            detector = YoloDetector()
            print("-- Using live YOLO detector")
        except Exception as e:
            print(f"-- WARNING: could not start YOLO detector, falling back to no detections: {e}")

    return camera, detector


async def fly_vision_from_prompt(
    prompt: str,
    system_address: str = "udpin://0.0.0.0:14540",
    model: str = DEFAULT_MODEL,
    dry_run: bool = False,
    use_real_camera: bool = False,
    use_real_detector: bool = False,
    snapshot_dir: pathlib.Path = pathlib.Path("/root/project/vision_snapshots"),
) -> None:
    cfg = _default_safety_config()

    print(f'-- Interpreting vision prompt: "{prompt}"')
    result = interpret_vision_prompt(prompt, cfg, model=model)

    if not result.ok:
        print(f"\n-- Interpretation FAILED after {result.attempts} attempt(s). Not executing. Errors:")
        for err in result.errors:
            print(f"   - {err}")
        sys.exit(1)

    print(f"\n-- Interpretation SUCCEEDED after {result.attempts} attempt(s). "
          f"Target: '{result.plan.target_class}'. Executing...\n")

    camera, detector = _build_camera_and_detector(use_real_camera, use_real_detector)

    drone = None if dry_run else System()
    executor = VisionFollowExecutor(
        drone, cfg.home_lat, cfg.home_lon,
        camera=camera, detector=detector, snapshot_dir=snapshot_dir,
        max_geofence_radius_m=cfg.max_geofence_radius_m,
        rtl_return_alt_m=cfg.max_alt_m, dry_run=dry_run,
    )
    await executor.connect_and_wait_ready(system_address)

    exec_result = await executor.execute(result.plan, result.search_plan)

    print("\n-- Vision pipeline run complete.")
    print(f"   target_found: {exec_result.target_found}")
    if exec_result.snapshot_path:
        print(f"   snapshot: {exec_result.snapshot_path}")
    if not exec_result.ok:
        print("\n-- FAILED:")
        for err in exec_result.errors:
            print(f"   - {err}")
        sys.exit(1)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Full vision pipeline: natural language instruction -> validated search+follow -> live flight."
    )
    parser.add_argument("prompt", help='e.g. "search this route and follow the first person you see"')
    parser.add_argument("--system-address", default="udpin://0.0.0.0:14540")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--dry-run", action="store_true",
                         help="Interpret and validate, but don't connect to any live vehicle")
    parser.add_argument("--real-camera", action="store_true",
                         help="Use the live Gazebo camera bridge (see vision_camera_gz.py -- best-effort, unverified live)")
    parser.add_argument("--real-detector", action="store_true",
                         help="Use a real YOLO detector (requires: pip install ultralytics)")
    args = parser.parse_args()

    asyncio.run(fly_vision_from_prompt(
        args.prompt, args.system_address, args.model, args.dry_run,
        args.real_camera, args.real_detector,
    ))
