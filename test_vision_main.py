"""
test_vision_main.py

Proves vision_main.py's wiring end-to-end -- vision_interpreter output
actually flows into vision_executor correctly -- with Ollama mocked and
dry_run=True, no live model, camera, or simulator needed.
"""

import asyncio
import json
from unittest.mock import patch

from vision_main import fly_vision_from_prompt

VALID_VISION_MISSION = json.dumps({
    "target_class": "person",
    "search_waypoints": [
        {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
        {"lat": 10.5272, "lon": 76.2145, "alt_m": 10},
    ],
    "search_speed_mps": 4,
    "follow_distance_m": 8,
    "follow_altitude_m": 10,
    "max_follow_duration_s": 60,
    "detection_confidence_threshold": 0.5,
})


def test_full_vision_pipeline_dry_run(tmp_path):
    with patch("vision_interpreter._call_ollama", return_value=VALID_VISION_MISSION):
        asyncio.run(fly_vision_from_prompt(
            "search this route and follow the first person you see",
            dry_run=True,
            snapshot_dir=tmp_path / "snapshots",
        ))
    # No exception == vision_interpreter's VisionFollowPlan + search
    # MissionPlan were accepted by vision_executor's execute() with no
    # shape mismatch between the two files, each until now only tested
    # in isolation. Target isn't found (MockDetector default returns no
    # detections), so this also exercises the "search complete, RTL"
    # path end-to-end.


if __name__ == "__main__":
    import tempfile
    import pathlib
    try:
        with tempfile.TemporaryDirectory() as d:
            test_full_vision_pipeline_dry_run(pathlib.Path(d))
        print("[PASS] test_full_vision_pipeline_dry_run")
    except Exception as e:
        print(f"[FAIL] test_full_vision_pipeline_dry_run: {e}")
        raise
