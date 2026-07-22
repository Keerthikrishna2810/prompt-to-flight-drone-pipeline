"""
test_squad_main.py

Proves squad_main.py's wiring end-to-end -- squad_interpreter output
actually flows into squad_executor correctly -- with Ollama mocked and
dry_run=True, no live model or simulator needed. Same philosophy as
test_main.py.
"""

import asyncio
import json
from unittest.mock import patch

from squad_main import fly_squad_from_prompt

VALID_SQUAD = json.dumps({
    "drone_count": 3, "mode": "formation", "formation": "wedge", "spacing_m": 10.0,
    "mission_type": "route", "repetitions": 1, "speed_mps": 5,
    "waypoints": [
        {"lat": 10.5270, "lon": 76.2140, "alt_m": 10},
        {"lat": 10.5272, "lon": 76.2145, "alt_m": 10},
    ],
})


def test_full_squad_pipeline_dry_run():
    with patch("squad_interpreter._call_ollama", return_value=VALID_SQUAD):
        asyncio.run(fly_squad_from_prompt("send three drones in a wedge down this route", dry_run=True))
    # No exception == squad_interpreter's MissionPlans were accepted by
    # squad_executor's fly_squad() with no shape mismatch between the two
    # files, each until now only tested in isolation.


if __name__ == "__main__":
    try:
        test_full_squad_pipeline_dry_run()
        print("[PASS] test_full_squad_pipeline_dry_run")
    except Exception as e:
        print(f"[FAIL] test_full_squad_pipeline_dry_run: {e}")
        raise
