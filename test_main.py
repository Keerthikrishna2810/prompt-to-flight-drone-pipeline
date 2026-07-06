"""
test_main.py

Proves main.py's wiring end-to-end -- interpreter output actually flows
into the executor correctly -- with Ollama mocked and dry_run=True, so no
live model or simulator is needed. Same philosophy as every other day:
verify the connection before trusting it against something live.
"""

import asyncio
import json
from unittest.mock import patch

from main import fly_from_prompt

VALID_LOOP = json.dumps({
    "mission_type": "loop", "repetitions": 2, "speed_mps": 4,
    "waypoints": [
        {"lat": 10.527409, "lon": 76.214284, "alt_m": 10},
        {"lat": 10.527439, "lon": 76.214284, "alt_m": 10},
        {"lat": 10.527439, "lon": 76.214314, "alt_m": 10},
        {"lat": 10.527409, "lon": 76.214314, "alt_m": 10},
    ],
})


def test_full_pipeline_dry_run():
    with patch("llm_interpreter._call_ollama", return_value=VALID_LOOP):
        asyncio.run(fly_from_prompt("fly a small square patrol loop twice", dry_run=True))
    # No exception raised == the interpreter's MissionPlan was accepted by
    # the executor's execute() without any shape/type mismatch between the
    # two files that were, until now, only ever tested in isolation.


if __name__ == "__main__":
    try:
        test_full_pipeline_dry_run()
        print("[PASS] test_full_pipeline_dry_run")
    except Exception as e:
        print(f"[FAIL] test_full_pipeline_dry_run: {e}")
        raise
