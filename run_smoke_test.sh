#!/bin/bash
# run_smoke_test.sh
#
# Boots PX4 SITL (headless, Gazebo Harmonic x500 quad) in the background,
# waits for it to come up, then runs the MAVSDK smoke test against it.
#
# Usage (inside the container):
#   /root/project/run_smoke_test.sh

set -euo pipefail

PX4_HOME="/root/PX4-Autopilot"
BOOT_WAIT_SECONDS=30

echo "== Starting PX4 SITL (headless, gz_x500) =="
cd "${PX4_HOME}"
HEADLESS=1 make px4_sitl gz_x500 > /root/project/px4_sitl.log 2>&1 &
PX4_PID=$!

echo "== PX4 SITL started (pid ${PX4_PID}), waiting ${BOOT_WAIT_SECONDS}s for boot =="
echo "   (tail -f /root/project/px4_sitl.log in another shell to watch boot output)"
sleep "${BOOT_WAIT_SECONDS}"

echo "== Running smoke test =="
python3 /root/project/smoke_test.py

echo "== Smoke test finished, shutting down PX4 SITL =="
kill "${PX4_PID}" 2>/dev/null || true
wait "${PX4_PID}" 2>/dev/null || true

echo "== Done =="
