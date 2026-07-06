#!/bin/bash
# fly.sh
#
# Single-command launcher for the full pipeline. Starts Ollama and PX4
# SITL only if they aren't already running (safe to re-run repeatedly in
# the same container -- this is what avoids the "PX4 server already
# running for instance 0" conflict from starting a second PX4 instance on
# top of a still-running one), waits for both to be ready, then runs the
# given prompt through the full pipeline.
#
# MUST be `source`d, not executed, so the backgrounded Ollama/PX4
# processes stay attached to your current shell session:
#
#   source /root/project/fly.sh "fly a small square patrol loop twice at 10 meters altitude"

if [ -z "$1" ]; then
    echo 'Usage: source fly.sh "<natural language mission prompt>"'
    return 1 2>/dev/null || exit 1
fi
PROMPT="$1"

# -- Ollama --------------------------------------------------------------
if ! curl -s -o /dev/null http://localhost:11434/; then
    echo "== Starting Ollama =="
    ollama serve > /root/project/ollama_server.log 2>&1 &
    for i in $(seq 1 30); do
        curl -s -o /dev/null http://localhost:11434/ && break
        sleep 1
    done
else
    echo "== Ollama already running =="
fi

# -- PX4 SITL (headless, setsid-detached so it can never be SIGTTIN'd) ---
if ! pgrep -f "bin/px4" > /dev/null; then
    echo "== Starting PX4 SITL (headless, gz_x500) =="
    cd /root/PX4-Autopilot
    setsid bash -c "HEADLESS=1 make px4_sitl gz_x500" \
        < /dev/null > /root/project/px4_sitl.log 2>&1 &
    echo "== Waiting ~30s for PX4 to boot =="
    sleep 30
    cd /root/project
else
    echo "== PX4 SITL already running =="
fi

echo "== Running full pipeline =="
python3 /root/project/main.py "$PROMPT"
