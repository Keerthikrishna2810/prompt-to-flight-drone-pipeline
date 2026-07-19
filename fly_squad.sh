#!/bin/bash
# fly_squad.sh
#
# Multi-vehicle counterpart to fly.sh. Starts Ollama (if not already
# running) plus N PX4 SITL instances against ONE shared Gazebo world (not
# N separate worlds), waits for everything to be ready, then runs the
# given squad prompt through the full squad pipeline.
#
# MUST be `source`d, not executed, for the same reason as fly.sh -- the
# backgrounded processes need to stay attached to your shell:
#
#   source /root/project/fly_squad.sh "send three drones in a wedge down this route" 3
#
# Arguments: 1) the prompt (required), 2) drone count (optional, default 3),
#            3) pass "rviz" as the third argument to also publish to RViz
#
# IMPORTANT: DRONE_COUNT below must be >= however many drones your prompt
# will actually produce (e.g. "send three drones..." needs DRONE_COUNT=3
# PX4 instances up before squad_main.py runs). squad_executor.py now fails
# fast with a clear error within 30s per drone if an instance is missing
# rather than hanging -- but matching the count still avoids that failure
# in the first place.
#
# -----------------------------------------------------------------------
# HONESTY NOTE (read this before debugging): unlike fly.sh, this script's
# multi-vehicle Gazebo spawn was written against PX4's documented
# multi-vehicle SITL approach (one Gazebo server, multiple `px4` binary
# instances joining it via -i and PX4_GZ_MODEL_POSE) but was NOT verified
# against a live Gazebo session while building this -- no GPU/display was
# available in the environment this was written in. The single-drone path
# (fly.sh, main.py) IS fully verified live, per WRITEUP.md. If this script
# doesn't spawn N vehicles cleanly on the first try, the most likely fix
# is checking Tools/simulation/gz/simulation-gazebo --help in your PX4
# checkout for this version's exact flags -- they've shifted across PX4
# releases before.
# -----------------------------------------------------------------------

if [ -z "$1" ]; then
    echo 'Usage: source fly_squad.sh "<natural language squad instruction>" [drone_count]'
    return 1 2>/dev/null || exit 1
fi
PROMPT="$1"
DRONE_COUNT="${2:-3}"

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

# -- Gazebo server (shared world, no vehicle attached yet) ----------------
if ! pgrep -f "gz sim" > /dev/null; then
    echo "== Starting shared Gazebo world (headless) =="
    cd /root/PX4-Autopilot
    setsid bash -c "HEADLESS=1 python3 Tools/simulation/gz/simulation-gazebo" \
        < /dev/null > /root/project/gz_server.log 2>&1 &
    echo "== Waiting ~15s for Gazebo server =="
    sleep 15
    cd /root/project
else
    echo "== Gazebo world already running =="
fi

# -- N PX4 SITL instances, one per drone, all joining the shared world ----
# Instance i uses MAVLink UDP port 14540+i by default -- this is what lets
# squad_executor.py's --base-port 14540 (the default) connect to drone 0
# on 14540, drone 1 on 14541, and so on with zero extra configuration.
# Spread instances 5m apart along the world's X axis so they don't spawn
# stacked on top of each other.
if ! pgrep -f "bin/px4 -i 0" > /dev/null; then
    echo "== Starting ${DRONE_COUNT} PX4 SITL instances =="
    cd /root/PX4-Autopilot
    for i in $(seq 0 $((DRONE_COUNT - 1))); do
        POSE_X=$((i * 5))
        setsid bash -c "PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 \
            PX4_GZ_MODEL_POSE=\"${POSE_X},0\" ./build/px4_sitl_default/bin/px4 -i ${i}" \
            < /dev/null > /root/project/px4_sitl_instance_${i}.log 2>&1 &
        sleep 2  # stagger instance startup -- launching all N at once has been
                 # a source of flaky spawns for other multi-vehicle PX4 setups
    done
    echo "== Waiting ~20s for all instances to come up =="
    sleep 20
    cd /root/project
else
    echo "== PX4 SITL instances already running =="
fi

RVIZ_FLAG=""
if [ "$3" == "rviz" ]; then
    RVIZ_FLAG="--rviz"
    if ! pgrep -f "rviz2" > /dev/null; then
        echo "== Starting RViz (rviz/formation.rviz) =="
        setsid ros2 run rviz2 rviz2 -d /root/project/rviz/formation.rviz \
            < /dev/null > /root/project/rviz.log 2>&1 &
        sleep 3
    fi
fi

echo "== Running full squad pipeline (${DRONE_COUNT} drones) =="
python3 /root/project/squad_main.py "$PROMPT" --base-port 14540 $RVIZ_FLAG
