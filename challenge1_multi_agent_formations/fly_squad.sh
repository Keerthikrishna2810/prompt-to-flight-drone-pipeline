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
    # If a real X display is attached (DISPLAY is set), run WITH the GUI so
    # you can watch the drones -- otherwise fall back to headless. This is
    # the only difference between "I can see it in Gazebo" and "it just
    # runs in the background" -- see the docker run flags in the README
    # section below for how DISPLAY gets set in the first place.
    if [ -n "$DISPLAY" ]; then
        GZ_FLAGS=""
        echo "== Starting shared Gazebo world (GUI -- DISPLAY=$DISPLAY detected) =="
    else
        GZ_FLAGS="--headless"
        echo "== Starting shared Gazebo world (headless -- no DISPLAY detected) =="
    fi
    cd /root/PX4-Autopilot
    setsid bash -c "python3 Tools/simulation/gz/simulation-gazebo $GZ_FLAGS" \
        < /dev/null > /root/project/gz_server.log 2>&1 &
    echo "== Waiting ~40s for Gazebo server (longer on first run -- it downloads vehicle models from GitHub the first time) =="
    sleep 40

    # Explicitly unpause the world clock. gz sim worlds can start paused;
    # if that happens, PX4 will connect and accept every command
    # successfully but the simulated clock (and therefore vehicle
    # position) never advances -- every telemetry read comes back
    # identical, which is exactly the "frozen drone" symptom this fixes.
    # Discover the actual world name rather than assuming "default" -- it
    # varies by PX4 version/world file, and guessing wrong fails the
    # service call silently while leaving the world paused.
    WORLD_NAME=""
    for i in $(seq 1 15); do
        WORLD_NAME=$(gz topic -l 2>/dev/null | grep -m1 -oP '(?<=^/world/)[^/]+(?=/clock$)')
        [ -n "$WORLD_NAME" ] && break
        sleep 1
    done
    WORLD_NAME="${WORLD_NAME:-default}"
    echo "== Unpausing world '${WORLD_NAME}' =="
    gz service -s /world/${WORLD_NAME}/control \
        --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
        --timeout 5000 --req 'pause: false' \
        || echo "== WARNING: unpause call to world '${WORLD_NAME}' failed -- run 'gz topic -l' yourself to find the real world name if drones still don't move =="

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
        setsid bash -c "source /opt/ros/humble/setup.bash && ros2 run rviz2 rviz2 -d /root/project/rviz/formation.rviz" \
            < /dev/null > /root/project/rviz.log 2>&1 &
        sleep 3
    fi
fi

echo "== Running full squad pipeline (${DRONE_COUNT} drones) =="
source /opt/ros/humble/setup.bash 2>/dev/null
python3 /root/project/squad_main.py "$PROMPT" --base-port 14540 $RVIZ_FLAG
