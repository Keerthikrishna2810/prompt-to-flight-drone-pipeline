#!/bin/bash
# fly_vision.sh
#
# Vision-follow counterpart to fly.sh / fly_squad.sh. Starts Ollama (if
# not already running), a single PX4 SITL instance using a
# camera-equipped vehicle model, and runs the given vision-follow prompt.
#
#   source fly_vision.sh "search this route and follow the first person you see" [real]
#
# Argument 2, if the literal word "real", turns on --real-camera
# --real-detector (live Gazebo camera + YOLO). Omit it (or run with no
# second argument) to use the tested MockDetector/NullCameraSource path
# instead -- interprets and validates the mission, flies the search
# route, but never actually "sees" anything, so it always takes the
# "target not found" branch. That's still a genuine, fully-tested run of
# everything except the live vision piece.
#
# -----------------------------------------------------------------------
# HONESTY NOTE: the "real" path (camera + detector) is the one part of
# Challenge 3 not verified live -- see vision_camera_gz.py's own
# docstring for exactly why and what to check first if it doesn't work.
# Everything else here (Ollama, Gazebo, PX4, the mission logic itself)
# uses the exact same launch pattern already proven working for
# Challenge 1's fly_squad.sh.
# -----------------------------------------------------------------------

if [ -z "$1" ]; then
    echo 'Usage: source fly_vision.sh "<natural language vision-follow instruction>" [real]'
    return 1 2>/dev/null || exit 1
fi
PROMPT="$1"
USE_REAL="$2"

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

# -- Gazebo server ---------------------------------------------------------
if ! pgrep -f "gz sim" > /dev/null; then
    if [ -n "$DISPLAY" ]; then
        GZ_FLAGS=""
        echo "== Starting Gazebo world (GUI -- DISPLAY=$DISPLAY detected) =="
    else
        GZ_FLAGS="--headless"
        echo "== Starting Gazebo world (headless) =="
    fi
    cd /root/PX4-Autopilot
    setsid bash -c "python3 Tools/simulation/gz/simulation-gazebo $GZ_FLAGS" \
        < /dev/null > /root/project/gz_server.log 2>&1 &
    echo "== Waiting ~40s for Gazebo server (longer on first run) =="
    sleep 40

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
        || echo "== WARNING: unpause call failed -- see fly_squad.sh's equivalent note =="

    cd /root/project
else
    echo "== Gazebo world already running =="
fi

# -- One PX4 instance, camera-equipped model if going for the real path ---
# gz_x500_mono_cam is PX4's own camera-equipped SITL model variant.
# If your PX4 checkout doesn't ship this exact model name, `ls
# /root/PX4-Autopilot/Tools/simulation/gz/models/` inside the container
# to see what camera-equipped variants ARE available and adjust below.
if [ "$USE_REAL" == "real" ]; then
    PX4_MODEL="gz_x500_mono_cam"
else
    PX4_MODEL="gz_x500"
fi

if ! pgrep -f "bin/px4 -i 0" > /dev/null; then
    echo "== Starting PX4 SITL instance (model: ${PX4_MODEL}) =="
    cd /root/PX4-Autopilot
    setsid bash -c "PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=${PX4_MODEL} \
        ./build/px4_sitl_default/bin/px4 -i 0" \
        < /dev/null > /root/project/px4_sitl_instance_0.log 2>&1 &
    echo "== Waiting ~20s for the instance to come up =="
    sleep 20
    cd /root/project
else
    echo "== PX4 SITL instance already running =="
fi

REAL_FLAGS=""
if [ "$USE_REAL" == "real" ]; then
    REAL_FLAGS="--real-camera --real-detector"
    if ! pgrep -f "ros_gz_bridge" > /dev/null; then
        echo "== Starting camera topic bridge =="
        source /opt/ros/humble/setup.bash 2>/dev/null
        setsid bash -c "source /opt/ros/humble/setup.bash && ros2 run ros_gz_bridge parameter_bridge \
            /world/${WORLD_NAME:-default}/model/x500_mono_cam_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image" \
            < /dev/null > /root/project/camera_bridge.log 2>&1 &
        sleep 3
    fi
fi

echo "== Running vision pipeline =="
source /opt/ros/humble/setup.bash 2>/dev/null
python3 /root/project/vision_main.py "$PROMPT" --system-address udpin://0.0.0.0:14540 $REAL_FLAGS
