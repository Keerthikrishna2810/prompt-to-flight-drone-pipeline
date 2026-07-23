#!/bin/bash
# Start PX4 SITL with Gazebo Harmonic
cd ~/PX4-Autopilot 2>/dev/null || cd /root/PX4-Autopilot 2>/dev/null || true
make px4_sitl gz_x500
