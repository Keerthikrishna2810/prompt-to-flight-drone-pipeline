#!/usr/bin/env python3
"""
smoke_test.py

Day 1 exit criterion for this project:
  "Drone visibly moves in Gazebo from a script with zero AI involved."

This script talks to PX4 SITL directly over MAVSDK -- no LLM, no JSON schema,
no validator. It exists purely to prove the plumbing (PX4 <-> Gazebo Harmonic
<-> MAVSDK <-> Python) works end to end before any of the pipeline layers
(schema / validator / executor / LLM) are built on top of it.

Flight profile: arm -> takeoff -> fly to one offset waypoint -> return to
launch. That's it.

Source: adapted from the official MAVSDK-Python example patterns.
https://github.com/mavlink/MAVSDK-Python (MIT license)
"""

import asyncio
from mavsdk import System


async def run():
    drone = System()
    print("-- Connecting to PX4 SITL on udp://:14540 ...")
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("-- Connected to drone")
            break

    print("-- Waiting for global position + home position lock ...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    # Grab the home position so our target altitude/offset is relative to it,
    # rather than hardcoding PX4's default Gazebo world coordinates.
    home_lat = home_lon = home_alt = None
    async for position in drone.telemetry.position():
        home_lat = position.latitude_deg
        home_lon = position.longitude_deg
        home_alt = position.absolute_altitude_m
        break

    print(f"-- Home position: lat={home_lat:.7f}, lon={home_lon:.7f}, "
          f"alt={home_alt:.1f}m AMSL")

    print("-- Arming")
    await drone.action.arm()

    takeoff_alt_m = 10.0
    print(f"-- Setting takeoff altitude to {takeoff_alt_m}m")
    await drone.action.set_takeoff_altitude(takeoff_alt_m)

    print("-- Taking off")
    await drone.action.takeoff()
    await asyncio.sleep(10)

    # ~30m north-east offset, 15m AGL. Small, deliberate offset so it's
    # visually obvious in Gazebo without leaving the default world bounds.
    target_lat = home_lat + 0.00027   # ~30m north
    target_lon = home_lon + 0.00027   # ~30m east
    target_alt = home_alt + 15.0      # 15m AGL, absolute (AMSL)

    print(f"-- Flying to waypoint: lat={target_lat:.7f}, lon={target_lon:.7f}, "
          f"alt={target_alt:.1f}m AMSL")
    await drone.action.goto_location(target_lat, target_lon, target_alt, 0.0)
    await asyncio.sleep(20)

    print("-- Returning to launch")
    await drone.action.return_to_launch()
    await asyncio.sleep(20)

    print("-- Smoke test complete. Check Gazebo / MAVSDK logs to confirm "
          "the vehicle actually moved and landed.")


if __name__ == "__main__":
    asyncio.run(run())
