"""
formation_viz.py

Challenge 1 extra -- publishes the squad's planned routes and live
positions to RViz, so the formation is visible even when you can't (or
don't want to) watch Gazebo's own 3D view.

This is intentionally NOT wired into PX4's own DDS/uXRCE bridge (the
usual "ROS 2 talks to PX4 directly" setup) -- that bridge is a real extra
moving part (uxrce_dds_agent, micro-ROS, topic namespacing per instance)
and bringing it in just for visualization would add exactly the kind of
extra failure surface that risks a stuck build. Instead, this node is fed
directly by squad_executor.py, which already has every drone's planned
route (from squad_validator.py) and live MAVSDK telemetry position --
this file just re-publishes those two things as RViz Markers. If it's
never called, or rclpy isn't installed, nothing about the actual flight
is affected -- see the VizPublisher protocol and _safe_viz_call() in
squad_executor.py.

No TF tree is used on purpose, for the same "keep the extra moving parts
to a minimum" reason -- every Marker is published directly in a "map"
frame using the same north/east metre offsets the rest of the project
already computes relative to a fixed reference point (SafetyConfig's
home_lat/home_lon). Set RViz's Fixed Frame to "map" (the provided
rviz/formation.rviz config already does this) and no static transform
publisher is needed.

Run alongside Gazebo:  ros2 run rviz2 rviz2 -d rviz/formation.rviz
"""

import math
from typing import Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from schema import MissionPlan

_EARTH_RADIUS_M = 6371000.0

# One distinct, stable color per drone index (RGBA, 0-1), cycling if
# drone_count exceeds this list's length -- unlikely at the project's
# 2-6 drone cap, but never crash over it either.
_DRONE_COLORS = [
    (0.90, 0.20, 0.20, 1.0),  # red
    (0.20, 0.50, 0.90, 1.0),  # blue
    (0.20, 0.80, 0.30, 1.0),  # green
    (0.95, 0.75, 0.10, 1.0),  # amber
    (0.65, 0.25, 0.85, 1.0),  # purple
    (0.10, 0.80, 0.80, 1.0),  # teal
]


def _latlon_to_local_xy(ref_lat: float, ref_lon: float, lat: float, lon: float) -> Point:
    """Same small-distance approximation as mission_executor.py's
    _offset_from_reference -- east becomes RViz x, north becomes RViz y,
    matching ROS's standard ENU convention."""
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)
    north_m = d_lat * _EARTH_RADIUS_M
    east_m = d_lon * _EARTH_RADIUS_M * math.cos(math.radians(ref_lat))
    return Point(x=east_m, y=north_m, z=0.0)


class RvizFormationPublisher:
    """Implements squad_executor.py's VizPublisher protocol (publish_plan,
    publish_position) -- duck-typed, squad_executor.py never imports this
    class directly, only ever through the optional --rviz CLI flag."""

    def __init__(self, reference_lat: float = 10.5270, reference_lon: float = 76.2140, node_name: str = "formation_viz"):
        self.reference_lat = reference_lat
        self.reference_lon = reference_lon
        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = Node(node_name)
        self._plan_pub = self._node.create_publisher(MarkerArray, "/squad/planned_routes", 10)
        self._pos_pub = self._node.create_publisher(MarkerArray, "/squad/live_positions", 10)
        # Trail of recent positions per drone, purely cosmetic (a fading
        # breadcrumb), capped so memory can't grow unbounded on a long flight.
        self._trails: Dict[int, list] = {}
        self._max_trail_points = 200

    def _color(self, drone_index: int) -> ColorRGBA:
        r, g, b, a = _DRONE_COLORS[drone_index % len(_DRONE_COLORS)]
        return ColorRGBA(r=r, g=g, b=b, a=a)

    def publish_plan(self, drone_index: int, plan: MissionPlan) -> None:
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self._node.get_clock().now().to_msg()
        marker.ns = "planned_route"
        marker.id = drone_index
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.3  # line width, metres
        marker.color = self._color(drone_index)
        marker.points = [
            _latlon_to_local_xy(self.reference_lat, self.reference_lon, wp.lat, wp.lon)
            for wp in plan.waypoints
        ]
        # For a route point, set z to intended AGL altitude so the plan
        # renders at the height the drone will actually fly, not flat on
        # the ground.
        for pt, wp in zip(marker.points, plan.waypoints):
            pt.z = wp.alt_m

        array = MarkerArray()
        array.markers.append(marker)
        self._plan_pub.publish(array)

    def publish_position(self, drone_index: int, lat: float, lon: float, alt_m: float) -> None:
        pt = _latlon_to_local_xy(self.reference_lat, self.reference_lon, lat, lon)
        pt.z = alt_m

        trail = self._trails.setdefault(drone_index, [])
        trail.append(pt)
        if len(trail) > self._max_trail_points:
            trail.pop(0)

        sphere = Marker()
        sphere.header.frame_id = "map"
        sphere.header.stamp = self._node.get_clock().now().to_msg()
        sphere.ns = "live_position"
        sphere.id = drone_index
        sphere.type = Marker.SPHERE
        sphere.action = Marker.ADD
        sphere.pose.position = pt
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 1.0
        sphere.color = self._color(drone_index)

        trail_marker = Marker()
        trail_marker.header.frame_id = "map"
        trail_marker.header.stamp = sphere.header.stamp
        trail_marker.ns = "trail"
        trail_marker.id = drone_index
        trail_marker.type = Marker.LINE_STRIP
        trail_marker.action = Marker.ADD
        trail_marker.scale.x = 0.15
        color = self._color(drone_index)
        color.a = 0.5
        trail_marker.color = color
        trail_marker.points = list(trail)

        array = MarkerArray()
        array.markers.append(sphere)
        array.markers.append(trail_marker)
        self._pos_pub.publish(array)

    def shutdown(self) -> None:
        self._node.destroy_node()
