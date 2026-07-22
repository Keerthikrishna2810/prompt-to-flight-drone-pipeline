"""
formation.py

Challenge 1 (multi-agent formations) -- pure geometry, zero drone
dependencies.

Same design principle as mission_executor.py's re-homing math: this file
only ever knows about metres, bearings, and lat/lon. It has never heard of
MAVSDK, PX4, or an LLM. That's deliberate -- it means every function here
is testable with plain asserts, no simulator, no mocked network call, same
as _offset_from_reference()/_apply_offset() in mission_executor.py.

What this module does:
    a shared "lead" route (list of Waypoint, exactly what a single-drone
    MissionPlan already uses) + a formation type + a drone count
        -> one offset route per drone, each still a valid list of Waypoint

The offsets are expressed in a drone-relative (forward, right) frame --
forward being the direction of travel on the route's first leg -- then
rotated once into world (north, east) metres using that leg's bearing.
Rotating once, rather than re-rotating per leg, is a deliberate scope
decision: it keeps a wedge pointing in a fixed compass direction for the
whole mission rather than continuously re-orienting around every turn.
Good enough to sweep a route in a wedge or send a squad down a corridor in
formation; a per-leg-rotating formation (so the wedge visibly pivots
through turns) is listed as follow-up work rather than built here -- see
WRITEUP.md Section 3.
"""

import math
from enum import Enum
from typing import List, Tuple

from schema import Waypoint

_EARTH_RADIUS_M = 6371000.0


class FormationType(str, Enum):
    LINE = "line"       # side by side, perpendicular to travel
    WEDGE = "wedge"      # V shape, point facing direction of travel
    COLUMN = "column"     # nose to tail, along direction of travel
    BOX = "box"         # 2x2 (or best fit) grid, for 4+ drones


def _bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial compass bearing (0=N, 90=E) from point 1 to point 2."""
    p1, p2 = math.radians(lat1), math.radians(lat2)
    dlon = math.radians(lon2 - lon1)
    x = math.sin(dlon) * math.cos(p2)
    y = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dlon)
    return math.degrees(math.atan2(x, y)) % 360


def _offset_from_reference(ref_lat: float, ref_lon: float, lat: float, lon: float) -> Tuple[float, float]:
    """Duplicated from mission_executor.py on purpose -- same reason that
    file duplicates validator.py's haversine rather than importing it:
    formation.py's only real contract is schema.py's Waypoint, not
    mission_executor internals."""
    d_lat = math.radians(lat - ref_lat)
    d_lon = math.radians(lon - ref_lon)
    north_m = d_lat * _EARTH_RADIUS_M
    east_m = d_lon * _EARTH_RADIUS_M * math.cos(math.radians(ref_lat))
    return north_m, east_m


def _apply_offset(base_lat: float, base_lon: float, north_m: float, east_m: float) -> Tuple[float, float]:
    d_lat = north_m / _EARTH_RADIUS_M
    d_lon = east_m / (_EARTH_RADIUS_M * math.cos(math.radians(base_lat)))
    return base_lat + math.degrees(d_lat), base_lon + math.degrees(d_lon)


def _rotate_forward_right_to_north_east(forward_m: float, right_m: float, bearing_deg: float) -> Tuple[float, float]:
    """Rotate a (forward, right) offset -- forward along the direction of
    travel, right perpendicular to it -- into (north, east) world metres,
    given the bearing (compass degrees) that 'forward' points along."""
    theta = math.radians(bearing_deg)
    north_m = forward_m * math.cos(theta) - right_m * math.sin(theta)
    east_m = forward_m * math.sin(theta) + right_m * math.cos(theta)
    return north_m, east_m


def slot_offsets(formation: FormationType, drone_count: int, spacing_m: float) -> List[Tuple[float, float]]:
    """Returns one (forward_m, right_m) slot per drone, index 0 first,
    relative to the squad's lead position (drone 0's route is the
    unmodified shared route -- it flies exactly where the LLM/route said).

    Values are in the drone-relative (forward, right) frame described at
    the top of this file -- caller rotates them into world metres with
    _rotate_forward_right_to_north_east() using the route's own bearing.
    """
    if drone_count < 2:
        raise ValueError("a formation needs at least 2 drones")

    if formation == FormationType.LINE:
        # Side by side, centred on the lead: for N drones, offsets are
        # symmetric about 0 along "right", e.g. N=3 -> [-spacing, 0, +spacing]
        mid = (drone_count - 1) / 2.0
        return [(0.0, (i - mid) * spacing_m) for i in range(drone_count)]

    if formation == FormationType.COLUMN:
        # Nose to tail, drone 0 leads, each following one further back.
        return [(-i * spacing_m, 0.0) for i in range(drone_count)]

    if formation == FormationType.WEDGE:
        # Point (drone 0) leads, wings fall back and out on alternating
        # sides -- classic V, e.g. drone1 back-left, drone2 back-right.
        offsets = [(0.0, 0.0)]
        for i in range(1, drone_count):
            rank = (i + 1) // 2          # 1,1,2,2,3,3... how far back this pair sits
            side = -1 if i % 2 == 1 else 1  # alternate left(-)/right(+)
            offsets.append((-rank * spacing_m, side * rank * spacing_m))
        return offsets

    if formation == FormationType.BOX:
        # Grid, filled row-major, roughly square: e.g. 4 -> 2x2, 6 -> 2x3.
        cols = math.ceil(math.sqrt(drone_count))
        offsets = []
        for i in range(drone_count):
            row, col = divmod(i, cols)
            offsets.append((-row * spacing_m, col * spacing_m))
        return offsets

    raise ValueError(f"unknown formation type: {formation}")


def apply_formation(
    waypoints: List[Waypoint],
    formation: FormationType,
    drone_count: int,
    spacing_m: float,
) -> List[List[Waypoint]]:
    """The main entry point: shared route + formation -> one route per
    drone. Every drone flies the same shape, offset by its formation slot,
    with the whole squad's alignment fixed to the bearing of the route's
    first leg. Altitude is left untouched per waypoint (formations here
    are horizontal; per-drone altitude separation is a validator concern,
    see squad_validator.py)."""
    if len(waypoints) < 2:
        raise ValueError("a route needs at least 2 waypoints to have a direction of travel")

    ref_lat, ref_lon = waypoints[0].lat, waypoints[0].lon
    bearing = _bearing_deg(waypoints[0].lat, waypoints[0].lon, waypoints[1].lat, waypoints[1].lon)
    slots = slot_offsets(formation, drone_count, spacing_m)

    per_drone_routes: List[List[Waypoint]] = []
    for forward_m, right_m in slots:
        north_m, east_m = _rotate_forward_right_to_north_east(forward_m, right_m, bearing)
        route: List[Waypoint] = []
        for wp in waypoints:
            wp_north, wp_east = _offset_from_reference(ref_lat, ref_lon, wp.lat, wp.lon)
            lat, lon = _apply_offset(ref_lat, ref_lon, wp_north + north_m, wp_east + east_m)
            route.append(Waypoint(lat=lat, lon=lon, alt_m=wp.alt_m))
        per_drone_routes.append(route)
    return per_drone_routes


def split_route_across_drones(waypoints: List[Waypoint], drone_count: int) -> List[List[Waypoint]]:
    """The other squad pattern: not 'fly together in formation' but 'split
    one long route into N lanes, one per drone' -- e.g. 'sweep this area,
    split into 3 lanes'. Splits the waypoint list into drone_count
    contiguous, roughly-equal chunks, each chunk sharing its boundary
    waypoint with the next so there's no gap in area coverage."""
    if drone_count < 2:
        raise ValueError("splitting a route needs at least 2 drones")
    n = len(waypoints)
    if n < drone_count + 1:
        raise ValueError(
            f"route has {n} waypoints, too few to split across {drone_count} drones "
            f"(need at least {drone_count + 1})"
        )

    # Evenly-spaced cut points across the route, each lane overlapping its
    # neighbour by exactly one shared waypoint (the seam).
    lanes: List[List[Waypoint]] = []
    seg_len = (n - 1) / drone_count
    for i in range(drone_count):
        start = round(i * seg_len)
        end = round((i + 1) * seg_len)
        lane = waypoints[start:end + 1]
        if len(lane) < 2:
            lane = waypoints[start:start + 2]
        lanes.append(lane)
    return lanes
