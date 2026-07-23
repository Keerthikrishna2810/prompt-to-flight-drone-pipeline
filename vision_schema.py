"""
vision_schema.py

Challenge 3 -- schema for a vision-follow mission: search a route for a
user-configurable target class, then follow it once found.

Same relationship to schema.py that squad_schema.py has: this is a level
ABOVE MissionPlan, not a replacement for it. The search phase reuses
Waypoint/MissionType directly; schema.py is untouched.
"""

from typing import List
from pydantic import BaseModel, Field

from schema import Waypoint


class VisionFollowPlan(BaseModel):
    target_class: str = Field(min_length=1, max_length=40,
                               description="What to look for, e.g. 'person', 'car', 'backpack' -- "
                                           "matched against whatever the configured detector supports.")
    search_waypoints: List[Waypoint] = Field(min_length=2, max_length=50,
                                              description="Patrol route to search while looking for the target.")
    search_speed_mps: float = Field(ge=0.5, le=8.0)
    follow_distance_m: float = Field(ge=3.0, le=30.0, default=8.0,
                                      description="Desired standoff distance from the target once locked on.")
    follow_altitude_m: float = Field(ge=3.0, le=25.0, default=10.0,
                                      description="Altitude to hold while following.")
    max_follow_duration_s: float = Field(ge=10.0, le=300.0, default=120.0,
                                          description="Give up following and RTL after this long, regardless of "
                                                       "whether the target is still in view.")
    detection_confidence_threshold: float = Field(ge=0.3, le=0.99, default=0.5)
