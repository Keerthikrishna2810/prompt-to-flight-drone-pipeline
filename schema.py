"""
Mission JSON schema — Day 2, Stage 1 (structural validation).

This defines the *shape* a mission plan must have: correct types, required
fields, and hard outer bounds (e.g. altitude can never exceed 30m no matter
what, regardless of any runtime safety config). Anything that fails to parse
here is rejected before Stage 2 (safety/sanity) ever runs.

Reference: the general "LLM emits structured intent, code enforces the
contract" pattern mirrors what ChatDrones (github.com/Gaurang-1402/ChatDrones)
does with its ROSGPT-style JSON command node, and what ros2-agent-ws
(github.com/limshoonkit/ros2-agent-ws) does when wrapping a locally-hosted
LLM. Only the pattern is reused here — this schema and validator are
written from scratch for this project's mission format.
"""

from enum import Enum
from typing import List
from pydantic import BaseModel, Field, model_validator


class MissionType(str, Enum):
    """Known command types. Anything outside this enum is rejected by
    Pydantic automatically — this is the 'known commands' guardrail."""
    LOOP = "loop"
    ROUTE = "route"


class Waypoint(BaseModel):
    lat: float = Field(ge=-90, le=90)
    lon: float = Field(ge=-180, le=180)
    alt_m: float = Field(ge=2, le=30, description="Altitude AGL in metres. Hard schema ceiling.")


class MissionPlan(BaseModel):
    mission_type: MissionType
    waypoints: List[Waypoint] = Field(min_length=1, max_length=50)
    repetitions: int = Field(default=1, ge=1, le=10)
    speed_mps: float = Field(ge=0.5, le=15, description="Hard schema ceiling — tightened further by Stage 2.")

    @model_validator(mode="after")
    def loop_needs_enough_points(self) -> "MissionPlan":
        if self.mission_type == MissionType.LOOP and len(self.waypoints) < 3:
            raise ValueError("mission_type 'loop' requires at least 3 waypoints to form a loop")
        return self
