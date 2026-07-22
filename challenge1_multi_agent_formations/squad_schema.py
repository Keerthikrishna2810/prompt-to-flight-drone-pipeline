"""
squad_schema.py

Challenge 1 -- the squad-level counterpart to schema.py's MissionPlan.

schema.py is left completely untouched on purpose (same "boring, never
touched once it works" principle the project already applies to
mission_executor.py). A SquadPlan is *not* a replacement for MissionPlan --
it's a level above it: squad-level intent (how many drones, what
formation, how far apart) plus one shared route, which formation.py then
expands into one ordinary MissionPlan-shaped route per drone. Everything
below that expansion point -- schema validation, safety validation,
execution -- is the exact same MissionPlan pipeline the core task already
built and tested.
"""

from enum import Enum
from typing import List
from pydantic import BaseModel, Field

from schema import MissionType, Waypoint
from formation import FormationType


class SquadMode(str, Enum):
    FORMATION = "formation"  # fly together, offset in a shape (line/wedge/column/box)
    SPLIT = "split"       # split one route into N lanes, one per drone (area sweep)


class SquadPlan(BaseModel):
    drone_count: int = Field(ge=2, le=6)
    mode: SquadMode
    formation: FormationType = Field(default=FormationType.LINE, description="Used when mode == 'formation'.")
    spacing_m: float = Field(default=8.0, ge=3, le=50, description="Minimum intended gap between drones, metres.")
    mission_type: MissionType
    waypoints: List[Waypoint] = Field(min_length=2, max_length=50, description="The shared/lead route.")
    repetitions: int = Field(default=1, ge=1, le=10)
    speed_mps: float = Field(ge=0.5, le=15)
