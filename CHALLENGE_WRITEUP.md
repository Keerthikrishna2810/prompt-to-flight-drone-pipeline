# Extension Task Write-up

## Challenge 1 -- Multi-agent formations (built, fully working, dry-run tested)

Approach: keep the entire core pipeline (schema.py -> validator.py ->
mission_executor.py) completely unchanged, and add a squad-level layer
above it.

- **formation.py** -- pure geometry. Given a shared route and a formation
  type (line / wedge / column / box), computes one offset route per drone,
  rotated to the route's initial bearing. No drone/LLM dependency, so it's
  testable with plain asserts (`test_formation.py`, 16 tests).
- **squad_schema.py** -- `SquadPlan`: squad-level intent (drone count,
  formation, spacing, one shared route). Sits next to `schema.py` without
  touching it.
- **squad_validator.py** -- expands a `SquadPlan` into N `MissionPlan`s via
  `formation.py`, then runs **every single one** through the existing
  `validate_mission_json()` unchanged, plus a new minimum-separation check
  across drones (`test_squad_validator.py`, 12 tests). A squad mission is
  refused outright, exactly like the core task's single-drone policy --
  nothing is auto-corrected.
- **squad_interpreter.py** -- same bounded-retry-with-feedback pattern as
  `llm_interpreter.py`. The LLM only ever proposes squad-level intent
  (formation, spacing, the shared route); it never computes an individual
  drone's offset -- that's deterministic code in `formation.py`. This is a
  stronger version of the core task's "LLM proposes, code disposes"
  principle (`test_squad_interpreter.py`, 5 tests, Ollama mocked).
- **squad_executor.py** -- runs N `MissionExecutor`s concurrently via
  `asyncio.gather`, one MAVSDK connection per drone (port `14540 + i`,
  matching PX4's own SITL instance-numbering convention). One drone's
  failure is isolated and reported, not silently swallowed. A bounded
  per-drone connect timeout (default 30s) means a missing/misconfigured PX4
  instance produces a clear error instead of hanging the whole squad
  forever (`test_squad_executor.py`, 8 tests, all dry-run).
- **squad_main.py** -- full pipeline wiring, mirrors `main.py`.
- **fly_squad.sh** -- multi-vehicle launch script, mirrors `fly.sh`'s
  idempotent startup pattern.
- **formation_viz.py** -- optional RViz publisher. Implements a two-method
  interface (`publish_plan`, `publish_position`) that `squad_executor.py`
  calls if (and only if) a viz object is supplied; any failure in it is
  caught and logged, never allowed to affect a live flight.

**69/69 tests pass** (the original 22 plus 47 new), all runnable with no
live PX4/Gazebo/Ollama session, same build-time-safe pattern the core task
already established in its Dockerfile.

**What's honestly unverified:** the multi-vehicle Gazebo spawn in
`fly_squad.sh` and the RViz config were written against PX4's documented
approach but not run live (no GPU/display available while building this).
Everything downstream of "N PX4 instances are up on the right ports" --
which is the part that actually matters for the multi-agent formations
challenge -- is fully tested. See `fly_squad.sh`'s own header comment for
the fallback if the Gazebo spawn needs adjusting for your PX4 checkout.

**Scope choices made on purpose, not oversights:**
- Formation alignment is fixed to the route's *first-leg* bearing, not
  re-rotated per leg -- a wedge stays pointed one way for the whole
  mission rather than pivoting through turns. Noted in `formation.py`'s
  docstring as the natural next step.
- Minimum separation is checked pre-flight (at validation time), not
  enforced as a live runtime guard during flight (e.g. if one drone drifts
  off course). That would need real-time cross-drone telemetry
  monitoring, a reasonable next addition.

## Challenge 2 -- SLAM / autonomous navigation (approach only)

This is the biggest lift of the three, mainly because it's a different
kind of problem from the other two: the core task's pipeline assumes
known, LLM-given waypoints; SLAM means the vehicle doesn't know the map
yet.

Approach if building this next:
1. Add a lidar (2D is enough for a first pass) to the x500 model in
   Gazebo, publishing a `LaserScan`-equivalent.
2. Bring in `slam_toolbox` (ROS 2) for online occupancy-grid SLAM, fed by
   that lidar plus PX4's own odometry (via the uXRCE-DDS bridge -- the
   piece deliberately avoided for Challenge 1's RViz visualization, since
   here it's actually load-bearing, not optional).
3. Replace the "fly to LLM-given absolute waypoints" step with "fly toward
   an LLM-given *region or heading*, using a frontier-exploration planner
   over the live occupancy grid" for the unknown parts of the mission --
   waypoints inside already-known space still go straight through the
   existing `validate_mission_json()` geofence/altitude checks unchanged.
4. The safety validator's job barely changes in spirit: it still refuses
   anything outside the geofence or altitude bounds -- it just validates
   against the live map's known-free-space mask in addition to the static
   checks it already does.

Riskiest part: the PX4<->ROS2 DDS bridge itself (`uxrce_dds_agent`) is a
real extra moving part with its own failure modes, which is exactly why
it wasn't pulled in for Challenge 1's lighter-weight visualization need.

## Challenge 3 -- Vision AI target detection + follow (approach only)

Approach if building this next:
1. Add a camera sensor to the x500 model in Gazebo (SDF plugin), publishing
   frames over Gazebo's own transport (or bridged to ROS 2 `Image` msgs).
2. Run a lightweight open-vocabulary or class-based detector (e.g.
   YOLO-family or a small open-vocabulary model) on each frame,
   parameterized by a user-configurable target class/description string --
   this is what makes the target type "configurable by the user" per the
   prompt.
3. On a positive detection: (a) save the frame and send it to the operator
   -- simplest version is writing it to a watched output directory /
   posting to a local webhook; (b) switch `MissionExecutor` from
   "fly the validated waypoint list" mode into a closed-loop "servo toward
   detection bounding-box centroid" mode, using `goto_location` calls
   computed from the target's estimated bearing/range rather than a
   pre-planned waypoint.
4. Safety-wise, this needs a maximum-follow-distance guard (don't chase a
   target outside the geofence) and a "lost target" timeout that returns
   to the last validated waypoint or RTLs -- both straightforward
   extensions of the existing `SafetyConfig`/`MissionExecutor` shape.

Riskiest part: getting a detector running reliably against simulated
camera frames without a lot of trial and error on model choice / confidence
thresholds -- the part most likely to need a longer debugging session than
Challenges 1 or 2.
