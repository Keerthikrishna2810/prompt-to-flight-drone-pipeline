# Extension Task Write-up

## Challenge 1 -- Multi-agent formations (built, tested, verified live)

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

**Verified live:** built the Docker image (PX4 v1.16.2 + Gazebo Harmonic +
Ollama + qwen2.5:7b-instruct, all baked into the image so `docker run`
needs no network for the model) and ran the full pipeline against 3 real
PX4 SITL instances sharing one Gazebo world. Prompt: *"send three drones
in a wedge down this route"*. The LLM interpreter produced valid squad
JSON on the first attempt, `squad_validator.py` accepted it, and
`squad_executor.py` connected to all 3 vehicles concurrently (ports
14540-14542), armed, took off, flew each drone's independently-offset
wedge route, and RTL'd -- all 3 audit logs show genuinely different
per-drone waypoints, confirming `formation.py`'s offset math is correct
against live vehicles, not just in the dry-run tests.

Two real infrastructure issues came up getting there, both now fixed in
`fly_squad.sh`:
1. PX4's `Tools/simulation/gz/simulation-gazebo` script reads a
   `--headless` CLI flag, not a `HEADLESS` environment variable -- the
   first version of the script got this wrong, which crashed Gazebo's Qt
   GUI against a nonexistent display and forced a slow internal restart.
   Fixed by using the correct flag, and by making the script auto-detect
   a live `DISPLAY` (so it runs the GUI when you actually want to watch,
   headless otherwise).
2. A `gz sim` world can start paused; PX4 will still connect, arm, and
   accept every command successfully against a paused world, but the
   simulated clock (and therefore vehicle position) never advances --
   commands report success while nothing visibly moves. Fixed with an
   explicit unpause call (`gz service ... --req 'pause: false'`) right
   after the world starts.

**69/69 unit/integration tests pass** (the original 22 plus 47 new), all
runnable with no live PX4/Gazebo/Ollama session -- the build-time-safe
pattern the core task established in its Dockerfile, which is what made
debugging the two live-only issues above fast: everything except the
final live-integration step was already known-good going in.

**Scope choices made on purpose, not oversights:**
- Formation alignment is fixed to the route's *first-leg* bearing, not
  re-rotated per leg -- a wedge stays pointed one way for the whole
  mission rather than pivoting through turns. Noted in `formation.py`'s
  docstring as the natural next step.
- Minimum separation is checked pre-flight (at validation time), not
  enforced as a live runtime guard during flight (e.g. if one drone drifts
  off course). That would need real-time cross-drone telemetry
  monitoring, a reasonable next addition.

**What's tested but not yet watched live:** `SquadMode.SPLIT` (splitting
one route into contiguous lanes across drones, for "sweep this area"
style instructions) exercises the exact same validated-plan ->
`fly_squad()` path as the wedge run above, and is covered by
`test_squad_fixtures.py`, but the live run above only demoed `formation`
mode. Likewise `formation_viz.py`'s RViz publishing is unit-tested
(`test_squad_executor.py`'s viz tests) and wired into `fly_squad.sh`, but
wasn't visually confirmed in an RViz window during this write-up.

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
