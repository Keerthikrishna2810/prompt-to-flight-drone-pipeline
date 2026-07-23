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

## Challenge 3 -- Vision AI target detection + follow (built, unit/integration tested; live camera unverified)

Same layering principle as Challenge 1: `schema.py`, `validator.py`,
`mission_executor.py` stay completely untouched.

- **vision_schema.py** -- `VisionFollowPlan`: target class (free text --
  this is what makes the target type "configurable by the user"), search
  route, follow distance/altitude, max follow duration, confidence
  threshold.
- **vision_detector.py** -- `TargetDetector` is a pluggable interface
  (`detect(frame) -> List[Detection]`), not a hardcoded dependency.
  `MockDetector` (scripted, deterministic) is what every other file's
  tests run against; `YoloDetector` is a real ultralytics-backed
  implementation, lazy-imported so the rest of the pipeline works and is
  fully testable with zero ML libraries installed -- same graceful-
  degradation pattern `formation_viz.py` uses for `rclpy`.
- **vision_follow_controller.py** -- pure geometry, no camera/drone
  dependency: converts a detection's normalized bounding box into a
  (forward, right) steering command, using box width as a depth proxy
  (a named, documented limitation -- see the file's own docstring --
  since a single 2D frame has no true depth information).
- **vision_validator.py** -- the search route goes through the
  unchanged `validate_mission_json()`, exactly like a normal mission.
- **vision_interpreter.py** -- same bounded-retry-with-feedback LLM
  pattern as `llm_interpreter.py`/`squad_interpreter.py`. The LLM only
  ever proposes target class + search route; it never computes
  follow-steering math.
- **vision_executor.py** -- flies the search route, polls the camera
  after each leg; on a qualifying detection, saves a snapshot (the
  "send a picture to the operator" requirement -- an MVP implementation
  that writes to a watched directory and prints a notification, with the
  actual push mechanism isolated to one function so swapping in email/
  Slack/webhook later doesn't touch anything else) and switches into a
  follow loop. Two safety properties worth calling out specifically:
  - The follow loop is a **provably bounded `for` loop** over a
    precomputed iteration count, not a `while True` guarded only by a
    timeout check -- a direct lesson from Challenge 1's own live
    debugging, where a timing assumption that quietly didn't hold turned
    into what looked like a hang. Here, even if every timing assumption
    inside the loop were wrong, it still physically cannot exceed
    `max_follow_duration_s / poll_interval_s` iterations.
  - Unlike Challenge 1's squad separation check (validated once,
    pre-flight), following a moving target needs a **live** safety
    guard: the follow loop checks distance-from-home against the
    geofence on every single iteration, not just at validation time,
    since the whole point of following is that the drone's position
    isn't pre-planned.
- **vision_main.py** -- full pipeline wiring, mirrors `main.py`/`squad_main.py`.

**37 new tests, all passing, all runnable with no live PX4/Gazebo/camera/
model** (98 total across the whole project now): follow-controller
geometry, detector scripting behavior, validator bounds, the full
search-found-follow / search-not-found / target-lost-mid-follow /
geofence-breach-mid-follow branches, the LLM retry loop, and a dedicated
test proving the follow loop's iteration count is bounded even when the
mocked detector reports a permanent detection.

**What's honestly unverified:** `vision_camera_gz.py` (the live Gazebo
camera bridge) was not run live -- no GPU/display/camera stream was
available while building this, the same constraint noted for Challenge
1's Gazebo spawn. Given today's real experience getting Challenge 1's
Gazebo integration working -- several genuine issues that each needed
live iteration to find and fix -- this file is explicitly scoped as a
stretch goal, not something a reviewer should expect to work on the
first try. `fly_vision.sh` supports running the entire tested pipeline
(interpretation, validation, search flight, RTL) without the live camera
at all, which still demonstrates every piece of Challenge 3's logic
except the literal act of a real detector seeing a real simulated
object.

**Riskiest remaining part**, unchanged from the original approach-only
assessment: getting a detector running reliably against actual simulated
camera frames without a lot of trial and error on model choice/
confidence thresholds. That risk hasn't gone away by building the rest
of the pipeline around it -- it's just now isolated to exactly one file
instead of blocking everything else.

