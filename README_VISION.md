# Challenge 3 -- Vision AI Target Detection + Follow

Extends the core pipeline with a search-then-follow behavior: patrol a
route looking for a user-configurable target, and once it's spotted,
save a snapshot and follow it. See [`CHALLENGE_WRITEUP.md`](./CHALLENGE_WRITEUP.md)
for the architecture and what's tested versus what's honestly unverified.

**Read this first:** the live camera + real detector path is the one
piece of this challenge not verified against a running simulator (no
GPU/display was available while building it -- see
`vision_camera_gz.py`'s own docstring). Everything else -- the
interpreter, the safety validator, the search/follow logic, the bounded
follow loop -- is fully tested and works with no live camera at all,
using a mock detector. Run the mock path first; treat the real camera
path as a stretch goal.

## 1-2. Docker/Git install, get the code, build the image

Same as [`README_SQUAD.md`](./README_SQUAD.md) sections 1-3 -- this is
the same image, one repo, one build. If you've already built it for
Challenge 1, you don't need to rebuild for this section unless you've
pulled a fresh copy of the Dockerfile.

## 3. Run it, mock path (no live camera, fully tested)

```bash
docker run -it drone-pipeline
```

Inside the container:

```bash
source fly_vision.sh "search this route and follow the first person you see"
```

This interprets the prompt, validates the mission, starts Gazebo + one
PX4 instance, flies the search route, and -- since no real camera is
attached in this mode -- always takes the "target not found" branch and
returns home. That's expected, not a bug: it proves every piece of the
pipeline except the literal act of seeing something.

## 4. Run it, real path (live camera + YOLO -- stretch goal, unverified)

```bash
source fly_vision.sh "search this route and follow the first person you see" real
```

The `real` argument switches to a camera-equipped vehicle model
(`gz_x500_mono_cam`), starts a ROS2 bridge for the camera topic, and
loads a real YOLO detector. If this doesn't work on the first try:

- Check `camera_bridge.log` and confirm the topic name the bridge is
  listening to actually matches what's running -- run `gz topic -l`
  yourself and compare against the hardcoded topic in `fly_vision.sh`.
- Check that `ros-humble-ros-gz-bridge`, `ros-humble-cv-bridge`, and
  `ultralytics` all installed successfully during the Docker build --
  those are in an explicitly optional Dockerfile block that's allowed to
  fail without breaking the rest of the image (see the Dockerfile's
  Challenge 3 section).
- If `PX4_SIM_MODEL=gz_x500_mono_cam` doesn't exist in your PX4
  checkout, list what camera-equipped variants actually are available:
  `ls /root/PX4-Autopilot/Tools/simulation/gz/models/` inside the
  container.

## 5. Prompts to try

| Prompt | What it exercises |
|---|---|
| `"search this route and follow the first person you see"` | Basic search + follow, target_class="person" |
| `"patrol this area looking for a car, follow it for up to 2 minutes if you find one"` | Configurable target class + custom max_follow_duration_s |
| `"look for a backpack and stay 15 meters away from it"` | Configurable target class + custom follow_distance_m |

For a rejection that doesn't touch the AI or a camera at all:

```bash
python3 vision_demo_reject.py
```

## 6. Folder contents (vision-specific files)

```
vision_schema.py              VisionFollowPlan -- target class, search route, follow parameters
vision_detector.py             Pluggable detector interface, MockDetector, optional real YoloDetector
vision_follow_controller.py     Pure geometry: bounding box -> steering command
vision_validator.py             Search route through the unchanged core safety pipeline
vision_interpreter.py           Prompt -> VisionFollowPlan, using the AI
vision_executor.py               Search + snapshot + bounded follow loop
vision_main.py                    Connects the AI step to the flying step
vision_camera_gz.py                Live Gazebo camera bridge -- see the "read this first" note above
vision_demo_reject.py             Safety-check demo, no AI or camera involved
fixtures/valid_vision_follow_person.json   Sample vision mission used to test validation
test_vision_*.py                    Automated tests, run during the Docker build
```

## 7. Current limitations

- Follow steering assumes the camera always faces geographic north --
  it does not track the vehicle's actual yaw/heading. Documented in
  `vision_executor.py`'s module docstring, with the direct fix noted
  (rotate the steering offset by live heading, the same technique
  `formation.py` uses for a route's bearing).
- Distance-to-target is estimated purely from bounding box size (no
  depth camera or stereo) -- accurate only insofar as the real target's
  size roughly matches the calibration constant in
  `vision_follow_controller.py`.
- The "send a picture to the operator" requirement is implemented as
  saving to a watched directory + a printed notification, not an actual
  push (email/Slack/etc) -- the interception point for adding that is
  isolated to one function (`_save_snapshot` in `vision_executor.py`).
