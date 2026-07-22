# Challenge 1 -- Multi-Agent Formations

Extends the core prompt-to-flight pipeline (see [`README.md`](./README.md) /
[`WRITEUP.md`](./WRITEUP.md)) to control 2-6 simulated drones at once. Type
one instruction -- *"send three drones in a wedge down this route"* -- and
every drone gets its own validated, offset flight plan, then all of them
fly concurrently in one shared simulator.

See [`CHALLENGE_WRITEUP.md`](./CHALLENGE_WRITEUP.md) for how and why it's
built this way. This file just covers how to run it, start to finish, on a
plain Linux machine with nothing installed yet.

## 1. Installing Docker and Git

Skip this section if `git --version` and `docker --version` already work.
Otherwise (Ubuntu/Debian -- swap `apt-get` for `dnf` on Fedora/RHEL):

```bash
sudo apt-get update
sudo apt-get install -y git

curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

sudo usermod -aG docker $USER
newgrp docker
```

Check both work:

```bash
git --version
docker --version
```

If `docker --version` fails even after `newgrp docker`, log out and back in
-- the group permission needs a fresh session on some setups.

## 2. Get the code

```bash
git clone https://github.com/Keerthikrishna2810/prompt-to-flight-drone-pipeline.git
cd prompt-to-flight-drone-pipeline
```

## 3. Build the image

```bash
docker build -t drone-pipeline .
```

**This takes a while -- budget 30-45 minutes the first time.** It compiles
PX4 from scratch, installs Gazebo, installs ROS 2 (for the RViz view, see
step 6), and downloads the local AI model (~4.7GB) so later runs never need
network access for it. Rebuilds after this are fast -- Docker reuses
everything unless a file actually changed.

**If the build fails partway through with a network error** (DNS lookup
failures, `dial udp ... network is unreachable`, or similar), it's usually
Docker's IPv6 DNS resolution, not your actual internet connection. Fix it
once, machine-wide:

```bash
echo '{"dns": ["8.8.8.8", "1.1.1.1"], "ipv6": false}' | sudo tee /etc/docker/daemon.json
sudo systemctl restart docker
```

Then re-run `docker build` -- everything already completed is cached, so
this only re-attempts the step that failed, not the whole build.

**If the build fails with "no space left on device,"** free up old Docker
data first:

```bash
docker system prune -a --volumes
```

## 4. Run it, headless (no GUI, fastest way to confirm it works)

```bash
docker run -it drone-pipeline
```

You're now inside the container -- your prompt should change to something
like `root@<hex-id>:~/project#`. From there:

```bash
source fly_squad.sh "send three drones in a wedge down this route" 3
```

The second argument (`3`) is the drone count -- match it to however many
drones your prompt asks for. First run also starts Gazebo and 3 PX4
instances, which takes about a minute; you'll see progress messages the
whole way. At the end you should see all three drones' full audit logs
(arm, takeoff, each waypoint, RTL), each with genuinely different
coordinates -- that's the formation offset math working.

**Important: don't use `--rm` while you're still exploring.** `--rm`
deletes the container the moment you exit it, log files and all. Leave it
off until you're confident everything works; clean up old containers later
with `docker container prune`.

## 5. Run it with the Gazebo window visible

Headless mode above proves the pipeline works but shows nothing on screen.
To actually watch the drones:

```bash
xhost +local:docker
docker run -it \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e LIBGL_ALWAYS_SOFTWARE=1 \
  drone-pipeline
```

`LIBGL_ALWAYS_SOFTWARE=1` forces software rendering -- the safe default
unless you've separately set up NVIDIA GPU passthrough for Docker. Then,
inside the container, same command as before:

```bash
source fly_squad.sh "send three drones in a wedge down this route" 3
```

`fly_squad.sh` detects the forwarded display automatically and launches
Gazebo with its window instead of headless -- no flag needed.

## 6. Also watching it in RViz

Add `rviz` as a third argument:

```bash
source fly_squad.sh "send three drones in a wedge down this route" 3 rviz
```

This opens a second window (RViz2) showing each drone's planned route as a
colored line and its live position as a moving sphere, using
[`formation_viz.py`](./formation_viz.py). Requires the same `DISPLAY`
forwarding as step 5.

## 7. Prompts to try

| Prompt | What it exercises |
|---|---|
| `"send three drones in a wedge down this route"` | Formation mode -- all drones fly together, offset into a V |
| `"put two drones in a line abreast and patrol this loop"` | Formation mode, different shape, closed loop |
| `"split this route into 3 lanes and sweep the area"` | Split mode -- one route divided across drones, not offset together |
| `"send six drones down this route in a wedge"` | Rejected -- squad schema caps at 6 drones |

For a rejection that doesn't touch the AI at all:

```bash
python3 squad_demo_reject.py
```

Hands the squad safety checker two unsafe missions directly -- too many
drones, and drones placed closer together than a requested minimum
separation. Same result every time, no model involved.

## 8. Folder contents (squad-specific files)

```
formation.py              Pure geometry -- turns one shared route into per-drone offset routes
squad_schema.py            SquadPlan -- squad-level intent (drone count, formation, spacing)
squad_validator.py         Expands a SquadPlan into per-drone plans, validates every one
squad_interpreter.py       Turns a prompt into squad-level intent, using the AI
squad_executor.py          Flies all drones concurrently, one MAVSDK connection each
squad_main.py               Connects the AI step to the flying step, for a squad
fly_squad.sh                Runs a squad prompt, starting Ollama/Gazebo/PX4 first if needed
formation_viz.py            Optional: publishes routes + live positions to RViz
rviz/formation.rviz          Pre-configured RViz layout (Fixed Frame set to "map")
squad_demo_reject.py        Safety-check demo for squads, no AI involved
fixtures/valid_*.json        Sample squad missions used to test validation
test_formation.py, test_squad_*.py   Automated tests, run during the Docker build
CHALLENGE_WRITEUP.md          Architecture, decisions, and approach for the other two challenges
```

Every squad test file runs as part of the Docker build itself, same as the
core task's tests -- if the build finishes, they already passed.

## 9. Current limitations

- Formation alignment is fixed to the route's first-leg direction, not
  re-rotated through turns -- a wedge stays pointed one way for the whole
  mission.
- Minimum drone separation is checked before flight, not continuously
  enforced during it -- if a drone drifts off its planned path mid-flight,
  that's not caught live.
- `gz sim` worlds can start paused; `fly_squad.sh` sends an explicit
  unpause command to guard against this, but if drones still appear frozen
  in place (position/altitude not changing across log lines), check
  `gz_server.log` inside the container for the actual world name -- the
  unpause call assumes it's `default`.
- The AI runs on CPU, not GPU -- expect 20-60 seconds per squad
  interpretation, longer than the single-drone case since the prompt is
  larger.
