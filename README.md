# Prompt-to-Flight Drone Simulation Pipeline

Type a plain-English instruction. A local AI model turns it into a flight
plan. The plan is checked for safety before anything happens. Only then
does a simulated drone actually fly it.

See [`WRITEUP.md`](./WRITEUP.md) for the full explanation of how and why
it's built this way. This file just covers how to run it.

## Installing Docker and Git

If the machine doesn't already have these, here's how to get both (Ubuntu/
Debian steps below — swap `apt-get` for `dnf` on Fedora/RHEL):

```bash
sudo apt-get update
sudo apt-get install -y git

curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

sudo usermod -aG docker $USER
newgrp docker
```

Check both installed:

```bash
git --version
docker --version
```

If `docker --version` fails even after `newgrp docker`, log out and back
in — the group permission needs a fresh session on some setups.

## Running it

Get the code:

```bash
git clone https://github.com/Keerthikrishna2810/prompt-to-flight-drone-pipeline.git
cd prompt-to-flight-drone-pipeline
```

Build the image:

```bash
docker build -t drone-sim:day1 .
```

First build takes 15–20 minutes — it's compiling the flight software and
installing the simulator from scratch. Rebuilds after that are fast,
since Docker reuses what it already built.

Start the container:

```bash
docker run -it --rm --network host -v ollama_models:/root/.ollama drone-sim:day1 bash
```

Then run a prompt:

```bash
source /root/project/fly.sh "fly a small square patrol loop twice at 10 meters altitude"
```

The first run also downloads the AI model (a few GB, one-time, saved for
every run after). Run `fly.sh` again with a different prompt as many times
as you like in the same container.

## Prompts to try

| Prompt | Expected result |
|---|---|
| `"fly a small square patrol loop twice at 10 meters altitude"` | Flies a closed square, twice |
| `"fly a triangular patrol at 8 meters, repeat it 3 times"` | Different shape, same pattern |
| `"fly straight out about 50 meters to the northeast at 15 meters and stop there"` | One-way flight, not a loop |
| `"climb straight up to 500 meters and hover"` | Refused — too high |
| `"loop this patrol negative three times"` | Refused or self-corrected — see `WRITEUP.md` |

For a rejection that doesn't depend on the AI at all:

```bash
python3 /root/project/demo_reject.py
```

This hands the safety checker an out-of-bounds mission directly, skipping
the AI entirely. Same result every time.

## Watching the drone fly

The simulator runs headless by default — no GUI, faster, no graphics
driver dependency, which matters on an unfamiliar machine. To see it:

**Terminal 1:**
```bash
xhost +local:docker
docker run -it --rm --network host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ollama_models:/root/.ollama drone-sim:day1 bash
```
Inside: `ollama serve &`, wait a few seconds, then
`cd /root/PX4-Autopilot && HEADLESS=0 make px4_sitl gz_x500`, and leave it
running.

**Terminal 2**, once the simulator window appears:
```bash
docker exec -it $(docker ps -q --filter ancestor=drone-sim:day1) bash
python3 /root/project/main.py "<your prompt>"
```
Watch Terminal 1's window while Terminal 2 runs.

## Folder contents

Dockerfile                    Builds everything: flight software, simulator, AI model runner
fly.sh                         Runs a prompt, starting Ollama/PX4 first if needed
main.py                        Connects the AI step to the flying step
llm_interpreter.py              Turns a prompt into a flight plan, using the AI
mission_executor.py             Takes an approved flight plan and flies it
schema.py                       Checks a flight plan is well-formed
validator.py                    Checks a flight plan is actually safe to fly
demo_reject.py                   Safety-check demo, no AI involved
fixtures/                        Sample flight plans used to test the safety checks
test_*.py                        Automated tests, run during the Docker build
WRITEUP.md                       Architecture, decisions, and what's next

Every test file runs as part of the Docker build itself. If the build
finishes, the tests already passed.

## Current limitations

- Arrival confirmation sometimes appears to pause mid-flight then catch up.
  Missions still finish correctly, but the check isn't as reliable as
  intended.
- Return-to-launch altitude doesn't land exactly on the configured number
  every time — it's safer than the default, just not precise.
- The AI runs on CPU, not GPU — 20–45 seconds per response.
