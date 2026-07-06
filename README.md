# Prompt-to-Flight Drone Simulation Pipeline

You type a plain-English instruction. A local AI model turns it into a
flight plan. That plan is checked for safety before anything happens. Only
then does a drone in a simulator actually fly it.

**For the full explanation of how and why it's built this way, see
[`WRITEUP.md`](./WRITEUP.md).** This file just covers how to run it.

## Before you start: installing Docker and Git

If the machine you're running this on doesn't already have Docker and Git
installed, here's how to get both (these steps are for Ubuntu/Debian-based
Linux, by far the most common case — if you're on Fedora/RHEL, swap
`apt-get` for `dnf`):

```bash
# Update package lists
sudo apt-get update

# Install Git
sudo apt-get install -y git

# Install Docker (official convenience script)
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Let your user run docker without typing sudo every time
sudo usermod -aG docker $USER
newgrp docker
```

Check both installed correctly:

```bash
git --version
docker --version
```

If `docker --version` doesn't work even after `newgrp docker`, log out and
log back in (or just restart the terminal) — the group permission needs a
fresh login session to take effect for some setups.

## How to run it

First, get the code onto the machine:

```bash
git clone https://github.com/<your-username>/<repo-name>.git
cd <repo-name>
```

Then build the image:

```bash
docker build -t drone-sim:day1 .
```

First build takes about 15–20 minutes (it's compiling the flight software
and installing the simulator from scratch). After that, rebuilds are fast —
seconds, not minutes — because Docker remembers what it already built.

Start the container:

```bash
docker run -it --rm --network host -v ollama_models:/root/.ollama drone-sim:day1 bash
```

Then, inside the container, run any prompt with one command:

```bash
source /root/project/fly.sh "fly a small square patrol loop twice at 10 meters altitude"
```

The first time you run this, it also downloads the AI model (a few GB,
one-time — it's saved and reused after that). You can run `fly.sh` again
with a different prompt as many times as you like in the same container.

## Prompts to try

| Prompt | What should happen |
|---|---|
| `"fly a small square patrol loop twice at 10 meters altitude"` | Drone flies a closed square, twice |
| `"fly a triangular patrol at 8 meters, repeat it 3 times"` | Different shape, works the same way |
| `"fly straight out about 50 meters to the northeast at 15 meters and stop there"` | A one-way flight, not a loop |
| `"climb straight up to 500 meters and hover"` | Refused — too high, never sent to the drone |
| `"loop this patrol negative three times"` | Refused — doesn't make sense as a number of repeats |

Want to see the safety check work without waiting on the AI at all? Run:

```bash
python3 /root/project/demo_reject.py
```

This skips the AI and hands the safety checker an obviously-too-far-away
mission directly. It always says no, instantly, the same way every time —
a clean way to prove the safety logic works on its own.

## Want to actually watch the drone fly?

By default the simulator runs invisibly in the background (this is on
purpose — it's faster and doesn't need graphics drivers set up, which
matters when running on someone else's machine for the first time). If you
want to see it:

**Terminal 1:**
```bash
xhost +local:docker
docker run -it --rm --network host \
  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ollama_models:/root/.ollama drone-sim:day1 bash
```
Inside: `ollama serve &`, wait a few seconds, then
`cd /root/PX4-Autopilot && HEADLESS=0 make px4_sitl gz_x500` and leave that
running.

**Terminal 2**, once a simulator window appears:
```bash
docker exec -it $(docker ps -q --filter ancestor=drone-sim:day1) bash
python3 /root/project/main.py "<your prompt>"
```
Watch Terminal 1's window while Terminal 2 runs.

## What's in this folder

```
Dockerfile                    Builds everything: flight software, simulator, AI model runner
fly.sh                         One command to run a prompt (starts everything it needs first)
main.py                        Ties the AI step and the flying step together
llm_interpreter.py              Turns your prompt into a flight plan, using the AI
mission_executor.py             Takes an approved flight plan and actually flies it
schema.py                       Checks a flight plan is well-formed
validator.py                    Checks a flight plan is actually safe to fly
demo_reject.py                   Instant safety-check demo, no AI needed
fixtures/                        Sample flight plans used to test the safety checks
test_*.py                        Automated tests — these run every time the image is built
WRITEUP.md                       Full write-up: how it works, what was solved, what's next
```

Every test file runs automatically while building the Docker image. If the
build finishes, the tests already passed — that's not just a claim, it's
checked every time.

## Known rough edges

Being upfront about a few things that aren't perfect (see `WRITEUP.md` for
the full detail):

- Sometimes the drone's position updates seem to pause briefly during a
  flight, then catch up. It doesn't stop the mission from completing
  correctly, but it's not as smooth as it should be.
- The "return home" altitude doesn't always land exactly on the number
  it's configured to use — it's safer than the default, just not
  perfectly precise.
- The AI runs on the processor, not the graphics card, so each response
  takes 20–45 seconds. It works, just not instantly.
