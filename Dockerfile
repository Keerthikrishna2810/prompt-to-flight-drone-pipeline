# =============================================================================
# Prompt -> LLM -> Validated JSON -> Executor -> PX4 SITL / Gazebo Harmonic
# Day 1 base image: PX4 SITL + Gazebo Harmonic + MAVSDK-Python + Ollama
#
# Sources / citations:
#   - PX4-Autopilot   https://github.com/PX4/PX4-Autopilot        (BSD-3-Clause)
#   - MAVSDK-Python   https://github.com/mavlink/MAVSDK-Python     (MIT)
#   - Ollama           https://github.com/ollama/ollama             (MIT)
#   - Gazebo Harmonic  https://gazebosim.org                        (Apache-2.0)
# =============================================================================
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC

# Make apt resilient to transient network drops -- ubuntu.sh below pulls a
# large dependency list (Gazebo Harmonic + PX4 build deps) over a long apt
# run, and a single dropped connection mid-fetch otherwise fails the whole
# build step from scratch. Retries + longer timeout cost nothing on a
# healthy connection.
RUN echo 'Acquire::Retries "5";' > /etc/apt/apt.conf.d/80-retries \
    && echo 'Acquire::http::Timeout "30";' >> /etc/apt/apt.conf.d/80-retries \
    && echo 'Acquire::https::Timeout "30";' >> /etc/apt/apt.conf.d/80-retries

# ---- Base tooling --------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        git \
        wget \
        curl \
        sudo \
        lsb-release \
        gnupg2 \
        ca-certificates \
        zstd \
        python3 \
        python3-pip \
        python3-venv \
        locales \
    && locale-gen en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# ---- PX4-Autopilot (pinned release for reproducibility) ------------------
# v1.16.2 is the latest fully-stable tagged release at the time of writing
# and has mature Gazebo Harmonic (gz sim) integration via `make px4_sitl gz_x500`.
ENV PX4_VERSION=v1.16.2
ENV PX4_HOME=/root/PX4-Autopilot

# Fail fast instead of hanging forever if the connection stalls mid-clone
# (git has no default timeout -- without this, a dead connection just sits
# there with no way to tell "stalled" from "slow"). Aborts if the transfer
# drops below 1000 bytes/sec for 60+ seconds straight.
RUN git config --global http.lowSpeedLimit 1000 \
    && git config --global http.lowSpeedTime 60

RUN git clone --progress --branch ${PX4_VERSION} --recursive --depth 1 \
        https://github.com/PX4/PX4-Autopilot.git ${PX4_HOME}

WORKDIR ${PX4_HOME}

# PX4's own setup script installs build deps + Gazebo Harmonic from the
# osrfoundation apt repo. --no-nuttx skips the (irrelevant) hardware FMU toolchain.
# NOTE: this step needs internet access to packages.osrfoundation.org and
# apt.gazebosim.org -- if your build environment blocks those, install
# Gazebo Harmonic manually per https://gazebosim.org/docs/harmonic/install_ubuntu
RUN bash ./Tools/setup/ubuntu.sh --no-nuttx

# Compile the SITL firmware once at build time so `docker run` later only
# needs to relink, not rebuild from scratch.
#
# IMPORTANT: `make px4_sitl gz_x500` is a build-AND-launch combo target --
# it compiles, then drops into an interactive PX4 shell (`pxh>`) waiting for
# commands. That hangs forever inside `docker build` (no stdin attached).
# `DONT_RUN=1` does not reliably suppress this for the gz_x500 combo target,
# so we build the firmware directly instead. The vehicle model (x500) is
# selected at *runtime*, not compile time, so this still fully warms the
# cache for the `gz_x500` run in run_smoke_test.sh.
RUN make px4_sitl_default

# ---- MAVSDK-Python ---------------------------------------------------------
RUN pip3 install --no-cache-dir mavsdk pydantic pytest requests

# ---- Ollama -----------------------------------------------------------------
RUN curl -fsSL https://ollama.com/install.sh | sh

# ---- Project files ----------------------------------------------------------
# Everything below this line changes often during development. Keeping it
# at the bottom of the file means edits here never invalidate the expensive
# PX4/Gazebo/Ollama layers above -- rebuilds after a code change are seconds,
# not tens of minutes.
WORKDIR /root/project

# Day 1
COPY smoke_test.py /root/project/smoke_test.py
COPY run_smoke_test.sh /root/project/run_smoke_test.sh
RUN chmod +x /root/project/run_smoke_test.sh

# Day 2 -- mission JSON schema + safety validator + fixtures
COPY schema.py /root/project/schema.py
COPY validator.py /root/project/validator.py
COPY test_validator.py /root/project/test_validator.py
COPY fixtures/ /root/project/fixtures/

# Build-time sanity check: if the validator doesn't pass its own fixtures,
# fail the build here rather than discovering it during the live demo.
RUN cd /root/project && python3 test_validator.py

# Day 3 -- deterministic mission executor
COPY mission_executor.py /root/project/mission_executor.py
COPY test_executor.py /root/project/test_executor.py

# Executor logic tests run entirely in dry-run mode (no live PX4/Gazebo
# needed), so this can be a build-time check too, same as Day 2's.
RUN cd /root/project && python3 test_executor.py

# Day 4 -- LLM interpreter (Ollama-backed prompt -> JSON draft)
COPY llm_interpreter.py /root/project/llm_interpreter.py
COPY test_llm_interpreter.py /root/project/test_llm_interpreter.py
COPY start_ollama.sh /root/project/start_ollama.sh
RUN chmod +x /root/project/start_ollama.sh

# These tests mock Ollama entirely (no live model needed), so they're
# build-time safe too -- proves the retry/validation logic before it's
# ever pointed at something as unpredictable as a real local LLM.
RUN cd /root/project && python3 test_llm_interpreter.py

# Day 4 completion -- full pipeline connector (prompt -> interpreter -> executor)
COPY main.py /root/project/main.py
COPY test_main.py /root/project/test_main.py
RUN cd /root/project && python3 test_main.py

# Single-command launcher: idempotent Ollama/PX4 startup + run
COPY fly.sh /root/project/fly.sh
RUN chmod +x /root/project/fly.sh

# Deterministic validator-rejection demo (no LLM involved) -- also serves
# as a build-time regression check: if the safety validator ever starts
# wrongly accepting an out-of-bounds mission, the build itself fails here.
COPY demo_reject.py /root/project/demo_reject.py
RUN cd /root/project && python3 demo_reject.py

# ---- Challenge 1 -- multi-agent formations ---------------------------------
# Same "boring, unchanged core" principle as everything above: schema.py,
# validator.py, mission_executor.py, llm_interpreter.py, main.py are not
# touched by any file below. This adds a squad-level layer ON TOP of them:
#   formation.py         -- pure geometry (line/wedge/column/box), no drone deps
#   squad_schema.py       -- SquadPlan, the squad-level counterpart to MissionPlan
#   squad_validator.py    -- expands a SquadPlan into N MissionPlans, runs each
#                            through the unchanged single-drone pipeline, plus
#                            a new minimum-separation check across drones
#   squad_interpreter.py  -- same retry-with-feedback LLM pattern as
#                            llm_interpreter.py, scoped to squad-level intent
#   squad_executor.py     -- N MissionExecutors run concurrently, one per drone,
#                            with a bounded per-drone connect timeout so one
#                            stuck connection can never hang the whole squad
#   squad_main.py          -- full pipeline wiring, mirrors main.py
COPY formation.py /root/project/formation.py
COPY squad_schema.py /root/project/squad_schema.py
COPY squad_validator.py /root/project/squad_validator.py
COPY squad_interpreter.py /root/project/squad_interpreter.py
COPY squad_executor.py /root/project/squad_executor.py
COPY squad_main.py /root/project/squad_main.py
COPY test_formation.py /root/project/test_formation.py
COPY test_squad_validator.py /root/project/test_squad_validator.py
COPY test_squad_interpreter.py /root/project/test_squad_interpreter.py
COPY test_squad_executor.py /root/project/test_squad_executor.py
COPY test_squad_main.py /root/project/test_squad_main.py
COPY test_squad_fixtures.py /root/project/test_squad_fixtures.py
COPY squad_demo_reject.py /root/project/squad_demo_reject.py
COPY fixtures/ /root/project/fixtures/

# Every one of these tests is pure Python, mocked-LLM, or dry-run --
# exactly like Day 2-4's tests -- so all of it is build-time safe with no
# live PX4/Gazebo/Ollama needed. If squad geometry, validation, or the
# retry loop is ever broken, the build fails here instead of during a
# live multi-vehicle demo.
RUN cd /root/project \
    && python3 test_formation.py \
    && python3 test_squad_validator.py \
    && python3 test_squad_interpreter.py \
    && python3 test_squad_executor.py \
    && python3 test_squad_main.py \
    && python3 test_squad_fixtures.py \
    && python3 squad_demo_reject.py

COPY fly_squad.sh /root/project/fly_squad.sh
RUN chmod +x /root/project/fly_squad.sh

# ---- Optional -- ROS 2 Humble + RViz2, for watching the squad in RViz -----
# NOT required for the squad pipeline itself -- Gazebo already shows every
# simulated drone with zero extra setup (HEADLESS=0 in fly_squad.sh), same
# as the core task's fly.sh. This block exists purely so formation_viz.py
# (squad_executor.py's optional --rviz flag) has somewhere to publish to.
#
# If this block causes build trouble or costs too much time, it's safe to
# comment out entirely: squad_executor.py's viz hook is designed to no-op
# cleanly with a warning if formation_viz.py/rclpy can't be imported (see
# squad_executor.py's run_squad_from_file / squad_main.py's use_rviz
# handling) -- nothing about flying the squad depends on this succeeding.
RUN apt-get update && apt-get install -y --no-install-recommends curl gnupg lsb-release \
    && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
        http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-humble-ros-base ros-humble-rviz2 python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

COPY formation_viz.py /root/project/formation_viz.py
COPY rviz/ /root/project/rviz/

# Pull the local LLM model at build time so `docker run` doesn't need to
# re-download it. Swap for whatever model you settle on.
# (Left commented out for Day 1 -- uncomment once Ollama step is confirmed working,
#  it adds several GB to the image.)
# RUN (ollama serve &) && sleep 5 && ollama pull qwen2.5:7b-instruct

CMD ["/bin/bash"]
