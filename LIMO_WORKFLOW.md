# LIMO Workflow — Run RoverMind on the Rover

> Last verified: 2026-05-26 (connection details superseded by `context/ENVIRONMENT.md`)

How to SSH into the LIMO Agilex Pro and bring up the RoverMind stack.

---

## 0. Before doing anything: find the rover on the current network

The IP changes every session; **do not hardcode it.** Connection identity (the rover's
stable MAC, the network profiles, and the arp-by-MAC discovery to get the current IP) now
lives in one place: **`context/ENVIRONMENT.md`**. Find the IP there, then come back here for
bring-up. `<LIMO_IP>` below means "the IP you discovered".

Quick version: run **`scripts/rover_connect.sh`** — it finds the rover by MAC, identifies the
active network profile, stamps `context/ENVIRONMENT.md`, and prints the tunnel command
(`--open` also opens it). Manual fallback: `arp -an | grep -i 54:ef:33:9e:e7:71` → the rover's
current IP (populate arp first with a subnet ping sweep if empty — see `context/ENVIRONMENT.md`).
Confirm with `ssh agilex@<LIMO_IP> 'hostname'` → `master`.

---

## 1. One-time setup (already done — for reference only)

Done on 2026-05-26. Skip this section if SSH key auth still works.

> **Rover re-imaged / fresh?** If `ros2 pkg list` shows only `limo_bringup` (no
> `safety_controller_*`) or `~/RoverMind` / `~/rovermind_ws` are missing, the
> LIMO was re-imaged and this setup is gone. Follow
> [`workflows/ROVER_BOOTSTRAP.md`](workflows/ROVER_BOOTSTRAP.md) to detect it and
> rebuild from scratch (no car movement) before §2.

### 1a. Key-based SSH (from your Mac)

```bash
ssh-copy-id agilex@<LIMO_IP>      # prompts for the rover password once
```

After this, `ssh agilex@<LIMO_IP>` should not ask for a password.

### 1b. Repo + workspace layout on the rover

The repo's two ROS2 packages live under `~/RoverMind/` but the **root**
of that repo is itself the `safety_controller_layer` package, which makes
`colcon` stop recursing and miss `safety_controller_layer_interfaces/`.
Workaround: symlink each package into a separate colcon workspace.

```bash
# On the rover:
cd ~
git clone https://github.com/natecl/RoverMind.git
mkdir -p ~/rovermind_ws/src
ln -sfn ~/RoverMind                              ~/rovermind_ws/src/safety_controller_layer
ln -sfn ~/RoverMind/safety_controller_layer_interfaces \
                                                 ~/rovermind_ws/src/safety_controller_layer_interfaces

# Build:
source /opt/ros/foxy/setup.bash
source ~/limo_ros2_ws/install/setup.bash         # for limo_bringup
cd ~/rovermind_ws
colcon build --symlink-install

# Python deps for the agent (see Known issues below — this currently fails on Python 3.8):
cd ~/RoverMind
pip3 install -r requirements.txt --break-system-packages
```

`colcon list` from `~/rovermind_ws` should show:

```
safety_controller_layer            src/safety_controller_layer            (ros.ament_python)
safety_controller_layer_interfaces src/safety_controller_layer_interfaces (ros.ament_cmake)
```

---

## 2. Every-session workflow

### 2a. Connect

```bash
# On your Mac (use the IP you confirmed in step 0):
ssh -L 9000:localhost:9000 agilex@<LIMO_IP>
```

The `-L 9000:localhost:9000` flag forwards Mac-side `localhost:9000` to the rover's `127.0.0.1:9000`, where the bridge (Terminal 3 below) will listen.

**The rover's `~/.bashrc` prompts on every interactive login:**

```
ros:noetic(1) foxy(2)  ?
```

**Type `2` and press Enter** to source ROS2 Foxy + `~/limo_ros2_ws`.
(Option `1` sources ROS1 Noetic + `~/agilex_ws/devel` — not what RoverMind
uses.) Do this in every SSH terminal you open.

Confirm you're on the right host:

```bash
hostname        # should print: master
hostname -I     # should match the IP you used
echo $ROS_DISTRO  # should print: foxy
```

### 2b. Add the RoverMind overlay

The `bashrc` prompt sources Foxy + `limo_ros2_ws` but not our overlay.
Run this in every terminal **after** answering `2`:

```bash
source ~/rovermind_ws/install/setup.bash
```

> **Non-interactive SSH note:** if you run `ssh agilex@<LIMO_IP> '<cmd>'`
> from your Mac, the `bashrc` prompt is skipped (the `read` gets no input
> and falls through the `case`), so nothing gets sourced. In that case
> you have to source all three manually:
>
> ```bash
> ssh agilex@<LIMO_IP> 'bash -lc "
>   source /opt/ros/foxy/setup.bash &&
>   source ~/limo_ros2_ws/install/setup.bash &&
>   source ~/rovermind_ws/install/setup.bash &&
>   <your command>"'
> ```

Quick sanity check:

```bash
ros2 pkg list | grep -E 'safety_controller|limo_bringup'
# expect: limo_bringup, safety_controller_layer, safety_controller_layer_interfaces
ros2 pkg executables safety_controller_layer
# expect: aeb_node, safety_controller_node
```

### 2c. Sync the repo from your Mac

Develop on the Mac, then push the whole repo onto the rover before bring-up.
Sync the **entire** tree — the bridge imports from `agent/`, `perception/`, and
`config/`, so a partial copy of just `bridge/` would break it.

```bash
# On your Mac (use the IP you confirmed in step 0):
rsync -av \
  --exclude '.git' --exclude '.venv' --exclude '__pycache__' \
  --exclude '*.pyc' --exclude '.pytest_cache' \
  ~/code/RoverMind/ agilex@<LIMO_IP>:~/RoverMind/
```

The colcon workspace `~/rovermind_ws` symlinks into `~/RoverMind` (§1b) and was
built with `--symlink-install`, so **Python edits are live without a rebuild**.
Rebuild only if you changed the `ExecuteCommand` action interface or added/moved
a package (the ament_cmake interfaces package is not symlinked):

```bash
# On the rover, only when interfaces/package structure changed:
cd ~/rovermind_ws && colcon build --symlink-install
```

### 2d. Bring up the stack (three terminals — three SSH sessions)

> **Tip:** open three SSH sessions to the rover, or use `tmux` / `screen`.
> Each command below blocks; do not background them.

**Terminal 1 — LIMO base drivers**

```bash
ros2 launch limo_bringup limo_start.launch.py
```

**Terminal 2 — RoverMind safety controller + emergency braking gate**

```bash
ros2 launch safety_controller_layer rovermind.launch.py
# bring-up debugging without the braking gate:
ros2 launch safety_controller_layer rovermind.launch.py use_aeb:=false
```

Expect to see:

```
[safety_controller_node-1] [INFO] [...] [safety_controller]:
    SafetyControllerNode up; ExecuteCommand action server ready on 'execute_command'.
```

**Terminal 3 — RoverMind Python 3.8 bridge**

```bash
# Must source ROS + overlay first (same as 2b), then:
python3.8 ~/RoverMind/bridge/bridge_server.py --bind 127.0.0.1:9000
```

Expect:

```
[bridge] listening on tcp://127.0.0.1:9000
```

The bridge owns the rover's rclpy ActionClient and Moondream client. The first
`capture_and_analyze` request loads Moondream onto the Jetson GPU (5–15 s); subsequent
requests are sub-second.

### 2e. Preflight checks — verify before you let it drive

`run_agent.py` issues an autonomous *physical drive*, and the end-to-end path is
still an unverified manual gate (§3). Before running it, confirm the rover has
everything the agent needs. Run these in an SSH terminal with the overlay sourced
(§2b), once all three terminals above are up.

```bash
# Nodes alive (expect /safety_controller and /emergency_brake)
ros2 node list

# Topics present (expect /scan /imu /odom /cmd_vel /cmd_vel_raw)
ros2 topic list

# LiDAR actually publishing -- a stale/missing /scan makes the AEB brake
# (scan_timeout_s = 1.0s). Expect a steady rate and populated ranges.
ros2 topic hz /scan
ros2 topic echo /scan --once

# Controller sensors -- missing /imu or /odom aborts a goal mid-maneuver
# with a RuntimeError. Expect steady rates.
ros2 topic hz /imu
ros2 topic hz /odom

# Action server up (expect /execute_command)
ros2 action list
ros2 action info /execute_command

# Bridge listening on the rover (Terminal 3)
ss -ltn | grep 9000
```

**Go / no-go safety gates** (the rover moves at up to 0.3 m/s and turns under its
own control):

- [ ] AEB is on — Terminal 2 was launched **without** `use_aeb:=false`. Never
      smoke-test the agent with the braking gate off; it would drive blind.
- [ ] Floor-level, open area, clear of stairs and table/ledge edges. The LiDAR
      scans a horizontal plane and **cannot see drop-offs** — it will drive off
      a ledge without braking.
- [ ] Nothing within ~0.5 m of the front arc at start (AEB trips at 0.40 m), and
      a clear path for the first maneuver. Note the AEB brakes forward motion
      only — rotation and reverse still pass through, so it can swing into a
      close obstacle while turning.
- [ ] A person is ready to Ctrl-C (or lift / power-off) the rover at any moment.
- [ ] From the Mac, the SSH tunnel (§2a) is up. The agent prints a friendly
      error if the bridge is unreachable — a quick way to confirm the tunnel
      before the rover moves.

### 2f. Run the agent

**On your Mac (separate terminal, in the Python 3.10 venv)**

```bash
cd /Users/n.chinlue/code/RoverMind
source .venv/bin/activate
export OPENAI_API_KEY=sk-...
python scripts/run_agent.py "drive to the water bottle"
```

The agent talks to the bridge over the SSH tunnel from §2a. No ROS or Moondream
is imported on the Mac side — those live behind the bridge.

### 2g. Shut down

`Ctrl-C` each terminal in reverse order (agent → safety stack → limo
bringup). The launch file is wired so that if either node crashes, the
whole launch tears down — so a hung terminal usually means something is
wrong, not that it's safe to leave running.

---

## 3. Verified status (2026-05-26)

What I confirmed on the rover during this setup:

- [x] SSH key auth from Mac works (`ssh agilex@192.168.137.235`).
- [x] `~/rovermind_ws` builds cleanly via `colcon build --symlink-install`.
- [x] `ros2 launch safety_controller_layer rovermind.launch.py use_aeb:=false`
      starts and the `SafetyControllerNode` reports "action server ready".
- [x] `pytest tests/test_aeb_math.py tests/test_control_math.py
      tests/test_controller.py tests/test_depth_math.py
      tests/test_scene_parsing.py tests/test_observation_formatter.py
      tests/test_command_executor_pure.py tests/test_params.py
      tests/test_config_loader.py` — 115 pure-logic tests pass under
      the rover's Python 3.8.
- [ ] `scripts/run_agent.py` end-to-end via the new bridge workflow — pending
      live verification on the rover (see Task 4.3 manual gate in the
      py38-bridge plan).

---

## 4. Known issues

### Python version mismatch (RESOLVED 2026-05-26)

The agent now runs in a Python 3.10 venv on the developer's Mac and reaches
the rover's rclpy + Moondream via the Python 3.8 bridge at
`bridge/bridge_server.py`. See §2 for the every-session workflow. The bridge
exposes JSON-RPC over an SSH-tunneled TCP socket (`-L 9000:localhost:9000`)
and re-uses `agent/command_executor.py` and `perception/vision_tool.py`
unchanged.

### `colcon` can't discover both ROS packages from the repo root

Already worked around — the root `package.xml` makes `colcon` stop
recursing and miss `safety_controller_layer_interfaces`. The symlink
layout in §1b avoids it. Don't put `~/RoverMind` directly into a
workspace `src/` — use the symlinks.
