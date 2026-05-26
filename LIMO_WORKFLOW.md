# LIMO Workflow — Run RoverMind on the Rover

How to SSH into the LIMO Agilex Pro and bring up the RoverMind stack.

---

## 0. Before doing anything: ask Nathan for the current Limo IP

> **PROMPT:** "What's the Limo's IP right now? Run `hostname -I` on the rover and paste the first IP."

The Limo's IP changes every session (Wi-Fi / hotspot reassignment). The
SSH config below is only valid for one session — re-confirm before reusing.

When verified on **2026-05-26**, the Limo reported:

```
$ hostname
master

$ hostname -I
192.168.137.235  192.168.55.1  fc94:10ff:c1d1:c276:f648:dfc1:9fc1:b32d
```

- `192.168.137.235` — Wi-Fi / hotspot IP (the one to SSH to from your Mac).
- `192.168.55.1` — Jetson's USB-Ethernet fallback (only reachable when the
  rover is tethered to the Mac with the supplied USB cable).
- The IPv6 is a temporary SLAAC address; ignore.

How to get the current IP if the Mac can't reach the rover:

1. Plug a screen + keyboard into the rover (or use the USB tether at
   `192.168.55.1`) and run `hostname -I`.
2. Or check the router/hotspot DHCP lease table for hostname `master`.

---

## 1. One-time setup (already done — for reference only)

Done on 2026-05-26. Skip this section if SSH key auth still works.

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
# On your Mac:
ssh agilex@<LIMO_IP>             # use the IP you confirmed in step 0
```

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

### 2c. Bring up the stack (three terminals — three SSH sessions)

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

**Terminal 3 — the agent**

```bash
cd ~/RoverMind
export OPENAI_API_KEY=sk-...
python3 scripts/run_agent.py "drive to the water bottle"
```

### 2d. Shut down

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
- [ ] `scripts/run_agent.py` end-to-end — **blocked**, see Known Issues.

---

## 4. Known issues

### Python version mismatch (agent won't import yet)

The rover ships Ubuntu 20.04 with `python3 → /usr/bin/python3.8`. ROS2
Foxy's `rclpy` is built against Python 3.8 only.

But `langgraph`, `langchain-core`, and the agent's `agent/state.py`
(which uses `typing.Annotated`) all require Python ≥3.9 — the README
itself calls for 3.10+.

Symptoms when running on Python 3.8:

```text
ERROR: Could not find a version that satisfies the requirement langgraph
ImportError: cannot import name 'Annotated' from 'typing'
ModuleNotFoundError: No module named 'langchain_core'
```

`/usr/bin/python3.9` exists on the rover, but installing langgraph against
it doesn't fix the problem on its own because `rclpy` (used by
`agent/command_executor.py` → `agent/graph.py`) is only available for
Python 3.8 under Foxy.

Options to unblock — pick one before the next bring-up:

1. **Upgrade ROS to Humble** (ships with Python 3.10) — the cleanest fix
   long-term, but requires reflashing the Jetson stack.
2. **Two-process split** — run the LangGraph agent in a Python 3.10
   `venv` and have it call the safety controller via the ROS2 action
   from a thin Python 3.8 bridge. The action interface
   (`safety_controller_layer_interfaces/action/ExecuteCommand`) already
   exists; the bridge would just need to translate the agent's
   `execute_command()` call into an `ActionClient` request.
3. **Rebuild `rclpy` for Python 3.9** locally on the Jetson — possible
   but fiddly.

Until one of those is in place, the agent terminal in §2c will fail at
import time. The ROS layer (terminals 1 and 2) is fully functional and
can be driven manually with `ros2 action send_goal /execute_command
safety_controller_layer_interfaces/action/ExecuteCommand ...` for
controller bring-up.

### `colcon` can't discover both ROS packages from the repo root

Already worked around — the root `package.xml` makes `colcon` stop
recursing and miss `safety_controller_layer_interfaces`. The symlink
layout in §1b avoids it. Don't put `~/RoverMind` directly into a
workspace `src/` — use the symlinks.
