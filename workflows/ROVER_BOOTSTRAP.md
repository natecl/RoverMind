# Rover Bootstrap — when the LIMO is fresh / re-imaged

Companion to [`../LIMO_WORKFLOW.md`](../LIMO_WORKFLOW.md). That doc's §1 assumes
the one-time setup is "already done." It often **isn't**: the LIMO gets
re-imaged between sessions, which wipes `~/RoverMind` and the colcon workspace
while leaving the factory ROS install intact. This doc is the procedure to
detect that case and rebuild from scratch — **without moving the car.**

> Discovered 2026-05-28 on `172.20.10.6`: hostname `master` and `limo_bringup`
> were present, but `~/RoverMind` and `~/rovermind_ws` were gone, so
> `ros2 pkg list` showed only `limo_bringup`. A plain `rsync` (LIMO_WORKFLOW §2c)
> is **not enough** in this state — you also need the workspace + symlinks +
> `colcon build` below.

---

## 0. Find the current IP from the Mac *before* asking the user

The IP changes every session. Check the Mac first — you usually already know it:

```bash
grep -iE "agilex|172\.|192\.168" ~/.ssh/config        # saved Host blocks
grep -E "172\.|192\.168" ~/.ssh/known_hosts | cut -d' ' -f1 | sort -u
arp -an | grep -E "172\.|192\.168"                     # live neighbours on the LAN
ifconfig | grep "inet "                                # which subnet the Mac is on
```

The rover answers to `User agilex`, hostname `master`. The hotspot subnet seen
so far is `172.20.10.x` (rover `.6`, Mac `.4`). If none of these resolve, fall
back to LIMO_WORKFLOW.md §0 (USB tether `192.168.55.1`, or ask Nathan to run
`hostname -I` on the rover).

If key auth is rejected (`Permission denied (publickey,password)`), the rover
was re-imaged — have the user run `ssh-copy-id agilex@<ip>` once (it needs the
rover password at an interactive prompt, so the user must do it, not the agent).

---

## 1. Detect a fresh / re-imaged rover (read-only)

Non-interactive SSH skips the `~/.bashrc` `ros:noetic(1) foxy(2)` prompt, so
source the overlays manually. These commands change nothing:

```bash
ssh agilex@<ip> 'bash -lc "
  source /opt/ros/foxy/setup.bash 2>/dev/null
  source ~/limo_ros2_ws/install/setup.bash 2>/dev/null
  source ~/rovermind_ws/install/setup.bash 2>/dev/null
  echo \$ROS_DISTRO                                   # expect: foxy
  ros2 pkg list | grep -E \"safety_controller|limo_bringup\"
  ros2 node list; ros2 topic list                     # what is already running
"'
ssh agilex@<ip> 'ls -d ~/RoverMind ~/rovermind_ws/install/setup.bash 2>&1'
```

**Fresh-rover symptoms:** `ros2 pkg list` shows only `limo_bringup` (no
`safety_controller_*`); `~/RoverMind` and/or `~/rovermind_ws` are
"No such file or directory". → run the bootstrap below.

Confirm prereqs survived the re-image (all should be present):
`~/limo_ros2_ws/install/setup.bash`, `colcon`, `python3.8`, `git`, free disk.

---

## 2. Full bootstrap (steps 1–5, no car movement)

### Step 1 — sync the repo from the Mac
Prefer `rsync` from the Mac over `git clone` on the rover, so the rover gets your
exact local working tree (e.g. unpushed branch work like `py38-bridge`). Tradeoff:
`--exclude '.git'` means the rover copy is **not** a git repo — fine for running.

```bash
# On the Mac:
rsync -av -e "ssh -o ConnectTimeout=10" \
  --exclude '.git' --exclude '.venv' --exclude '__pycache__' \
  --exclude '*.pyc' --exclude '.pytest_cache' \
  ~/code/RoverMind/ agilex@<ip>:~/RoverMind/
```

### Steps 2–5 — workspace, symlinks, build (on the rover)
The repo root is itself the `safety_controller_layer` package, which makes
`colcon` stop recursing and miss the interfaces package — hence the symlink
layout (see LIMO_WORKFLOW.md §1b).

```bash
ssh agilex@<ip> 'bash -lc "
  set -e
  mkdir -p ~/rovermind_ws/src
  ln -sfn ~/RoverMind                                 ~/rovermind_ws/src/safety_controller_layer
  ln -sfn ~/RoverMind/safety_controller_layer_interfaces ~/rovermind_ws/src/safety_controller_layer_interfaces
  source /opt/ros/foxy/setup.bash
  source ~/limo_ros2_ws/install/setup.bash
  cd ~/rovermind_ws
  colcon build --symlink-install
"'
```

`colcon build` emits setuptools `setup.py install is deprecated` **warnings** on
Python 3.8 — those are noise, not failures. Look for `Summary: 2 packages
finished`. Built with `--symlink-install`, so later Python edits are live
without a rebuild; rebuild only when the `ExecuteCommand` interface or package
structure changes (the ament_cmake interfaces package is not symlinked).

**Not part of bootstrap:** `pip3 install -r requirements.txt
--break-system-packages` is flagged failing on Python 3.8 (LIMO_WORKFLOW.md §4).
Defer it to the bridge stage (§2d Terminal 3).

### Verify (read-only)
```bash
ssh agilex@<ip> 'bash -lc "
  source /opt/ros/foxy/setup.bash; source ~/limo_ros2_ws/install/setup.bash
  source ~/rovermind_ws/install/setup.bash
  cd ~/rovermind_ws && colcon list
  ros2 pkg list | grep -E safety_controller
  ros2 pkg executables safety_controller_layer
"'
```
Expect both packages from `colcon list`, all three in `ros2 pkg list`, and
`aeb_node` + `safety_controller_node` as executables.

---

## 2.5. Preflight gotchas (bring-up — found 2026-05-28)

Two issues block the LIMO_WORKFLOW.md §2e preflight on a stock re-imaged rover.
Both are silent (the stack *looks* up — action server ready, topics listed) but
no real data flows. Symptom of either: `ros2 topic echo <topic> --once` returns
nothing even though `ros2 topic info <topic>` shows a publisher.

### A. Stale Husarnet DDS profile blocks local data delivery
`~/.bashrc` exports `CYCLONEDDS_URI=file:///var/lib/theconstruct.rrl/cyclonedds.xml`,
a Husarnet/TheConstruct **VPN** profile (`Transport=udp6`, `AllowMulticast=false`,
unicast `<Peers>` = `husarnet-local` / `agilex-desktop` / `fc94:…` addresses that
don't exist on a plain hotspot). Participants discover each other (publisher
counts look right) but **samples never arrive**. RoverMind runs entirely on the
rover (the Mac agent reaches it via the SSH-tunnelled bridge on TCP 9000, not
ROS-over-network), so a **loopback** profile is correct.

- **Don't** just set `ROS_LOCALHOST_ONLY=1`: on loopback it disables multicast
  *and* caps `MaxAutoParticipantIndex` at ~9 → with ~10 nodes (limo + lidar + 3
  static-TF + safety + AEB + bridge + the bridge's lazy executor) you hit
  `Failed to find a free participant index for domain 2` and new participants
  (including the executor created on the first drive command) can't join.
- **Fix:** point `CYCLONEDDS_URI` at `~/rm_cyclonedds.xml` — loopback,
  `AllowMulticast=false`, `<Peer address="localhost"/>`, and
  `<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>`. (See `~/rm_env.sh`,
  which sources this for every non-interactive launch.)

### B. AEB `/scan` QoS mismatch
The YDLidar publishes `/scan` **BEST_EFFORT**; `emergency_brake` subscribed
**RELIABLE** (bare depth-`10`). Incompatible → AEB receives no scans → permanent
stale-scan fail-safe brake (safe but blind). Tell-tale log line:
`emergency_brake: ... incompatible QoS. No messages will be received ... RELIABILITY_QOS_POLICY`.
Fixed in `aeb_node.py` (uses `qos_profile_sensor_data` for `/scan`; guarded by
`tests/test_aeb_qos.py`, which runs on the rover). The controller's `/imu` `/odom`
subs are fine — those publishers are RELIABLE.

### Quick verification that both are fixed
```bash
ssh agilex@<ip> 'bash -lc "source ~/rm_env.sh >/dev/null 2>&1
  ros2 topic echo /imu --once | head -3                       # data flows -> DDS ok
  ros2 topic echo /scan --once --qos-reliability best_effort --no-arr | head -3
  grep -c \"incompatible QoS\" ~/rovermind_logs/safety.log     # expect 0 -> AEB QoS ok
"'
```
(`ros2 topic hz` does **not** accept `--qos-reliability` on Foxy; use `echo`.
Use `--qos-reliability best_effort` for `/scan`; the older `--field` flag errors.)

---

## 3. Safety rule — STOP after the build

Bootstrap leaves the rover *armed but idle*. Do **not** continue to
LIMO_WORKFLOW.md §2d (launch nodes) or §2f (`run_agent.py`, which physically
drives at up to 0.3 m/s) without **explicit per-step permission from the user**.
Before any movement, walk the LIMO_WORKFLOW.md §2e go/no-go gates (AEB on, open
floor clear of drop-offs, clear front arc, a human ready to Ctrl-C / lift).

The agent (an automated assistant) must ask before running anything that
launches nodes or moves the car — read-only inspection is fine, state changes
and movement are gated.
