# RoverMind Agent — End-to-End Workflow & Field Notes

How to bring the rover up from cold and run the autonomous agent
("drive to the \<target\>") through the Python 3.8 bridge. This is the
canonical, **checked-in** companion to [`LIMO_WORKFLOW.md`](LIMO_WORKFLOW.md)
(SSH + stack bring-up). Detailed planning/runbook files under `workflows/` and
`docs/` are **gitignored** (local-only); this file captures the durable
procedure plus everything learned during the first full live bring-up
(2026-05-29) where all four smoke gates passed.

---

## 1. Architecture (where each piece runs)

```
┌─────────────── Mac (Python 3.10 venv) ───────────────┐        ┌──────────────────── Rover "master" (agilex, Python 3.8) ────────────────────┐
│ scripts/run_agent.py                                  │        │  bridge/bridge_server.py  (owns rclpy ActionClient + Moondream)              │
│   LangGraph reasoner + agent/prompts.py SYSTEM_PROMPT │  SSH   │     ├── execute_command ──► safety_controller ─► /cmd_vel_raw                 │
│   tools: look / turn / forward / search / stop        │ tunnel │     │                          ─► emergency_brake (AEB) ─► /cmd_vel ─► limo_base │
│   BridgeClient ──── tcp://localhost:9000 ─────────────┼────────┼────►│     └── capture_and_analyze ─► Orbbec camera (color+depth) + Moondream2 GPU  │
└───────────────────────────────────────────────────────┘  -L    └──────────────────────────────────────────────────────────────────────────────┘
```

- **The agent, the LLM, and the prompt run on the Mac.** No ROS or ML is
  imported on the Mac — all rover I/O goes over the SSH-tunnelled bridge (TCP
  9000, JSON-RPC).
- **The bridge, ROS stack, camera, and Moondream run on the rover.** The bridge
  builds the `SceneObservation` (so `perception/scene_parsing.py` runs *there*).
- **Direction/distance parsing runs on the rover; the reasoning prompt runs on
  the Mac.** A change to `scene_parsing.py` needs a rover deploy + bridge
  restart; a change to `agent/prompts.py` only needs to be present on the Mac.

---

## 2. Connect to the rover (the IP changes every session)

The rover is **always** MAC `54:ef:33:9e:e7:71`, user `agilex`, hostname
`master`. There are 2–3 AgileX rovers around — do **not** use `…e7:73` or
`…ea:7f`. Both Mac and rover must be on the **same peer-to-peer-capable
network** (a phone/Windows hotspot, typically `192.168.137.x`). A corporate/
guest Wi-Fi with client isolation will *not* work even if both are "connected".

```bash
# On the Mac — find the current IP by MAC:
for i in $(seq 1 254); do ping -c1 -W300 192.168.137.$i >/dev/null 2>&1 & done; wait
arp -an | grep -i "54:ef:33:9e:e7:71"     # -> our rover's current IP
ssh agilex@<ip> 'cat /sys/class/net/wlan0/address; ls ~/moondream2_local/.vendor_done'
```
Sudo password for `agilex` is set on the rover; `sudo` is **not** passwordless.

### Wi-Fi to this rover is flaky — use these patterns
SSH to this rover drops intermittently (random `exit 255` mid-command, even
with clean `ping`). What works:
- **Power-save off** after every boot: `sudo iw dev wlan0 set power_save off`
  (the driver may revert it; re-apply if drops worsen).
- **SSH connection multiplexing** — one persistent master, reused for all calls:
  ```bash
  ssh -o ControlMaster=auto -o ControlPath='/tmp/rm/cm-%r@%h:%p' \
      -o ControlPersist=900 -o ServerAliveInterval=15 -fN agilex@<ip>
  # then every command: ssh -o ControlPath='/tmp/rm/cm-%r@%h:%p' agilex@<ip> '...'
  ```
- **Keep launch commands instant.** Long SSH commands (with `sleep`, sourcing,
  builds) get cut by a drop mid-stream. Put slow work in a **rover-side launcher
  script** and trigger it with one fast, detached command (see §4). Do **not**
  hammer with tight retry loops — that trips sshd throttling and makes it worse.
- **Run all rover processes detached** so an SSH drop never tears them down:
  `setsid bash -c "<cmd> > log 2>&1 < /dev/null" & disown`.

---

## 3. One-time-per-session environment (`~/rm_env.sh`)

Non-interactive SSH skips the rover's `~/.bashrc` `ros:noetic(1) foxy(2)`
prompt, so nothing gets sourced. Create `~/rm_env.sh` once and source it in every
launch. **Critical:** `PYTHONPATH` must *prepend* the repo, never replace it —
replacing it drops rclpy and the bridge fails with `rclpy is not available`.

```bash
# ~/rm_env.sh
source /opt/ros/foxy/setup.bash
source ~/limo_ros2_ws/install/setup.bash
source ~/rovermind_ws/install/setup.bash
export ROS_DOMAIN_ID=2
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$HOME/rm_cyclonedds.xml
export MOONDREAM_MODEL_PATH=$HOME/moondream2_local
export PYTHONPATH=$HOME/RoverMind:$PYTHONPATH        # PREPEND, do not overwrite
```

`~/rm_cyclonedds.xml` — loopback profile. We preemptively set
`MaxAutoParticipantIndex=200` to avoid "Failed to find a free participant index"
once ~10 nodes are up; in practice the error did **not** appear, and node-to-node
DDS delivery worked fine (a fresh `ros2 topic echo/hz` from the CLI may show no
samples — that is a late-joiner/QoS artifact, not a real delivery failure).

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <General><NetworkInterfaceAddress>127.0.0.1</NetworkInterfaceAddress>
             <AllowMulticast>false</AllowMulticast></General>
    <Discovery><ParticipantIndex>auto</ParticipantIndex>
               <MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>
               <Peers><Peer address="localhost"/></Peers></Discovery>
  </Domain>
</CycloneDDS>
```

---

## 4. Bring up the stack (rover-side launcher scripts, all detached)

Each component gets a tiny launcher script (written once via `cat`) and is fired
with one instant detached SSH command. This survives the flaky link.

```bash
# Launchers (write once on the rover):
#   ~/safety_run.sh : source ~/rm_env.sh; exec ros2 launch safety_controller_layer rovermind.launch.py
#   ~/cam_run.sh    : source ~/rm_env.sh; exec ros2 launch orbbec_camera dabai_dcw2.launch.py
#   ~/bridge_run.sh : source ~/rm_env.sh; cd ~/RoverMind; exec python3.8 bridge/bridge_server.py --bind 127.0.0.1:9000

# Trigger each (instant return; work runs detached on the rover):
ssh agilex@<ip> 'setsid bash -c "bash ~/<launcher>.sh > ~/rovermind_logs/<x>.log 2>&1 < /dev/null" & disown; echo FIRED'
```

Bring-up order:
1. **LIMO base drivers** — `ros2 launch limo_bringup limo_start.launch.py`
   → `/scan /imu /odom /cmd_vel`, `limo_base`, `ydlidar`.
2. **Safety controller + AEB** — `~/safety_run.sh` (**AEB ON — see gotcha
   below**). Expect `SafetyControllerNode up; ExecuteCommand action server ready`
   and `EmergencyBrakeNode up: gating /cmd_vel_raw -> /cmd_vel`.
3. **Camera** — `~/cam_run.sh`. `limo_start` does **not** start the camera; the
   Orbbec needs its own launch (this rover = Dabai DCW2, USB `2bc5:0657`).
   → `/camera/color/image_raw`, `/camera/depth/image_raw`.
4. **Bridge** — `~/bridge_run.sh`. Expect `[bridge] listening on
   tcp://127.0.0.1:9000`.

Then from the Mac, forward the port (over the existing master is easiest):
```bash
ssh -o ControlPath='/tmp/rm/cm-%r@%h:%p' -O forward -L 9000:localhost:9000 agilex@<ip>
```

### Memory (8 GB Jetson, unified GPU/CPU memory)
Moondream + full ROS stack + camera is **tight** (idle ≈ 6.8 GB used, ~190 MB
free, ~1–2 GB swapped). It is tight-but-stable once Moondream has loaded (the
memory peak is the one-time model load). To add headroom, stop the desktop GUI
(non-destructive, reversible): `sudo systemctl stop gdm` (frees a few hundred MB
+ reduces swap pressure; `start gdm` to restore). Watch `free -h`.

---

## 5. Smoke gates (run in order, do not skip; motion gates need permission)

| Gate | What | Pass |
|------|------|------|
| **1.5** | `BridgeClient.ping()` over the tunnel | returns `pong` |
| **2.3** | `execute_command(heading_deg=0, distance_m=0.3)` — **first motion** | `success=True` **and** rover physically drives ~30 cm |
| **3.4** | `capture_and_analyze("<target>")` — real camera + Moondream | returns a plausible `SceneObservation`; first call loads Moondream (~25–40 s), then ~16 s/call |
| **4.3** | `python scripts/run_agent.py "drive to the <target>"` from the Mac | terminal `status: arrived` (or coherent `lost`/`aborted`) **and** rover meaningfully approaches |

Physical observations ("did it move?", "did it approach?", "is the scene
plausible?") require a human — the agent cannot see the rover.

---

## 6. Run the agent (Gate 4.3)

```bash
# On the Mac, in the venv, with the tunnel + AEB-on stack up:
cd /Users/n.chinlue/code/RoverMind
source .venv/bin/activate
python scripts/run_agent.py "drive to the water bottle"   # OPENAI_API_KEY auto-loaded from .env
```
The reasoner loops: `look` → (`turn` if clearly to one side / `forward` if
centered) → `look` → … → `stop("arrived")` when centered AND close. Each `look`
is ~16 s (three Moondream questions), so a run is a few minutes.

---

## 7. Field notes / gotchas learned (2026-05-29 first full bring-up)

1. **AEB is the command relay, not just a brake.** `safety_controller` publishes
   only `/cmd_vel_raw`; `limo_base` subscribes only to `/cmd_vel`; the AEB node
   bridges `/cmd_vel_raw → /cmd_vel`. Launching with `use_aeb:=false` leaves
   `/cmd_vel` with **0 publishers** → commands never reach the motors → the rover
   sits still and `execute_command` aborts `"drive_distance did not converge
   within budget"`. **Always launch with AEB ON** (no `use_aeb` arg). Sanity
   check: `ros2 topic info /cmd_vel` must show `Publisher count: 1`. (The
   `docs/.../py38-bridge-rover-smoke.md` runbook says `use_aeb:=false` for Gate
   2.3 — that is wrong for this rover.)

2. **Camera is a separate launch.** `limo_start.launch.py` brings up base + lidar
   only. Start the Orbbec Dabai DCW2 with
   `ros2 launch orbbec_camera dabai_dcw2.launch.py` to get color+depth.

3. **Bridge `PYTHONPATH`** must prepend `~/RoverMind` (`$HOME/RoverMind:$PYTHONPATH`),
   not replace it, or rclpy disappears and `execute_command` fails with
   `rclpy is not available`. (`capture_and_analyze` alone doesn't need rclpy,
   which is why a Moondream-only check can pass while drive fails.)

4. **Direction parsing — center includes slight offsets.** Moondream answers
   verbosely, e.g. "in the middle of the image, slightly to the right of center."
   `parse_direction` treats any mention of middle/center as **`center` → drive
   forward**, and only a *clear* side (no center mention) as a turn. The prompt
   matches: prefer forward progress, turn only when clearly off to one side.
   (Earlier this returned `None` for such answers → the rover spun in place
   searching; that is the bug fixed in this PR.)

5. **Target placement for a real drive.** Too far → Moondream reports
   not-found → the agent search-rotates. Too close → it reports "close" →
   `should_stop=True` → the agent insta-"arrives" without driving. Place the
   target at a **medium distance, clearly in the forward view** for an actual
   approach.

6. **Moondream cost / no leak.** First call ≈ 25–40 s (model → GPU), then ≈ 16 s
   per call (three sequential VLM questions). It does **not** reload between
   calls; 16 s is steady-state inference on this Jetson, not a leak.

---

## 8. Known follow-ups

- **Depth distance never populates** — `capture_and_analyze` always returns
  `distance_m=None`, `distance_source="vlm"`, so distance relies on Moondream's
  verbal estimate (which over-reports "close"; observed "close" when actually
  medium). The bridge *does* wire `depth_fn`, and the local Moondream snapshot
  *does* implement `point`, so the fallback is a runtime outcome: either
  `moondream.point(target)` returns no point, or the depth patch at that pixel is
  all-invalid (depth not registered/aligned to color, or holes). Pinpointing
  needs instrumenting the bridge's capture path (a second standalone Moondream
  probe is not feasible — memory is too tight). Until fixed, the agent stops on
  Moondream's verbal "close".
- **Wi-Fi stability** — physical/RF issue between Mac and rover; multiplexing +
  detached launchers work around it but do not fix the underlying drops.
