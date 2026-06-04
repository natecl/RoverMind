# Errors — taxonomy & diagnosis

What can fail, where it's raised, and how to tell which one you're hitting. Exception classes
are defined in the files named below.

## Bridge / connectivity (`bridge/errors.py`)

- **`BridgeUnreachable`** — can't connect to the rover bridge. Almost always the **SSH tunnel
  is down** or the bridge process isn't running. Check: tunnel up (`-L 9000:localhost:9000`),
  `ss -ltn | grep 9000` on the rover, rover reachable (see `ENVIRONMENT.md` to rediscover the IP).
- **`BridgeProtocolError`** — connected, but the reply was malformed or had a mismatched id.
  Usually a **version skew** (rover repo not synced) or a crashed/half-written response. Re-sync
  the repo to the rover and restart the bridge.
- **`MalformedFrameError`** (`bridge/wire.py`) — framing-level corruption (bad length header /
  truncated payload). Surfaces as a `BridgeProtocolError` to callers.

## Motion / control

- **`CommandExecutorError`** (`agent/command_executor.py`, re-exported via `bridge/errors.py`) —
  the ExecuteCommand action failed or was rejected. Common message
  `"drive_distance did not converge within budget"` → **AEB is off** so `/cmd_vel` has no
  publisher and the rover never moves (see ADR `0003`); or genuine timeout.
- **`ControllerTimeoutError`** (`safety_controller_layer/control_math.py`) — a rotate/drive loop
  exceeded its computed timeout budget. Check `/imu` and `/odom` are actually publishing
  (a stale sensor stalls the loop), and that the path wasn't AEB-braked the whole time.
- **`ControllerCancelledError`** — the goal was cancelled mid-maneuver (cancel request honored
  by the action server). Expected on Ctrl-C / abort, not a bug by itself.
- **`rclpy is not available`** (RuntimeError from `command_executor` import path) — the rover
  bridge was started with a clobbered `PYTHONPATH`. It must **prepend** the repo
  (`$HOME/RoverMind:$PYTHONPATH`), not replace it. Note `capture_and_analyze` doesn't need rclpy,
  so vision can pass while drive fails.

## Perception

- **`FrameCaptureError`** (`perception/vision_tool.py`, re-exported via `bridge/errors.py`) —
  no camera frame within the timeout. The **camera is a separate launch** (`orbbec_camera
  dabai_dcw2.launch.py`); `limo_start` does not start it. Check `/camera/color/image_raw` exists.
- **`distance_m` always `None` / `distance_source="vlm"`** — not an exception, a known gap:
  depth never populates, so distance falls back to Moondream's verbal estimate (which
  over-reports "close"). Either `moondream.point(target)` returned no point or the depth patch
  was all-invalid (depth not aligned to color, or holes). See `AGENT_WORKFLOW.md` follow-ups.

## Fail-safe behaviors (by design, not errors)

- Missing/stale `/scan` → `aeb_node` **brakes** (treats nearest obstacle as 0 m).
- Missing/stale `/cmd_vel_raw` → `aeb_node` publishes a **zero** Twist.
- Missing `/imu` or `/odom` → the controller **aborts the goal** (RuntimeError) rather than
  driving blind.

## First diagnostic moves

1. `BridgeClient.ping()` → isolates tunnel/bridge from everything downstream.
2. `ros2 topic info /cmd_vel` → `Publisher count: 1` confirms AEB is relaying.
3. `ros2 topic hz /scan /imu /odom` → confirms the sensors the controller and AEB depend on.
