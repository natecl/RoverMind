# Autonomous Emergency Braking (AEB) — Design Spec

> **Status:** approved design, ready for implementation planning.
> **Date:** 2026-05-21

## Goal

Add an autonomous emergency braking layer to the RoverMind safety stack. The
system continuously watches the LIMO Pro's 2D lidar and, whenever an obstacle
enters the rover's forward path, prevents the rover from driving into it by
zeroing forward velocity on `/cmd_vel` — independent of, and with final
authority over, whatever the controller or any other source is commanding.

## Context

The existing stack publishes velocity straight to `/cmd_vel`, which the LIMO
base driver consumes:

- `safety_controller_node.py` — `ExecuteCommand` action server, publishes `/cmd_vel`.
- `test_drive.py` — manual bring-up script, publishes `/cmd_vel`.
- A future VLM agent layer will also issue motion.

Nothing today stops the rover from driving into an obstacle. The codebase
already establishes a pattern this feature follows: **pure decision logic with
zero ROS imports** (`control_math.py`, fully unit-testable on the dev laptop
where `rclpy` is unavailable) plus a **thin ROS wrapper node**
(`safety_controller_node.py`).

## Locked-in design decisions (from clarifying Q&A 2026-05-21)

- **Trigger source:** 2D lidar `/scan` (`sensor_msgs/LaserScan`). Brake when an
  obstacle enters a threshold distance within the forward arc.
- **Recovery mode:** auto-release with **hysteresis** — a separate, larger
  release distance plus a dwell time, so the brake cannot chatter on/off when
  an obstacle or noisy reading sits near a single threshold. No manual reset.
- **Brake scope:** block **forward motion only**. When braking, forward linear
  velocity (`linear.x > 0`) is zeroed; rotation-in-place and reverse pass
  through, so the rover (or the agent) can turn or back away to escape rather
  than being trapped against the obstacle.
- **Architecture:** **velocity gate (inline filter)**. The AEB node is the sole
  publisher of `/cmd_vel`; all command sources are retargeted to `/cmd_vel_raw`.
  This gives the brake guaranteed authority — it cannot be out-voted by a
  racing publisher.
- **Package:** lives in the existing `safety_controller_layer` package as new
  modules (no new package).
- **Test strategy:** pure pytest for the decision logic; the ROS node has no
  laptop unit test (no `rclpy`), only manual hardware verification — consistent
  with `safety_controller_node.py`.

## Architecture

```
SafetyController ─┐
test_drive.py     ─┼─→ /cmd_vel_raw ─→ [EmergencyBrakeNode] ─→ /cmd_vel ─→ LIMO base
future agent      ─┘                          ↑
                                            /scan
```

The AEB node sits inline between every velocity source and the motors. It is
the **only** publisher of `/cmd_vel` (the topic the base driver consumes).
Because no other node writes that topic, the brake decision is final.

## File structure

**Create:**

- `safety_controller_layer/aeb_math.py` — pure decision logic, zero ROS imports.
- `safety_controller_layer/aeb_node.py` — thin ROS2 node.
- `tests/test_aeb_math.py` — unit tests for the pure logic.

**Modify:**

- `safety_controller_layer/safety_controller_node.py` — change the output topic
  constant `CMD_VEL_TOPIC` from `/cmd_vel` to `/cmd_vel_raw`.
- `test_drive.py` — change `CMD_VEL_TOPIC` from `/cmd_vel` to `/cmd_vel_raw`.
- `setup.py` — add a `console_scripts` entry point:
  `aeb_node = safety_controller_layer.aeb_node:main`.

## Pure-logic core (`aeb_math.py`)

Four units, all unit-testable without `rclpy`.

### `AebParams` — frozen dataclass

| Field | Default | Meaning |
|---|---|---|
| `trigger_distance_m` | `0.40` | Brake ON when the nearest forward obstacle is closer than this. |
| `release_distance_m` | `0.60` | Brake OFF only once the path is clear *past* this distance. |
| `release_dwell_s` | `0.5` | The path must stay clear past `release_distance_m` continuously for this long before the brake releases. |
| `forward_arc_deg` | `60.0` | Total forward sector width checked for obstacles (±30°). |
| `output_rate_hz` | `20.0` | Rate of the node's decision/publish tick. |
| `command_timeout_s` | `0.5` | A cached `/cmd_vel_raw` command older than this is treated as a zero Twist. |
| `scan_timeout_s` | `1.0` | If no `/scan` has arrived within this window, fail safe (brake). |

Construction validates `release_distance_m > trigger_distance_m`; otherwise the
hysteresis dead band is invalid and construction raises `ValueError`.

### `min_forward_range(ranges, angle_min, angle_increment, range_min, arc_half_width_rad) -> float`

Pure function. Given a `LaserScan`'s range array and geometry metadata, returns
the nearest **valid** obstacle distance within the forward arc.

- Forward is 0 rad. Beam `i` is at angle `angle_min + i * angle_increment`;
  each beam angle is normalised via `wrap_angle` (reused from `control_math.py`)
  so the arc straddling 0 rad is handled correctly.
- A beam is in-arc when `abs(wrapped_angle) <= arc_half_width_rad`.
- A reading is valid when it is finite and `>= range_min` (filters out NaN,
  `inf`, zero, and sub-minimum readings).
- Returns the minimum valid in-arc reading, or `inf` if there is no valid
  in-arc reading.

### `BrakeStateMachine`

Holds the hysteresis state across ticks.

- State: `braking: bool` (initial `False`), `_clear_since: Optional[float]`
  (initial `None`).
- `update(min_range: float, now: float) -> bool` — returns whether the brake is
  active after this update:
  - `min_range < trigger_distance_m` → `braking = True`, `_clear_since = None`.
  - else if currently `braking`:
    - `min_range > release_distance_m`: start the dwell timer
      (`_clear_since = now` if unset); once `now - _clear_since >=
      release_dwell_s`, set `braking = False`, `_clear_since = None`.
    - `trigger_distance_m <= min_range <= release_distance_m` (the dead band):
      hold `braking = True`, reset `_clear_since = None` (the path is not yet
      clear enough to count toward dwell).
  - else (`not braking`, `min_range >= trigger_distance_m`): no change.

The node's fail-safe feeds `min_range = 0.0` whenever `/scan` is missing or
stale, so the initial `braking = False` is never observed by a real publish —
the first tick always decides from a genuine reading or the fail-safe value.

### `gate_twist(linear_x, angular_z, braking) -> (linear_x, angular_z)`

Pure function. If `braking and linear_x > 0.0`, returns `(0.0, angular_z)`.
Otherwise returns `(linear_x, angular_z)` unchanged — reverse (`linear_x < 0`)
and rotation always pass through.

## ROS node (`aeb_node.py`)

`EmergencyBrakeNode(Node)`:

- Subscribes `/scan` (`sensor_msgs/LaserScan`) — caches the latest message and
  its arrival time.
- Subscribes `/cmd_vel_raw` (`geometry_msgs/Twist`) — caches the latest message
  and its arrival time.
- Publishes `/cmd_vel` (`geometry_msgs/Twist`).
- Runs a timer at `output_rate_hz` (20 Hz) — the single decision point per tick:
  1. If `/scan` is missing or older than `scan_timeout_s`, use `min_range =
     0.0` (fail-safe brake); otherwise compute `min_range` via
     `min_forward_range` from the cached scan.
  2. `braking = state_machine.update(min_range, now)`.
  3. If `/cmd_vel_raw` is missing or older than `command_timeout_s`, treat the
     raw command as a zero Twist; otherwise use the cached command.
  4. `lin, ang = gate_twist(raw.linear.x, raw.angular.z, braking)`.
  5. Publish the resulting Twist on `/cmd_vel`.
  6. Log (info/warn) on every brake on→off / off→on transition.
- A single-threaded executor is sufficient — all callbacks are lightweight and
  there is no blocking control loop (unlike `safety_controller_node.py`).
- On shutdown, publishes a final zero Twist.

`quaternion_to_yaw` is not needed here; the lidar arc math uses scan angles
directly.

## Fail-safe behavior

The tick decouples output rate from input rate, so the brake decision is always
fresh and stale commands cannot linger:

- **No `/scan` yet at startup, or `/scan` stale beyond `scan_timeout_s`** →
  brake. The rover will not drive forward until a fresh scan confirms a clear
  path.
- **`/cmd_vel_raw` stale beyond `command_timeout_s`** → treated as a zero Twist.
  A dropped or crashed publisher stops the rover rather than letting it coast on
  the last command.
- **A fresh `/scan` whose forward arc contains no valid beam** → treated as
  clear (`min_range = inf`). Occasional all-invalid arcs are transient lidar
  noise; a genuinely dead sensor is caught by `scan_timeout_s`.
- **Node shutdown** → a final zero Twist is published.

## Testing

### `tests/test_aeb_math.py` — pure unit tests (run on the dev laptop)

- `AebParams`: defaults match the spec table; construction with
  `release_distance_m <= trigger_distance_m` raises `ValueError`.
- `min_forward_range`: detects a forward obstacle; ignores an obstacle outside
  the arc; filters NaN / `inf` / zero / sub-`range_min` readings; returns `inf`
  for an empty or all-invalid arc; handles the arc straddling 0 rad.
- `BrakeStateMachine`: trips below `trigger_distance_m`; holds while the reading
  is in the dead band; releases only after the reading stays past
  `release_distance_m` for `release_dwell_s` (driven by a fake clock); the dwell
  timer resets if an obstacle re-enters the dead band; no on/off chatter when a
  reading hovers at a threshold.
- `gate_twist`: forward velocity zeroed when braking; rotation and reverse pass
  through when braking; nothing altered when not braking.

### `aeb_node.py` — manual hardware verification

No automated laptop test (`rclpy` unavailable), consistent with
`safety_controller_node.py`. On the rover:

- Drive the rover toward a wall — it stops before contact.
- Confirm that, while braked, the rover can still rotate in place and reverse.
- Back the rover away from the obstacle — confirm the brake auto-releases after
  the dwell time and forward motion resumes.

### Convention

Implemented test-first (red → green → commit per unit), matching the repo's TDD
convention.

## Out of scope (YAGNI)

- **`should_abort` hook integration** — feeding obstacle detection into
  `SafetyController.should_abort` is a possible later complement; the velocity
  gate is the authoritative mechanism and stands alone.
- **Depth-camera fusion** and **rear / lateral obstacle detection** — forward
  2D lidar only.
- **Latching e-stop / manual reset** — recovery is auto-release only.
- **A combined launch file** bringing up the controller and AEB node together —
  useful for deployment but not part of this feature's core logic.
- **The VLM agent layer** — unaffected; it will publish to `/cmd_vel_raw` like
  any other source when it is built.
