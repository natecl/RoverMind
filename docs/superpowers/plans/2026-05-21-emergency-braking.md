# Autonomous Emergency Braking (AEB) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an autonomous emergency braking layer that zeroes the LIMO Pro's forward velocity whenever the 2D lidar sees an obstacle in the rover's forward path.

**Architecture:** A velocity gate. All command sources publish `/cmd_vel_raw`; a new `EmergencyBrakeNode` is the sole publisher of `/cmd_vel`, forwarding commands but zeroing forward linear velocity while a `/scan` obstacle is inside the forward arc. The brake decision is pure logic (`aeb_math.py`, zero ROS imports, fully laptop-testable) wrapped by a thin ROS node (`aeb_node.py`), mirroring the existing `control_math.py` / `safety_controller_node.py` split.

**Tech Stack:** Python 3.13, pytest 8.x for unit tests. On the rover only: rclpy, sensor_msgs, geometry_msgs (ROS2 Foxy).

**Spec:** `docs/superpowers/specs/2026-05-21-emergency-braking-design.md`

**Commit convention:** every commit message in this plan should end with the trailer line:
`Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>`

---

## File Structure

**Create:**
- `safety_controller_layer/aeb_math.py` — pure decision logic: `AebParams`, `min_forward_range`, `BrakeStateMachine`, `gate_twist`. Zero ROS imports.
- `safety_controller_layer/aeb_node.py` — ROS2 `EmergencyBrakeNode` velocity gate + `main()`.
- `tests/test_aeb_math.py` — unit tests for the pure logic.

**Modify:**
- `safety_controller_layer/safety_controller_node.py` — output topic `/cmd_vel` → `/cmd_vel_raw`.
- `test_drive.py` — output topic `/cmd_vel` → `/cmd_vel_raw`.
- `tests/test_execute_command_integration.py` — control-loop subscription `/cmd_vel` → `/cmd_vel_raw`.
- `setup.py` — add the `aeb_node` console entry point.

**Baseline:** `python3 -m pytest -q` currently reports **39 passed, 3 skipped**.

---

## Phase 1: Brake Decision From a Known Distance

**End state of phase:** given an obstacle distance, the system decides whether to brake — with hysteresis and dwell so it cannot chatter — and gates a Twist accordingly (forward zeroed while braking; rotation and reverse pass). All pure logic, fully laptop-testable. Proven end-to-end by a scenario test.

### Task 1.1: `AebParams` dataclass with spec defaults

**Files:**
- Create: `safety_controller_layer/aeb_math.py`
- Test: `tests/test_aeb_math.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_aeb_math.py`:
```python
import math

import pytest

from safety_controller_layer.aeb_math import AebParams


def test_aeb_params_defaults_match_spec():
    p = AebParams()
    assert p.trigger_distance_m == 0.40
    assert p.release_distance_m == 0.60
    assert p.release_dwell_s == 0.5
    assert p.forward_arc_deg == 60.0
    assert p.output_rate_hz == 20.0
    assert p.command_timeout_s == 0.5
    assert p.scan_timeout_s == 1.0


def test_aeb_params_rejects_release_not_greater_than_trigger():
    with pytest.raises(ValueError):
        AebParams(trigger_distance_m=0.5, release_distance_m=0.5)
    with pytest.raises(ValueError):
        AebParams(trigger_distance_m=0.5, release_distance_m=0.4)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_aeb_math.py -v`
Expected: `ModuleNotFoundError: No module named 'safety_controller_layer.aeb_math'`.

- [ ] **Step 3: Write minimal implementation**

Create `safety_controller_layer/aeb_math.py`:
```python
"""Pure decision logic for the autonomous emergency braking (AEB) layer.

Zero ROS imports -- fully unit-testable on the dev laptop. The ROS wiring lives
in aeb_node.py.
"""

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from safety_controller_layer.control_math import wrap_angle


@dataclass(frozen=True)
class AebParams:
    """Tunable parameters for the emergency braking layer."""

    trigger_distance_m: float = 0.40
    release_distance_m: float = 0.60
    release_dwell_s: float = 0.5
    forward_arc_deg: float = 60.0
    output_rate_hz: float = 20.0
    command_timeout_s: float = 0.5
    scan_timeout_s: float = 1.0

    def __post_init__(self) -> None:
        if self.release_distance_m <= self.trigger_distance_m:
            raise ValueError(
                f"release_distance_m ({self.release_distance_m}) must be greater "
                f"than trigger_distance_m ({self.trigger_distance_m})"
            )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_aeb_math.py -v`
Expected: 2 passed.

- [ ] **Step 5: Commit**

```bash
git add safety_controller_layer/aeb_math.py tests/test_aeb_math.py
git commit -m "feat: add AebParams dataclass with spec defaults"
```

---

### Task 1.2: `gate_twist` — zero forward velocity while braking

**Files:**
- Modify: `safety_controller_layer/aeb_math.py`
- Test: `tests/test_aeb_math.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_aeb_math.py`:
```python
from safety_controller_layer.aeb_math import gate_twist


def test_gate_twist_zeroes_forward_when_braking():
    assert gate_twist(0.3, 0.0, braking=True) == (0.0, 0.0)


def test_gate_twist_passes_rotation_when_braking():
    assert gate_twist(0.0, 0.5, braking=True) == (0.0, 0.5)


def test_gate_twist_passes_reverse_when_braking():
    assert gate_twist(-0.2, 0.0, braking=True) == (-0.2, 0.0)


def test_gate_twist_passes_everything_when_not_braking():
    assert gate_twist(0.3, 0.5, braking=False) == (0.3, 0.5)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_aeb_math.py -v`
Expected: `ImportError: cannot import name 'gate_twist'`.

- [ ] **Step 3: Write minimal implementation**

Append to `safety_controller_layer/aeb_math.py`:
```python
def gate_twist(
    linear_x: float, angular_z: float, braking: bool
) -> Tuple[float, float]:
    """Zero forward linear velocity while braking; pass rotation and reverse."""
    if braking and linear_x > 0.0:
        return (0.0, angular_z)
    return (linear_x, angular_z)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_aeb_math.py -v`
Expected: 6 passed.

- [ ] **Step 5: Commit**

```bash
git add safety_controller_layer/aeb_math.py tests/test_aeb_math.py
git commit -m "feat: add gate_twist forward-velocity filter"
```

---

### Task 1.3: `BrakeStateMachine` — trip below trigger distance

**Files:**
- Modify: `safety_controller_layer/aeb_math.py`
- Test: `tests/test_aeb_math.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_aeb_math.py`:
```python
from safety_controller_layer.aeb_math import BrakeStateMachine


def test_brake_state_machine_starts_not_braking():
    sm = BrakeStateMachine(AebParams())
    assert sm.braking is False


def test_brake_state_machine_trips_below_trigger_distance():
    sm = BrakeStateMachine(AebParams())
    assert sm.update(min_range=0.30, now=0.0) is True
    assert sm.braking is True


def test_brake_state_machine_stays_clear_above_trigger_distance():
    sm = BrakeStateMachine(AebParams())
    assert sm.update(min_range=1.0, now=0.0) is False
    assert sm.braking is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_aeb_math.py -v`
Expected: `ImportError: cannot import name 'BrakeStateMachine'`.

- [ ] **Step 3: Write minimal implementation**

Append to `safety_controller_layer/aeb_math.py`:
```python
class BrakeStateMachine:
    """Maps forward obstacle distance to a brake flag (trip half only)."""

    def __init__(self, params: AebParams) -> None:
        self.params = params
        self.braking = False

    def update(self, min_range: float, now: float) -> bool:
        if min_range < self.params.trigger_distance_m:
            self.braking = True
        return self.braking
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_aeb_math.py -v`
Expected: 9 passed.

- [ ] **Step 5: Commit**

```bash
git add safety_controller_layer/aeb_math.py tests/test_aeb_math.py
git commit -m "feat: add BrakeStateMachine trigger logic"
```

---

### Task 1.4: `BrakeStateMachine` — hysteresis release with dwell

**Files:**
- Modify: `safety_controller_layer/aeb_math.py`
- Test: `tests/test_aeb_math.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_aeb_math.py`:
```python
def test_brake_holds_in_dead_band():
    sm = BrakeStateMachine(AebParams())
    sm.update(min_range=0.30, now=0.0)                   # trip
    assert sm.update(min_range=0.50, now=1.0) is True    # dead band -> hold


def test_brake_releases_after_dwell_past_release_distance():
    sm = BrakeStateMachine(AebParams())                  # dwell = 0.5 s
    sm.update(min_range=0.30, now=0.0)                   # trip
    assert sm.update(min_range=0.70, now=1.0) is True    # clear, dwell starts
    assert sm.update(min_range=0.70, now=1.4) is True    # 0.4 s < dwell -> hold
    assert sm.update(min_range=0.70, now=1.5) is False   # 0.5 s >= dwell -> release


def test_brake_does_not_release_before_dwell_elapses():
    sm = BrakeStateMachine(AebParams())
    sm.update(min_range=0.30, now=0.0)                   # trip
    sm.update(min_range=0.70, now=1.0)                   # clear, dwell starts
    assert sm.update(min_range=0.70, now=1.49) is True   # still within dwell


def test_brake_dwell_resets_when_obstacle_reenters_dead_band():
    sm = BrakeStateMachine(AebParams())                  # dwell = 0.5 s
    sm.update(min_range=0.30, now=0.0)                   # trip
    sm.update(min_range=0.70, now=1.0)                   # clear past release
    sm.update(min_range=0.50, now=1.2)                   # back into dead band
    assert sm.update(min_range=0.70, now=1.6) is True    # dwell restarts at 1.6
    assert sm.update(min_range=0.70, now=2.0) is True    # 0.4 s into restart
    assert sm.update(min_range=0.70, now=2.1) is False   # 0.5 s -> release


def test_brake_does_not_chatter_at_trigger_threshold():
    sm = BrakeStateMachine(AebParams())
    sm.update(min_range=0.30, now=0.0)                   # trip
    for t in range(1, 20):
        assert sm.update(min_range=0.41, now=float(t)) is True
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_aeb_math.py -v`
Expected: `test_brake_releases_after_dwell_past_release_distance` fails — the trip-only `update` never sets `braking` back to `False`.

- [ ] **Step 3: Write minimal implementation**

In `safety_controller_layer/aeb_math.py`, replace the entire `BrakeStateMachine` class with the hysteresis-aware version:
```python
class BrakeStateMachine:
    """Hysteresis state machine: maps forward obstacle distance to a brake flag.

    Braking trips when the nearest obstacle is closer than trigger_distance_m.
    It releases only after the path stays clear beyond release_distance_m for
    release_dwell_s continuously -- the gap between the two distances is a dead
    band that prevents on/off chatter.
    """

    def __init__(self, params: AebParams) -> None:
        self.params = params
        self.braking = False
        self._clear_since: Optional[float] = None

    def update(self, min_range: float, now: float) -> bool:
        params = self.params
        if min_range < params.trigger_distance_m:
            self.braking = True
            self._clear_since = None
        elif self.braking:
            if min_range > params.release_distance_m:
                if self._clear_since is None:
                    self._clear_since = now
                elif now - self._clear_since >= params.release_dwell_s:
                    self.braking = False
                    self._clear_since = None
            else:
                # dead band: trigger_distance_m <= min_range <= release_distance_m
                self._clear_since = None
        return self.braking
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_aeb_math.py -v`
Expected: 14 passed.

- [ ] **Step 5: Commit**

```bash
git add safety_controller_layer/aeb_math.py tests/test_aeb_math.py
git commit -m "feat: add hysteresis release with dwell to BrakeStateMachine"
```

---

### Task 1.5: Phase 1 capstone — brake/release scenario

**Files:**
- Test: `tests/test_aeb_math.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_aeb_math.py`:
```python
def test_phase1_brake_and_release_scenario():
    """End-to-end Phase 1 slice: a sequence of obstacle distances drives the
    state machine, and the resulting brake flag gates a steady 0.3 m/s forward
    command."""
    params = AebParams()
    sm = BrakeStateMachine(params)
    forward_cmd = (0.3, 0.0)  # linear_x, angular_z

    # Far away -> command passes untouched.
    braking = sm.update(min_range=2.0, now=0.0)
    assert gate_twist(*forward_cmd, braking=braking) == (0.3, 0.0)

    # Obstacle inside trigger -> forward zeroed.
    braking = sm.update(min_range=0.35, now=0.1)
    assert gate_twist(*forward_cmd, braking=braking) == (0.0, 0.0)

    # While braked, a rotate command still passes.
    assert gate_twist(0.0, 0.5, braking=braking) == (0.0, 0.5)

    # Back away past release distance; brake holds until dwell elapses.
    braking = sm.update(min_range=0.80, now=0.2)
    assert gate_twist(*forward_cmd, braking=braking) == (0.0, 0.0)

    # After the dwell, brake releases and forward motion resumes.
    braking = sm.update(min_range=0.80, now=0.7)
    assert gate_twist(*forward_cmd, braking=braking) == (0.3, 0.0)
```

- [ ] **Step 2: Run test to verify it passes immediately**

Run: `python3 -m pytest tests/test_aeb_math.py -v`
Expected: 15 passed. This is a capstone scenario test exercising already-implemented units; it should pass with no new code. If it fails, a Task 1.2–1.4 unit has a bug — fix it before continuing.

- [ ] **Step 3: Commit**

```bash
git add tests/test_aeb_math.py
git commit -m "test: add Phase 1 brake/release scenario capstone"
```

---

## Phase 2: Brake On Real Lidar Data

**End state of phase:** `min_forward_range` reduces a raw `LaserScan`'s range array to the nearest forward-arc obstacle distance, completing the pure pipeline `LaserScan -> min_forward_range -> BrakeStateMachine -> gate_twist`. Proven end-to-end by a capstone test fed a fake scan.

### Task 2.1: `min_forward_range` — nearest obstacle in the forward arc

**Files:**
- Modify: `safety_controller_layer/aeb_math.py`
- Test: `tests/test_aeb_math.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_aeb_math.py`:
```python
from safety_controller_layer.aeb_math import min_forward_range


def test_min_forward_range_finds_forward_obstacle():
    # 5 beams at -90, -45, 0, +45, +90 deg; only the 0 deg beam is in the arc
    ranges = [5.0, 5.0, 0.8, 5.0, 5.0]
    result = min_forward_range(
        ranges, angle_min=-math.pi / 2, angle_increment=math.pi / 4,
        range_min=0.05, arc_half_width_rad=math.radians(30),
    )
    assert result == 0.8


def test_min_forward_range_ignores_obstacle_outside_arc():
    # close obstacle at +45 deg, outside the +/-30 deg arc
    ranges = [5.0, 5.0, 5.0, 0.3, 5.0]
    result = min_forward_range(
        ranges, angle_min=-math.pi / 2, angle_increment=math.pi / 4,
        range_min=0.05, arc_half_width_rad=math.radians(30),
    )
    assert result == 5.0


def test_min_forward_range_filters_invalid_readings():
    # in-arc beams at -20, 0, +20 deg read inf, 0.0, 1.2 -> only 1.2 is valid
    ranges = [math.nan, math.inf, 0.0, 1.2, 5.0]
    result = min_forward_range(
        ranges, angle_min=-math.radians(40), angle_increment=math.radians(20),
        range_min=0.05, arc_half_width_rad=math.radians(30),
    )
    assert result == 1.2


def test_min_forward_range_filters_sub_range_min_readings():
    # 0.03 is below range_min (0.05) -> discarded; 2.0 is the answer
    ranges = [2.0, 0.03]
    result = min_forward_range(
        ranges, angle_min=-math.radians(10), angle_increment=math.radians(20),
        range_min=0.05, arc_half_width_rad=math.radians(30),
    )
    assert result == 2.0


def test_min_forward_range_returns_inf_when_arc_empty():
    # both in-arc beams invalid -> inf
    ranges = [math.nan, math.inf]
    result = min_forward_range(
        ranges, angle_min=-math.radians(10), angle_increment=math.radians(20),
        range_min=0.05, arc_half_width_rad=math.radians(30),
    )
    assert result == math.inf


def test_min_forward_range_handles_arc_straddling_zero():
    # 8 beams 45 deg apart starting at 0 rad; forward arc spans beam 0 and beam 7
    ranges = [0.9, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.7]
    result = min_forward_range(
        ranges, angle_min=0.0, angle_increment=math.radians(45),
        range_min=0.05, arc_half_width_rad=math.radians(50),
    )
    assert result == 0.7
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_aeb_math.py -v`
Expected: `ImportError: cannot import name 'min_forward_range'`.

- [ ] **Step 3: Write minimal implementation**

Append to `safety_controller_layer/aeb_math.py`:
```python
def min_forward_range(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    arc_half_width_rad: float,
) -> float:
    """Nearest valid obstacle distance within the forward arc.

    Forward is 0 rad. A beam counts when its angle (normalised to [-pi, pi))
    falls within +/- arc_half_width_rad. A reading counts when it is finite and
    at least range_min. Returns inf when no valid in-arc reading exists.
    """
    nearest = math.inf
    for i, r in enumerate(ranges):
        angle = wrap_angle(angle_min + i * angle_increment)
        if abs(angle) > arc_half_width_rad:
            continue
        if not math.isfinite(r) or r < range_min:
            continue
        if r < nearest:
            nearest = r
    return nearest
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_aeb_math.py -v`
Expected: 21 passed.

- [ ] **Step 5: Commit**

```bash
git add safety_controller_layer/aeb_math.py tests/test_aeb_math.py
git commit -m "feat: add min_forward_range lidar arc reducer"
```

---

### Task 2.2: Phase 2 capstone — full pipeline from a LaserScan

**Files:**
- Test: `tests/test_aeb_math.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_aeb_math.py`:
```python
def test_phase2_full_pipeline_brakes_on_lidar_obstacle():
    """End-to-end Phase 2 slice: a raw scan's ranges feed min_forward_range,
    whose output drives the state machine and gates the twist."""
    params = AebParams()
    sm = BrakeStateMachine(params)
    arc_half = math.radians(params.forward_arc_deg / 2.0)
    forward_cmd = (0.3, 0.0)

    # Scan with a clear forward arc (3 beams at -20, 0, +20 deg) -> command passes.
    clear_ranges = [3.0, 3.0, 3.0]
    min_range = min_forward_range(
        clear_ranges, angle_min=-math.radians(20),
        angle_increment=math.radians(20), range_min=0.05,
        arc_half_width_rad=arc_half,
    )
    braking = sm.update(min_range, now=0.0)
    assert gate_twist(*forward_cmd, braking=braking) == (0.3, 0.0)

    # Scan with an obstacle 0.25 m dead ahead -> forward zeroed.
    blocked_ranges = [3.0, 0.25, 3.0]
    min_range = min_forward_range(
        blocked_ranges, angle_min=-math.radians(20),
        angle_increment=math.radians(20), range_min=0.05,
        arc_half_width_rad=arc_half,
    )
    braking = sm.update(min_range, now=0.1)
    assert gate_twist(*forward_cmd, braking=braking) == (0.0, 0.0)
```

- [ ] **Step 2: Run test to verify it passes immediately**

Run: `python3 -m pytest tests/test_aeb_math.py -v`
Expected: 22 passed. Capstone scenario over already-implemented units; passes with no new code. If it fails, fix the offending unit before continuing.

- [ ] **Step 3: Verify the whole suite still passes**

Run: `python3 -m pytest -q`
Expected: `61 passed, 3 skipped`.

- [ ] **Step 4: Commit**

```bash
git add tests/test_aeb_math.py
git commit -m "test: add Phase 2 full-pipeline scenario capstone"
```

---

## Phase 3: Run As a Live ROS Node (Manual Hardware Verification)

**End state of phase:** a runnable `aeb_node.py` velocity gate, with every command source retargeted to `/cmd_vel_raw` so the brake node is the sole `/cmd_vel` publisher. **No automated tests** — `rclpy` is not installable on the dev laptop. Verification is `py_compile` for syntax plus manual checks on the rover.

### Task 3.1: Create `EmergencyBrakeNode` and register its entry point

**Files:**
- Create: `safety_controller_layer/aeb_node.py`
- Modify: `setup.py`

- [ ] **Step 1: Write the implementation (no unit test possible without rclpy)**

Create `safety_controller_layer/aeb_node.py`:
```python
"""ROS2 emergency-braking velocity gate.

Sits inline between every velocity source and the LIMO base driver: command
sources publish /cmd_vel_raw, this node republishes /cmd_vel. While an obstacle
is inside the forward arc of /scan, forward linear velocity is zeroed; rotation
and reverse pass through, so the rover can still turn or back away.

A fixed-rate timer is the single decision point: it computes the nearest
forward obstacle, updates the hysteresis state machine, gates the latest raw
command, and publishes. This decouples the output rate from the (possibly
bursty) input rates and guarantees the brake decision is always fresh.

Fail-safe: a missing/stale /scan brakes; a missing/stale /cmd_vel_raw is treated
as a zero command.

This module is intentionally untested locally: rclpy is not installable on the
development machine. Verify on hardware. The pure decision logic lives in
aeb_math.py and is fully unit-tested.
"""

import math
import sys
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

from safety_controller_layer.aeb_math import (
    AebParams,
    BrakeStateMachine,
    gate_twist,
    min_forward_range,
)

CMD_VEL_IN_TOPIC = "/cmd_vel_raw"
CMD_VEL_OUT_TOPIC = "/cmd_vel"
SCAN_TOPIC = "/scan"


class EmergencyBrakeNode(Node):
    """Velocity gate: republishes /cmd_vel_raw to /cmd_vel, braking on /scan."""

    def __init__(self, params: Optional[AebParams] = None):
        super().__init__("emergency_brake")
        self._params = params or AebParams()
        self._state = BrakeStateMachine(self._params)
        self._arc_half_width_rad = math.radians(self._params.forward_arc_deg / 2.0)

        self._latest_scan: Optional[LaserScan] = None
        self._scan_stamp: float = 0.0
        self._latest_cmd: Optional[Twist] = None
        self._cmd_stamp: float = 0.0
        self._was_braking = False

        self._cmd_pub = self.create_publisher(Twist, CMD_VEL_OUT_TOPIC, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, 10)
        self.create_subscription(Twist, CMD_VEL_IN_TOPIC, self._on_cmd, 10)
        self.create_timer(1.0 / self._params.output_rate_hz, self._tick)
        self.get_logger().info(
            f"EmergencyBrakeNode up: gating {CMD_VEL_IN_TOPIC} -> "
            f"{CMD_VEL_OUT_TOPIC} on {SCAN_TOPIC} obstacles."
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self._latest_scan = msg
        self._scan_stamp = time.monotonic()

    def _on_cmd(self, msg: Twist) -> None:
        self._latest_cmd = msg
        self._cmd_stamp = time.monotonic()

    def _tick(self) -> None:
        now = time.monotonic()

        # Obstacle distance -- fail-safe to 0.0 (brake) when scan missing/stale.
        if (
            self._latest_scan is None
            or now - self._scan_stamp > self._params.scan_timeout_s
        ):
            min_range = 0.0
        else:
            s = self._latest_scan
            min_range = min_forward_range(
                s.ranges, s.angle_min, s.angle_increment,
                s.range_min, self._arc_half_width_rad,
            )

        braking = self._state.update(min_range, now)

        # Raw command -- fail-safe to zero Twist when missing/stale.
        if (
            self._latest_cmd is None
            or now - self._cmd_stamp > self._params.command_timeout_s
        ):
            raw_linear, raw_angular = 0.0, 0.0
        else:
            raw_linear = self._latest_cmd.linear.x
            raw_angular = self._latest_cmd.angular.z

        linear, angular = gate_twist(raw_linear, raw_angular, braking)

        out = Twist()
        out.linear.x = linear
        out.angular.z = angular
        self._cmd_pub.publish(out)

        if braking != self._was_braking:
            if braking:
                self.get_logger().warn("BRAKE ENGAGED: obstacle in forward arc")
            else:
                self.get_logger().info("Brake released: forward path clear")
            self._was_braking = braking


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = EmergencyBrakeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted; stopping rover.")
    finally:
        try:
            node._cmd_pub.publish(Twist())
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Add the console entry point**

In `setup.py`, replace the `entry_points` block:
```python
    entry_points={
        "console_scripts": [
            "safety_controller_node = "
            "safety_controller_layer.safety_controller_node:main",
        ],
    },
```
with:
```python
    entry_points={
        "console_scripts": [
            "safety_controller_node = "
            "safety_controller_layer.safety_controller_node:main",
            "aeb_node = safety_controller_layer.aeb_node:main",
        ],
    },
```

- [ ] **Step 3: Verify the new module is syntactically valid**

Run: `python3 -m py_compile safety_controller_layer/aeb_node.py`
Expected: no output, exit code 0. (The `rclpy` import is not executed by `py_compile`, so this passes on the laptop.)

- [ ] **Step 4: Verify no tests regressed**

Run: `python3 -m pytest -q`
Expected: `61 passed, 3 skipped`. No test imports `aeb_node`, so the count is unchanged from Task 2.2.

- [ ] **Step 5: Commit**

```bash
git add safety_controller_layer/aeb_node.py setup.py
git commit -m "feat: add EmergencyBrakeNode ROS2 velocity gate"
```

---

### Task 3.2: Retarget command sources to `/cmd_vel_raw`

**Files:**
- Modify: `safety_controller_layer/safety_controller_node.py`
- Modify: `test_drive.py`
- Modify: `tests/test_execute_command_integration.py`

- [ ] **Step 1: Retarget the safety controller node output**

In `safety_controller_layer/safety_controller_node.py`, change the topic constant:
```python
CMD_VEL_TOPIC = "/cmd_vel"
```
to:
```python
CMD_VEL_TOPIC = "/cmd_vel_raw"
```

In the same file's module docstring, update the two `/cmd_vel` mentions for accuracy:
- `rover to the new heading then drives it forward, publishing /cmd_vel throughout`
  → `rover to the new heading then drives it forward, publishing /cmd_vel_raw throughout`
- `/cmd_vel (geometry_msgs/Twist).`
  → `/cmd_vel_raw (geometry_msgs/Twist).`

- [ ] **Step 2: Retarget the manual test-drive script**

In `test_drive.py`, change the topic constant:
```python
CMD_VEL_TOPIC = "/cmd_vel"
```
to:
```python
CMD_VEL_TOPIC = "/cmd_vel_raw"
```

In the same file's module docstring, update the `/cmd_vel` mention:
- `"""Manually publish Twist commands to /cmd_vel for rover bring-up testing.`
  → `"""Manually publish Twist commands to /cmd_vel_raw for rover bring-up testing.`

- [ ] **Step 3: Retarget the integration test's control-loop subscription**

In `tests/test_execute_command_integration.py`, the loop-closing node subscribes to whatever the controller publishes. Change line ~71:
```python
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
```
to:
```python
        self.create_subscription(Twist, "/cmd_vel_raw", self._on_cmd_vel, 10)
```

- [ ] **Step 4: Verify syntax of all three modified files**

Run:
```bash
python3 -m py_compile safety_controller_layer/safety_controller_node.py test_drive.py tests/test_execute_command_integration.py
```
Expected: no output, exit code 0.

- [ ] **Step 5: Verify no tests regressed**

Run: `python3 -m pytest -q`
Expected: `61 passed, 3 skipped`. The retarget changes only topic strings; the integration test is skipped on the laptop regardless, and on the rover it now correctly tracks the controller's `/cmd_vel_raw` output.

- [ ] **Step 6: Commit**

```bash
git add safety_controller_layer/safety_controller_node.py test_drive.py tests/test_execute_command_integration.py
git commit -m "refactor: retarget command sources to /cmd_vel_raw behind the AEB gate"
```

- [ ] **Step 7: Manual hardware verification (deferred until on the rover)**

On the LIMO Pro with ROS2 Foxy sourced, after `colcon build`:
```bash
# Terminal 1 -- LIMO base drivers (publishes /scan, consumes /cmd_vel)
ros2 launch limo_bringup limo_start.launch.py

# Terminal 2 -- confirm the lidar is publishing
ros2 topic hz /scan        # expect ~6-12 Hz

# Terminal 3 -- start the emergency brake gate
ros2 run safety_controller_layer aeb_node

# Terminal 4 -- drive forward toward a wall via the manual script
python3 test_drive.py      # publishes /cmd_vel_raw

# Terminal 5 -- observe the gated output
ros2 topic echo /cmd_vel
```
Expected behavior:
- With a clear path, `/cmd_vel` mirrors `/cmd_vel_raw`.
- Driving toward a wall, `/cmd_vel` `linear.x` drops to `0.0` once the wall is within ~0.40 m; the node logs `BRAKE ENGAGED`.
- While braked, sending a rotation command still turns the rover, and reverse still backs it up.
- After backing past ~0.60 m for ~0.5 s, the node logs `Brake released` and forward motion resumes.

---

## Self-Review

**Spec coverage:**
- ✅ Trigger source 2D lidar `/scan` → Task 3.1 subscribes `/scan`; Task 2.1 `min_forward_range`.
- ✅ Forward arc obstacle detection (±30°) → Task 1.1 `forward_arc_deg=60.0`; Task 3.1 derives `arc_half_width_rad`; Task 2.1 arc filter.
- ✅ Auto-release with hysteresis (trigger/release distances + dwell) → Task 1.1 params; Task 1.4 `BrakeStateMachine`.
- ✅ Brake scope: forward only, rotation/reverse pass → Task 1.2 `gate_twist`.
- ✅ Velocity gate, AEB sole `/cmd_vel` publisher → Task 3.1 node; Task 3.2 retargets all sources to `/cmd_vel_raw`.
- ✅ Lives in `safety_controller_layer` package → Tasks 3.1/3.2 file paths.
- ✅ Pure-logic / thin-wrapper split, laptop-testable logic → Phases 1–2 (`aeb_math.py`); Phase 3 (`aeb_node.py`, no laptop test).
- ✅ Fail-safe: missing/stale `/scan` brakes → Task 3.1 `_tick` (`min_range = 0.0`).
- ✅ Fail-safe: missing/stale `/cmd_vel_raw` → zero Twist → Task 3.1 `_tick`.
- ✅ Fail-safe: all-invalid fresh scan arc → clear (`inf`) → Task 2.1 returns `inf`.
- ✅ Shutdown publishes zero Twist → Task 3.1 `main()` `finally`.
- ✅ `AebParams` validates `release > trigger` → Task 1.1 `__post_init__`.
- ✅ TDD red-green-commit → every Phase 1–2 task; Phase 3 documents why no unit tests.
- ✅ Vertical slices per phase → P1 = decision from a distance; P2 = decision from real lidar; P3 = live ROS node.
- ✅ Setup entry point → Task 3.1 Step 2.

**Placeholder scan:** every step contains real code or a runnable command with expected output. No `TBD`/`TODO`/"add error handling".

**Type consistency:**
- `AebParams` field names (`trigger_distance_m`, `release_distance_m`, `release_dwell_s`, `forward_arc_deg`, `output_rate_hz`, `command_timeout_s`, `scan_timeout_s`) are stable from Task 1.1; later tasks only read them.
- `BrakeStateMachine(params)` constructor and `update(min_range, now) -> bool` signature are consistent across Tasks 1.3, 1.4, 1.5, 2.2, and the node in 3.1.
- `gate_twist(linear_x, angular_z, braking) -> Tuple[float, float]` signature consistent across Tasks 1.2, 1.5, 2.2, 3.1.
- `min_forward_range(ranges, angle_min, angle_increment, range_min, arc_half_width_rad) -> float` signature consistent across Tasks 2.1, 2.2, 3.1.
- Topic constants `CMD_VEL_IN_TOPIC="/cmd_vel_raw"`, `CMD_VEL_OUT_TOPIC="/cmd_vel"`, `SCAN_TOPIC="/scan"` (Task 3.1) match the retargeted `CMD_VEL_TOPIC="/cmd_vel_raw"` in Task 3.2.

---

## Test Count Progression (verification checkpoint)

| End of task | `pytest -q` |
|---|---|
| baseline | 39 passed, 3 skipped |
| 1.1 | 41 passed, 3 skipped |
| 1.2 | 45 passed, 3 skipped |
| 1.3 | 48 passed, 3 skipped |
| 1.4 | 53 passed, 3 skipped |
| 1.5 | 54 passed, 3 skipped |
| 2.1 | 60 passed, 3 skipped |
| 2.2 | 61 passed, 3 skipped |
| 3.1 | 61 passed, 3 skipped |
| 3.2 | 61 passed, 3 skipped |

If the count diverges, stop and investigate before continuing.
