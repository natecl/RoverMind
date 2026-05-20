# Safety Controller Layer Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a ROS2 safety controller layer for the Agilex LIMO Pro that exposes `execute_command(heading_degree, distance_m)` — turning to a relative heading using IMU yaw feedback, then driving forward a measured distance using `/odom` displacement, with parameter clamps, per-phase timeouts, and clean stops between phases.

**Architecture:** Split into two files inside `safety_controller_layer/`. `control_math.py` contains all pure logic — the `ControllerParams` dataclass, math primitives (`wrap_angle`, `proportional_turn`, `displacement`), the `SafetyController` class with dependency-injected sensor/publisher/clock callables, and `ControllerTimeoutError`. `safety_controller_node.py` is a thin ROS2 node that subscribes `/imu` and `/odom`, publishes `/cmd_vel`, and wires `rclpy.spin_once`-driven callables into `SafetyController`. The pure-logic side has zero ROS imports and is fully unit-testable without `rclpy` (which is not available on the dev laptop).

**Tech Stack:** Python 3.13, pytest 8.x for unit tests. On the rover only: rclpy, sensor_msgs, nav_msgs, geometry_msgs (ROS2 Foxy).

**Locked-in design decisions** (from clarifying Q&A 2026-05-19):
- `heading_degree` is a **relative delta** from current yaw, not absolute world heading.
- Yaw source: `/imu` (sensor_msgs/Imu), yaw from orientation quaternion via in-house conversion (no `tf_transformations` dependency).
- Distance metric: Euclidean displacement from start pose snapshot at the entry of `drive_distance`.
- Spin strategy: single node, `rclpy.spin_once(timeout=0)` inside the 20 Hz loop, blocking `execute_command`.
- Test strategy: pure pytest with dependency-injected callables; the ROS node has no automated tests (manual hardware verification only).
- Drive phase publishes pure forward velocity (no heading correction).
- Distance exit: `traveled >= target` (no tolerance — worst-case overshoot ~1.5 cm at 0.3 m/s, 20 Hz).
- Per-phase timeout: nominal duration + safety margin, clamped to `[min_timeout_s, max_timeout_s]`; on expiry publish zero-Twist and raise `ControllerTimeoutError`.
- Parameters supplied via a frozen `ControllerParams` dataclass with spec defaults.
- Folder: `safety_controller_layer/` (snake_case Python package).

---

## File Structure

**Create:**
- `safety_controller_layer/__init__.py` — empty package marker
- `safety_controller_layer/control_math.py` — pure-logic core (ControllerParams, math primitives, SafetyController, ControllerTimeoutError)
- `safety_controller_layer/safety_controller_node.py` — ROS2 wrapper node + main() entry point
- `tests/__init__.py` — empty package marker
- `tests/conftest.py` — pytest config to add repo root to sys.path
- `tests/test_control_math.py` — unit tests for math primitives, params, exception
- `tests/test_controller.py` — unit tests for SafetyController rotate/drive/execute_command
- `pyproject.toml` — pytest configuration
- `requirements-dev.txt` — pytest pin

**Modify:** none (existing `test_drive.py` and `README.md` are untouched)

---

## Phase 0: Scaffolding

### Task 0.1: Repository scaffolding (one-time setup)

**Files:**
- Create: `safety_controller_layer/__init__.py`
- Create: `tests/__init__.py`
- Create: `tests/conftest.py`
- Create: `requirements-dev.txt`
- Create: `pyproject.toml`

- [ ] **Step 1: Install pytest into the active environment**

Run:
```bash
python3 -m pip install --user pytest==8.3.3
```
Expected: `Successfully installed pytest-8.3.3 ...` (or "already satisfied" if present).

- [ ] **Step 2: Create package markers**

Create `safety_controller_layer/__init__.py` containing a single newline (empty file).
Create `tests/__init__.py` containing a single newline (empty file).

- [ ] **Step 3: Create `tests/conftest.py`**

```python
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
```

- [ ] **Step 4: Create `requirements-dev.txt`**

```
pytest==8.3.3
```

- [ ] **Step 5: Create `pyproject.toml`**

```toml
[tool.pytest.ini_options]
testpaths = ["tests"]
python_files = ["test_*.py"]
```

- [ ] **Step 6: Verify pytest discovers no tests yet**

Run: `python3 -m pytest -v`
Expected: `no tests ran in 0.0Xs` (exit code 5). Confirms wiring is correct, just no tests to find.

- [ ] **Step 7: Commit**

```bash
git add safety_controller_layer/__init__.py tests/__init__.py tests/conftest.py requirements-dev.txt pyproject.toml
git commit -m "chore: scaffold safety_controller_layer package and pytest config"
```

---

## Phase 1: Rotate-to-Heading Vertical Slice

**End state of phase:** a pure-Python `SafetyController(params, get_yaw=..., publish_twist=..., sleep=..., now=...).rotate_to_heading(delta_rad)` that converges to the target yaw via proportional control, clamps angular velocity to `±max_angular`, exits within `heading_tolerance_rad`, publishes a zero-Twist stop on exit, and raises `ControllerTimeoutError` on stalls. All testable without ROS.

### Task 1.1: `ControllerParams` dataclass with spec defaults

**Files:**
- Create: `safety_controller_layer/control_math.py`
- Test: `tests/test_control_math.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_control_math.py`:
```python
import dataclasses

from safety_controller_layer.control_math import ControllerParams


def test_controller_params_defaults_match_spec():
    p = ControllerParams()
    assert p.max_linear == 0.3
    assert p.max_angular == 0.5
    assert p.heading_tolerance_rad == 0.035
    assert p.heading_kp == 1.0
    assert p.loop_rate_hz == 20.0


def test_controller_params_is_immutable():
    p = ControllerParams()
    try:
        p.max_linear = 0.9  # type: ignore[misc]
    except dataclasses.FrozenInstanceError:
        return
    raise AssertionError("ControllerParams should be frozen")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_control_math.py -v`
Expected: `ModuleNotFoundError: No module named 'safety_controller_layer.control_math'`.

- [ ] **Step 3: Write minimal implementation**

Create `safety_controller_layer/control_math.py`:
```python
import math
from dataclasses import dataclass
from typing import Callable, Tuple


@dataclass(frozen=True)
class ControllerParams:
    max_linear: float = 0.3
    max_angular: float = 0.5
    heading_tolerance_rad: float = 0.035
    heading_kp: float = 1.0
    loop_rate_hz: float = 20.0
    timeout_safety_margin_s: float = 2.0
    min_timeout_s: float = 1.0
    max_timeout_s: float = 30.0
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_control_math.py -v`
Expected: 2 passed.

- [ ] **Step 5: Commit**

```bash
git add safety_controller_layer/control_math.py tests/test_control_math.py
git commit -m "feat: add ControllerParams dataclass with spec defaults"
```

---

### Task 1.2: `wrap_angle` — shortest-path heading errors

**Files:**
- Modify: `safety_controller_layer/control_math.py`
- Test: `tests/test_control_math.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_control_math.py`:
```python
import math

from safety_controller_layer.control_math import wrap_angle


def test_wrap_angle_within_range_unchanged():
    assert wrap_angle(0.0) == 0.0
    assert wrap_angle(1.0) == 1.0
    assert wrap_angle(-1.0) == -1.0


def test_wrap_angle_handles_full_rotation():
    assert math.isclose(wrap_angle(2 * math.pi), 0.0, abs_tol=1e-9)
    assert math.isclose(wrap_angle(-2 * math.pi), 0.0, abs_tol=1e-9)


def test_wrap_angle_picks_short_path_past_pi():
    # 3*pi/2 (270 deg) should wrap to -pi/2 (-90 deg) — go the short way
    assert math.isclose(wrap_angle(3 * math.pi / 2), -math.pi / 2, abs_tol=1e-9)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_control_math.py -v`
Expected: `ImportError: cannot import name 'wrap_angle'`.

- [ ] **Step 3: Write minimal implementation**

Append to `safety_controller_layer/control_math.py` (below the `ControllerParams` dataclass):
```python
def wrap_angle(rad: float) -> float:
    """Wrap an angle in radians to the range [-pi, pi)."""
    return ((rad + math.pi) % (2 * math.pi)) - math.pi
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_control_math.py -v`
Expected: 5 passed.

- [ ] **Step 5: Commit**

```bash
git add safety_controller_layer/control_math.py tests/test_control_math.py
git commit -m "feat: add wrap_angle for shortest-path heading errors"
```

---

### Task 1.3: `proportional_turn` — `Kp * error` clamped to `±max_angular`

**Files:**
- Modify: `safety_controller_layer/control_math.py`
- Test: `tests/test_control_math.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_control_math.py`:
```python
from safety_controller_layer.control_math import proportional_turn


def test_proportional_turn_sign_matches_error():
    p = ControllerParams()
    assert proportional_turn(error_rad=0.5, params=p) > 0
    assert proportional_turn(error_rad=-0.5, params=p) < 0


def test_proportional_turn_clamps_to_max_angular():
    p = ControllerParams(max_angular=0.5, heading_kp=1.0)
    assert proportional_turn(error_rad=10.0, params=p) == 0.5
    assert proportional_turn(error_rad=-10.0, params=p) == -0.5


def test_proportional_turn_unclamped_in_linear_region():
    p = ControllerParams(max_angular=0.5, heading_kp=1.0)
    assert math.isclose(proportional_turn(error_rad=0.2, params=p), 0.2)


def test_proportional_turn_respects_higher_kp():
    p = ControllerParams(max_angular=10.0, heading_kp=2.0)
    assert math.isclose(proportional_turn(error_rad=0.5, params=p), 1.0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_control_math.py -v`
Expected: `ImportError: cannot import name 'proportional_turn'`.

- [ ] **Step 3: Write minimal implementation**

Append to `safety_controller_layer/control_math.py`:
```python
def proportional_turn(error_rad: float, params: ControllerParams) -> float:
    """P-controller output for heading, clamped to +/- max_angular."""
    raw = params.heading_kp * error_rad
    return max(-params.max_angular, min(params.max_angular, raw))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_control_math.py -v`
Expected: 9 passed.

- [ ] **Step 5: Commit**

```bash
git add safety_controller_layer/control_math.py tests/test_control_math.py
git commit -m "feat: add proportional_turn with angular velocity clamp"
```

---

### Task 1.4: `SafetyController.rotate_to_heading` — dependency-injected loop

**Files:**
- Modify: `safety_controller_layer/control_math.py`
- Test: `tests/test_controller.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_controller.py`:
```python
import math

import pytest

from safety_controller_layer.control_math import (
    ControllerParams,
    SafetyController,
)


class FakeWorld:
    """Simulates a perfect rover whose yaw advances at the last commanded angular velocity."""

    def __init__(self, initial_yaw=0.0):
        self.yaw = initial_yaw
        self.position = (0.0, 0.0)
        self.published = []  # list of (linear_x, angular_z)
        self.time = 0.0

    def get_yaw(self):
        return self.yaw

    def get_position(self):
        return self.position

    def publish(self, linear_x, angular_z):
        self.published.append((linear_x, angular_z))

    def sleep(self, seconds):
        last_angular = self.published[-1][1] if self.published else 0.0
        self.yaw += last_angular * seconds
        self.time += seconds

    def now(self):
        return self.time


def _make_controller(world, params=None):
    return SafetyController(
        params=params or ControllerParams(),
        get_yaw=world.get_yaw,
        get_position=world.get_position,
        publish_twist=world.publish,
        sleep=world.sleep,
        now=world.now,
    )


def test_rotate_to_heading_converges_within_tolerance():
    world = FakeWorld(initial_yaw=0.0)
    controller = _make_controller(world)
    controller.rotate_to_heading(math.radians(30))
    assert abs(math.radians(30) - world.yaw) < ControllerParams().heading_tolerance_rad


def test_rotate_to_heading_publishes_zero_stop_on_exit():
    world = FakeWorld(initial_yaw=0.0)
    controller = _make_controller(world)
    controller.rotate_to_heading(math.radians(30))
    assert world.published[-1] == (0.0, 0.0)


def test_rotate_to_heading_clamps_angular_velocity():
    world = FakeWorld(initial_yaw=0.0)
    controller = _make_controller(world)
    controller.rotate_to_heading(math.radians(90))
    max_seen = max(abs(a) for _, a in world.published)
    assert max_seen <= ControllerParams().max_angular + 1e-9


def test_rotate_to_heading_zero_delta_is_immediate_stop():
    world = FakeWorld(initial_yaw=0.0)
    controller = _make_controller(world)
    controller.rotate_to_heading(0.0)
    assert world.published == [(0.0, 0.0)]


def test_rotate_to_heading_publishes_no_linear_motion():
    world = FakeWorld(initial_yaw=0.0)
    controller = _make_controller(world)
    controller.rotate_to_heading(math.radians(45))
    assert all(linear == 0.0 for linear, _ in world.published)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_controller.py -v`
Expected: `ImportError: cannot import name 'SafetyController'`.

- [ ] **Step 3: Write minimal implementation**

Append to `safety_controller_layer/control_math.py`:
```python
class SafetyController:
    def __init__(
        self,
        params: ControllerParams,
        get_yaw: Callable[[], float],
        get_position: Callable[[], Tuple[float, float]],
        publish_twist: Callable[[float, float], None],
        sleep: Callable[[float], None],
        now: Callable[[], float],
    ):
        self.params = params
        self._get_yaw = get_yaw
        self._get_position = get_position
        self._publish = publish_twist
        self._sleep = sleep
        self._now = now

    def rotate_to_heading(self, heading_delta_rad: float) -> None:
        params = self.params
        period = 1.0 / params.loop_rate_hz
        start_yaw = self._get_yaw()
        target_yaw = start_yaw + heading_delta_rad
        while True:
            error = wrap_angle(target_yaw - self._get_yaw())
            if abs(error) < params.heading_tolerance_rad:
                self._publish(0.0, 0.0)
                return
            angular = proportional_turn(error, params)
            self._publish(0.0, angular)
            self._sleep(period)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_controller.py -v`
Expected: 5 passed.

- [ ] **Step 5: Commit**

```bash
git add safety_controller_layer/control_math.py tests/test_controller.py
git commit -m "feat: add SafetyController.rotate_to_heading with proportional control"
```

---

### Task 1.5: Per-phase timeout for `rotate_to_heading`

**Files:**
- Modify: `safety_controller_layer/control_math.py`
- Test: `tests/test_controller.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_controller.py`:
```python
from safety_controller_layer.control_math import ControllerTimeoutError


class StuckYawWorld(FakeWorld):
    """IMU is frozen — yaw never updates regardless of commanded velocity."""

    def sleep(self, seconds):
        self.time += seconds  # advance clock, do NOT update yaw


def test_rotate_to_heading_raises_on_timeout():
    world = StuckYawWorld(initial_yaw=0.0)
    controller = _make_controller(world)
    with pytest.raises(ControllerTimeoutError):
        controller.rotate_to_heading(math.radians(30))


def test_rotate_to_heading_publishes_stop_before_raising_on_timeout():
    world = StuckYawWorld(initial_yaw=0.0)
    controller = _make_controller(world)
    with pytest.raises(ControllerTimeoutError):
        controller.rotate_to_heading(math.radians(30))
    assert world.published[-1] == (0.0, 0.0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_controller.py -v`
Expected: `ImportError: cannot import name 'ControllerTimeoutError'`.

- [ ] **Step 3: Write minimal implementation**

In `safety_controller_layer/control_math.py`, add the exception class above `SafetyController`:
```python
class ControllerTimeoutError(Exception):
    """Raised when a control phase exceeds its computed timeout budget."""
```

Add the timeout helper above `SafetyController`:
```python
def rotate_timeout_seconds(heading_delta_rad: float, params: ControllerParams) -> float:
    nominal = abs(heading_delta_rad) / params.max_angular if params.max_angular > 0 else params.max_timeout_s
    budget = nominal + params.timeout_safety_margin_s
    return max(params.min_timeout_s, min(params.max_timeout_s, budget))
```

Replace the body of `SafetyController.rotate_to_heading` with the timeout-aware version:
```python
    def rotate_to_heading(self, heading_delta_rad: float) -> None:
        params = self.params
        period = 1.0 / params.loop_rate_hz
        start_yaw = self._get_yaw()
        target_yaw = start_yaw + heading_delta_rad
        deadline = self._now() + rotate_timeout_seconds(heading_delta_rad, params)
        while True:
            error = wrap_angle(target_yaw - self._get_yaw())
            if abs(error) < params.heading_tolerance_rad:
                self._publish(0.0, 0.0)
                return
            if self._now() >= deadline:
                self._publish(0.0, 0.0)
                raise ControllerTimeoutError(
                    f"rotate_to_heading did not converge within budget "
                    f"(target_delta={heading_delta_rad:.3f} rad)"
                )
            angular = proportional_turn(error, params)
            self._publish(0.0, angular)
            self._sleep(period)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_controller.py -v`
Expected: 7 passed (5 from 1.4 + 2 new).

- [ ] **Step 5: Commit**

```bash
git add safety_controller_layer/control_math.py tests/test_controller.py
git commit -m "feat: add per-phase timeout to rotate_to_heading"
```

---

## Phase 2: Drive-Distance Vertical Slice

**End state of phase:** `SafetyController.drive_distance(distance_m)` drives forward at `max_linear` until Euclidean displacement from start position meets `distance_m`, publishes zero stop on exit, and raises `ControllerTimeoutError` on stalls.

### Task 2.1: `displacement` math primitive

**Files:**
- Modify: `safety_controller_layer/control_math.py`
- Test: `tests/test_control_math.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_control_math.py`:
```python
from safety_controller_layer.control_math import displacement


def test_displacement_zero_for_same_point():
    assert displacement((0.0, 0.0), (0.0, 0.0)) == 0.0


def test_displacement_pure_x():
    assert math.isclose(displacement((0.0, 0.0), (1.5, 0.0)), 1.5)


def test_displacement_diagonal_uses_euclidean():
    assert math.isclose(displacement((0.0, 0.0), (3.0, 4.0)), 5.0)


def test_displacement_is_symmetric():
    assert math.isclose(
        displacement((1.0, 2.0), (4.0, 6.0)),
        displacement((4.0, 6.0), (1.0, 2.0)),
    )
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_control_math.py -v`
Expected: `ImportError: cannot import name 'displacement'`.

- [ ] **Step 3: Write minimal implementation**

Append to `safety_controller_layer/control_math.py`:
```python
def displacement(start_xy: Tuple[float, float], current_xy: Tuple[float, float]) -> float:
    """Euclidean distance between two 2D points."""
    dx = current_xy[0] - start_xy[0]
    dy = current_xy[1] - start_xy[1]
    return math.sqrt(dx * dx + dy * dy)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_control_math.py -v`
Expected: 13 passed.

- [ ] **Step 5: Commit**

```bash
git add safety_controller_layer/control_math.py tests/test_control_math.py
git commit -m "feat: add Euclidean displacement primitive"
```

---

### Task 2.2: `SafetyController.drive_distance` with displacement-based exit

**Files:**
- Modify: `safety_controller_layer/control_math.py`
- Test: `tests/test_controller.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_controller.py`:
```python
class DrivingWorld(FakeWorld):
    """Rover moves in +x at exactly the commanded linear velocity (yaw frozen at 0)."""

    def sleep(self, seconds):
        last_linear = self.published[-1][0] if self.published else 0.0
        x, y = self.position
        self.position = (x + last_linear * seconds, y)
        self.time += seconds


def test_drive_distance_reaches_target():
    world = DrivingWorld()
    controller = _make_controller(world)
    controller.drive_distance(1.0)
    traveled = math.hypot(world.position[0], world.position[1])
    assert traveled >= 1.0


def test_drive_distance_publishes_zero_stop_on_exit():
    world = DrivingWorld()
    controller = _make_controller(world)
    controller.drive_distance(1.0)
    assert world.published[-1] == (0.0, 0.0)


def test_drive_distance_uses_max_linear_during_drive():
    world = DrivingWorld()
    controller = _make_controller(world)
    controller.drive_distance(1.0)
    p = ControllerParams()
    for linear, angular in world.published[:-1]:
        assert linear == p.max_linear
        assert angular == 0.0


def test_drive_distance_zero_target_is_immediate_stop():
    world = DrivingWorld()
    controller = _make_controller(world)
    controller.drive_distance(0.0)
    assert world.published == [(0.0, 0.0)]


def test_drive_distance_overshoot_bounded_by_one_tick():
    world = DrivingWorld()
    controller = _make_controller(world)
    controller.drive_distance(1.0)
    p = ControllerParams()
    max_overshoot = p.max_linear * (1.0 / p.loop_rate_hz)
    traveled = math.hypot(world.position[0], world.position[1])
    assert traveled - 1.0 < max_overshoot + 1e-9
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_controller.py -v`
Expected: `AttributeError: 'SafetyController' object has no attribute 'drive_distance'`.

- [ ] **Step 3: Write minimal implementation**

Add the drive-timeout helper above `SafetyController` in `safety_controller_layer/control_math.py`:
```python
def drive_timeout_seconds(distance_m: float, params: ControllerParams) -> float:
    nominal = abs(distance_m) / params.max_linear if params.max_linear > 0 else params.max_timeout_s
    budget = nominal + params.timeout_safety_margin_s
    return max(params.min_timeout_s, min(params.max_timeout_s, budget))
```

Add the method to `SafetyController`:
```python
    def drive_distance(self, distance_m: float) -> None:
        params = self.params
        period = 1.0 / params.loop_rate_hz
        start_xy = self._get_position()
        deadline = self._now() + drive_timeout_seconds(distance_m, params)
        while True:
            traveled = displacement(start_xy, self._get_position())
            if traveled >= distance_m:
                self._publish(0.0, 0.0)
                return
            if self._now() >= deadline:
                self._publish(0.0, 0.0)
                raise ControllerTimeoutError(
                    f"drive_distance did not converge within budget "
                    f"(target_distance={distance_m:.3f} m)"
                )
            self._publish(params.max_linear, 0.0)
            self._sleep(period)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_controller.py -v`
Expected: 12 passed (7 from Phase 1 + 5 new).

- [ ] **Step 5: Commit**

```bash
git add safety_controller_layer/control_math.py tests/test_controller.py
git commit -m "feat: add SafetyController.drive_distance with displacement-based exit"
```

---

### Task 2.3: Lock in `drive_distance` timeout behavior with explicit tests

**Files:**
- Test: `tests/test_controller.py`

- [ ] **Step 1: Write the failing test (or confirm green if 2.2 implemented timeout correctly)**

Append to `tests/test_controller.py`:
```python
class StuckOdomWorld(FakeWorld):
    """Odom is frozen — position never updates regardless of commanded velocity."""

    def sleep(self, seconds):
        self.time += seconds  # advance clock, do NOT update position


def test_drive_distance_raises_on_timeout():
    world = StuckOdomWorld()
    controller = _make_controller(world)
    with pytest.raises(ControllerTimeoutError):
        controller.drive_distance(1.0)


def test_drive_distance_publishes_stop_before_raising_on_timeout():
    world = StuckOdomWorld()
    controller = _make_controller(world)
    with pytest.raises(ControllerTimeoutError):
        controller.drive_distance(1.0)
    assert world.published[-1] == (0.0, 0.0)
```

- [ ] **Step 2: Run tests**

Run: `python3 -m pytest tests/test_controller.py -v`
Expected: 14 passed. If the new tests fail, the timeout block in `drive_distance` (Task 2.2) is wrong — fix it before continuing.

- [ ] **Step 3: Commit**

```bash
git add tests/test_controller.py
git commit -m "test: lock in drive_distance timeout behavior"
```

---

## Phase 3: `execute_command` Orchestration

**End state of phase:** `SafetyController.execute_command(heading_degree, distance_m)` converts degrees to radians, calls `rotate_to_heading` then `drive_distance` in sequence. The zero-Twist between phases is guaranteed by each method's own stop-on-exit. `ControllerTimeoutError` propagates from either phase.

### Task 3.1: `execute_command` chains rotate then drive (degrees in, radians internally)

**Files:**
- Modify: `safety_controller_layer/control_math.py`
- Test: `tests/test_controller.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_controller.py`:
```python
class FullWorld(FakeWorld):
    """Rotates during turn phase, drives along current yaw during drive phase."""

    def sleep(self, seconds):
        last_linear, last_angular = self.published[-1] if self.published else (0.0, 0.0)
        self.yaw += last_angular * seconds
        x, y = self.position
        self.position = (
            x + last_linear * math.cos(self.yaw) * seconds,
            y + last_linear * math.sin(self.yaw) * seconds,
        )
        self.time += seconds


def test_execute_command_rotates_then_drives():
    world = FullWorld()
    controller = _make_controller(world)
    controller.execute_command(heading_degree=90.0, distance_m=1.0)
    assert abs(math.radians(90) - world.yaw) < ControllerParams().heading_tolerance_rad
    assert math.hypot(world.position[0], world.position[1]) >= 1.0


def test_execute_command_publishes_stop_between_phases():
    world = FullWorld()
    controller = _make_controller(world)
    controller.execute_command(heading_degree=45.0, distance_m=0.5)
    # final command is a stop (drive's exit-stop)
    assert world.published[-1] == (0.0, 0.0)
    # the moment the first non-zero linear appears, the immediately preceding command must be a stop (rotate's exit-stop)
    drive_start_idx = next(
        (i for i, (lin, _) in enumerate(world.published) if lin > 0.0),
        None,
    )
    assert drive_start_idx is not None
    assert world.published[drive_start_idx - 1] == (0.0, 0.0)


def test_execute_command_zero_zero_is_double_stop():
    world = FullWorld()
    controller = _make_controller(world)
    controller.execute_command(heading_degree=0.0, distance_m=0.0)
    assert world.published == [(0.0, 0.0), (0.0, 0.0)]


def test_execute_command_propagates_rotate_timeout():
    world = StuckYawWorld()
    controller = _make_controller(world)
    with pytest.raises(ControllerTimeoutError):
        controller.execute_command(heading_degree=30.0, distance_m=1.0)


def test_execute_command_negative_heading_turns_other_way():
    world = FullWorld()
    controller = _make_controller(world)
    controller.execute_command(heading_degree=-45.0, distance_m=0.0)
    assert abs(math.radians(-45) - world.yaw) < ControllerParams().heading_tolerance_rad
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_controller.py -v`
Expected: `AttributeError: 'SafetyController' object has no attribute 'execute_command'`.

- [ ] **Step 3: Write minimal implementation**

Append to `SafetyController` in `safety_controller_layer/control_math.py`:
```python
    def execute_command(self, heading_degree: float, distance_m: float) -> None:
        self.rotate_to_heading(math.radians(heading_degree))
        self.drive_distance(distance_m)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_controller.py -v`
Expected: 19 passed (14 from Phases 1–2 + 5 new).

- [ ] **Step 5: Commit**

```bash
git add safety_controller_layer/control_math.py tests/test_controller.py
git commit -m "feat: add SafetyController.execute_command orchestrating rotate then drive"
```

---

## Phase 4: ROS2 Node Wrapper (Manual Hardware Verification)

**End state of phase:** a runnable `safety_controller_layer/safety_controller_node.py` that subscribes `/imu` and `/odom`, publishes `/cmd_vel`, and wraps `SafetyController` with `rclpy.spin_once`-driven callables. **No automated tests** — `rclpy` cannot be installed on this dev laptop. Verification is manual on the rover.

### Task 4.1: ROS node skeleton wiring `SafetyController` to ROS topics

**Files:**
- Create: `safety_controller_layer/safety_controller_node.py`

- [ ] **Step 1: Write the implementation (no unit test possible without rclpy)**

Create `safety_controller_layer/safety_controller_node.py`:
```python
"""ROS2 wrapper around SafetyController.

Subscribes /imu (sensor_msgs/Imu) and /odom (nav_msgs/Odometry), publishes
/cmd_vel (geometry_msgs/Twist). Bridges rclpy.spin_once-driven sensor reads
into the pure-logic SafetyController.

Unit tests for this file are intentionally omitted: rclpy is not available
on the development machine. Verify on hardware using the main() entry point.
"""

import math
import sys
import time
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from safety_controller_layer.control_math import (
    ControllerParams,
    ControllerTimeoutError,
    SafetyController,
)

CMD_VEL_TOPIC = "/cmd_vel"
IMU_TOPIC = "/imu"
ODOM_TOPIC = "/odom"


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw from a quaternion (ZYX Euler convention, no external deps)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SafetyControllerNode(Node):
    def __init__(self, params: Optional[ControllerParams] = None):
        super().__init__("safety_controller")
        self._params = params or ControllerParams()
        self._latest_yaw: Optional[float] = None
        self._latest_xy: Optional[Tuple[float, float]] = None

        self._cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(Imu, IMU_TOPIC, self._on_imu, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)

        self._controller = SafetyController(
            params=self._params,
            get_yaw=self._read_yaw,
            get_position=self._read_position,
            publish_twist=self._publish_twist,
            sleep=self._spin_sleep,
            now=time.monotonic,
        )

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        self._latest_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._latest_xy = (p.x, p.y)

    def _read_yaw(self) -> float:
        rclpy.spin_once(self, timeout_sec=0.0)
        if self._latest_yaw is None:
            raise RuntimeError(f"No IMU message received on {IMU_TOPIC} yet")
        return self._latest_yaw

    def _read_position(self) -> Tuple[float, float]:
        rclpy.spin_once(self, timeout_sec=0.0)
        if self._latest_xy is None:
            raise RuntimeError(f"No odometry message received on {ODOM_TOPIC} yet")
        return self._latest_xy

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)

    def _spin_sleep(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while True:
            remaining = end - time.monotonic()
            if remaining <= 0.0:
                return
            rclpy.spin_once(self, timeout_sec=remaining)

    def execute_command(self, heading_degree: float, distance_m: float) -> None:
        self._controller.execute_command(heading_degree, distance_m)


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = SafetyControllerNode()
    try:
        node.get_logger().info("Waiting for first /imu and /odom messages...")
        while node._latest_yaw is None or node._latest_xy is None:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.get_logger().info(
            "Sensors ready. SafetyControllerNode is up; "
            "import and call node.execute_command(heading_degree, distance_m) from another script."
        )
        rclpy.spin(node)
    except (KeyboardInterrupt, ControllerTimeoutError) as e:
        node.get_logger().warn(f"Stopping: {e!r}")
    finally:
        try:
            node._publish_twist(0.0, 0.0)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Verify the file is syntactically valid Python (rclpy import will fail at runtime — that's expected on this laptop)**

Run: `python3 -m py_compile safety_controller_layer/safety_controller_node.py`
Expected: no output, exit code 0. Confirms syntax is valid even without `rclpy` installed.

- [ ] **Step 3: Verify no other tests regress**

Run: `python3 -m pytest -v`
Expected: 32 passed (13 in `test_control_math.py` + 19 in `test_controller.py` — the cumulative total from Phases 1–3). The ROS file is not imported by any test.

- [ ] **Step 4: Commit**

```bash
git add safety_controller_layer/safety_controller_node.py
git commit -m "feat: add SafetyControllerNode ROS2 wrapper for SafetyController"
```

- [ ] **Step 5: Manual hardware verification (deferred until on the rover)**

On the LIMO Pro with ROS2 Foxy sourced:
```bash
# Terminal 1 — start LIMO base drivers
ros2 launch limo_bringup limo_start.launch.py

# Terminal 2 — confirm IMU and odom are publishing at expected rates
ros2 topic hz /imu     # expect ~100 Hz
ros2 topic hz /odom    # expect ~20-50 Hz

# Terminal 3 — start the safety controller node
cd ~/RoverMind
python3 -m safety_controller_layer.safety_controller_node

# Terminal 4 — write a small driver script that imports SafetyControllerNode and
# calls execute_command(30.0, 0.5); run it after Terminal 3 reports "Sensors ready"

# Terminal 5 — observe /cmd_vel to verify clamps
ros2 topic echo /cmd_vel
```
Expected: rover rotates ~30° then drives ~0.5 m forward. `/cmd_vel` shows `linear.x` ≤ 0.3 and `angular.z` ≤ 0.5 throughout, with a clean `(0.0, 0.0)` Twist between rotate and drive phases and at the end.

---

## Self-Review

**Spec coverage:**
- ✅ Read current yaw from IMU → Task 4.1 subscribes `/imu`; Task 1.4 reads it via `get_yaw` callable
- ✅ Compute target yaw = current + desired heading change → Task 1.4: `target_yaw = start_yaw + heading_delta_rad`
- ✅ Publish angular velocity proportional to error → Task 1.3 `proportional_turn`, used in Task 1.4
- ✅ Tolerance ~2° (0.035 rad) → Task 1.1 default `heading_tolerance_rad=0.035`; Task 1.4 exit check
- ✅ Forward velocity for desired distance → Task 2.2 publishes `max_linear` until `traveled >= target`
- ✅ Distance estimated from `/odom` → Task 4.1 subscribes `/odom`; Task 2.1 `displacement`; Task 2.2 uses it
- ✅ Parameter clamps (max_linear=0.3, max_angular=0.5, heading_tolerance=0.035, heading_kp=1.0) → Task 1.1
- ✅ `angular_vel = KP * heading_error` clamped to `max_angular` → Task 1.3
- ✅ `execute_command(heading_degree, distance_m)` → Task 3.1
- ✅ Two blocks: `rotate_to_heading(heading_degree)` then `drive_distance(distance_m)` → Tasks 1.4, 2.2, 3.1
- ✅ ~20 Hz tight loop reading sensor, computing error, publishing velocity, checking exit → Task 1.1 default `loop_rate_hz=20.0`; loop bodies in 1.4 and 2.2
- ✅ TDD red-green-commit per task → every task in Phases 0–3; Phase 4 documents why no unit tests
- ✅ Vertical slices per phase → Phase 1 = full rotate end-to-end; Phase 2 = full drive end-to-end; Phase 3 = full command end-to-end; Phase 4 = ROS integration

**Placeholder scan:** every step contains actual code or a runnable command with expected output. No `TBD` / `TODO` / "add appropriate error handling" / "fill in details".

**Type consistency:**
- `SafetyController.__init__` callable parameter names (`get_yaw`, `get_position`, `publish_twist`, `sleep`, `now`) are identical across the test factory `_make_controller` (introduced in Task 1.4) and the ROS wrapper (Task 4.1).
- `ControllerParams` field names are stable from Task 1.1 onward; later tasks only read them, never rename.
- `ControllerTimeoutError` name is consistent across Tasks 1.5, 2.3, 3.1, and 4.1.
- `displacement(start_xy, current_xy)` signature consistent between Tasks 2.1 and 2.2.
- `rotate_timeout_seconds` / `drive_timeout_seconds` helper names consistent.

---

## Test Count Progression (verification checkpoint)

| End of phase | `pytest -v` total passed |
|---|---|
| 0.1 | 0 (no tests collected) |
| 1.1 | 2 |
| 1.2 | 5 |
| 1.3 | 9 |
| 1.4 | 14 (5 controller tests added) |
| 1.5 | 16 (2 timeout tests added) |
| 2.1 | 20 (4 displacement tests added) |
| 2.2 | 25 (5 drive tests added) |
| 2.3 | 27 (2 drive-timeout tests added) |
| 3.1 | 32 (5 execute_command tests added) |
| 4.1 | 32 (ROS file adds no tests) |

If the count diverges, stop and investigate before continuing.

---

## Post-Plan Amendments (recorded during execution)

The plan above was executed as written through Task 3.1. Phase 4 and the
final review changed the ROS layer beyond the original spin-once skeleton:

- **Task 4.1 as planned** shipped a `main()` that called `rclpy.spin()` with no
  way to invoke `execute_command` — the final whole-branch review flagged this
  as Critical (the node could not actually drive the rover).
- **ROS wrapper reworked into an action server.** `safety_controller_node.py`
  now exposes an `ExecuteCommand` ROS2 action (goal: `heading_degree`,
  `distance_m`; result: `success`, `message`; feedback: `phase`). It uses a
  `MultiThreadedExecutor` (≥3 threads) + `ReentrantCallbackGroup` so the
  `/imu` and `/odom` callbacks keep running while the blocking control loop
  executes. The action interface is defined in
  `safety_controller_layer_interfaces/action/ExecuteCommand.action`; generating
  it into a buildable ament interfaces package is a deferred follow-up, so the
  node will not import until that build exists.
- **Task C1 — cancellation in the core.** `SafetyController` gained an optional
  `should_abort` predicate and a `ControllerCancelledError`. The rotate/drive
  loops poll it each tick (after convergence, before timeout). +6 tests → 39.
- **Task C2 — cancellation in the node.** The action server accepts cancel
  requests and wires `should_abort` to the active goal handle's cancel flag, so
  a maneuver stops within one control tick and reports the goal `canceled()`.
- **Final-review fixes folded in:** drive-phase timeout test for
  `execute_command` (39th test), negative-`distance_m` rejection, explicit
  executor thread count, guarded `destroy_node()`.

**Final test count: 39 passed.** Outstanding follow-ups (non-blocking): scaffold
the ROS2 interfaces package so the action interface builds; document/lock the
GIL-atomicity assumption on the shared goal handle; add an integration test for
the ROS layer once a mock `rclpy` shim or hardware is available.
