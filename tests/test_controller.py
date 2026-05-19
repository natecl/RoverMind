import math

import pytest

from safety_controller_layer.control_math import (
    ControllerParams,
    ControllerTimeoutError,
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


class StuckYawWorld(FakeWorld):
    """IMU is frozen — yaw never updates regardless of commanded velocity."""

    def sleep(self, seconds):
        self.time += seconds  # advance clock, do NOT update yaw


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
