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
