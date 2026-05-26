"""Smoke tests for launch/rovermind.launch.py.

`launch` and `launch_ros` ship with ROS2, not with pip, so these tests are
skipped on the dev laptop and exercised inside `colcon test`. Same pattern as
test_ros_package_loads.py.
"""

import importlib.util
from pathlib import Path

import pytest

pytest.importorskip("launch_ros")

from launch import LaunchContext, LaunchDescription
from launch_ros.actions import Node

REPO_ROOT = Path(__file__).resolve().parents[1]
LAUNCH_FILE = REPO_ROOT / "launch" / "rovermind.launch.py"


def _load_launch_module():
    spec = importlib.util.spec_from_file_location("rovermind_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _node_executables(description: LaunchDescription):
    # Node.node_executable is a list of Substitution objects (typically a
    # single TextSubstitution), not a plain string. Resolve through a
    # LaunchContext so the test compares against the actual text.
    ctx = LaunchContext()
    return [
        "".join(sub.perform(ctx) for sub in action.node_executable)
        for action in description.entities
        if isinstance(action, Node)
    ]


def test_generate_launch_description_includes_both_nodes():
    module = _load_launch_module()
    description = module.generate_launch_description()
    executables = _node_executables(description)
    assert "safety_controller_node" in executables
    assert "aeb_node" in executables


def test_use_aeb_arg_is_declared():
    from launch.actions import DeclareLaunchArgument

    module = _load_launch_module()
    description = module.generate_launch_description()
    declared = [
        a.name for a in description.entities
        if isinstance(a, DeclareLaunchArgument)
    ]
    assert "use_aeb" in declared


def test_node_exit_shuts_down_the_launch():
    """A node death must take the whole launch down so half-stack states are
    impossible. We assert (at parse time) that each Node has an
    OnProcessExit handler registered that emits a Shutdown."""
    from launch.actions import RegisterEventHandler, Shutdown
    from launch.event_handlers import OnProcessExit

    module = _load_launch_module()
    description = module.generate_launch_description()
    handlers = [
        a for a in description.entities
        if isinstance(a, RegisterEventHandler)
    ]
    on_exit_handlers = [
        h.event_handler for h in handlers
        if isinstance(h.event_handler, OnProcessExit)
    ]
    assert len(on_exit_handlers) >= 2, (
        "expected at least one OnProcessExit per node so any node death "
        "tears down the whole launch"
    )
    for h in on_exit_handlers:
        # OnActionEventBase stores its on_exit actions in a name-mangled
        # private attribute; the public way to surface them is describe(),
        # which returns (description_str, [actions]). The return type
        # permits nested iterables, so flatten one level before checking.
        _, raw_actions = h.describe()
        actions = []
        for a in raw_actions:
            if isinstance(a, (list, tuple)):
                actions.extend(a)
            else:
                actions.append(a)
        assert any(isinstance(a, Shutdown) for a in actions), (
            "each OnProcessExit must emit a Shutdown so the launch "
            "terminates loudly"
        )
