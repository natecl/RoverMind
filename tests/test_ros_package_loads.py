"""Smoke tests: the ROS2 interfaces package built and the node package loads.

Verifies the generated ExecuteCommand action type is importable with the
expected goal/result/feedback fields, and that SafetyControllerNode constructs
and registers its action server without raising.

Runnable only inside a built + sourced ROS2 workspace (`colcon test`). Off the
robot, where rclpy is not installed, the whole module is skipped.
"""

import pytest

# ROS2 is not installable on the dev laptop; skip cleanly when absent.
pytest.importorskip("rclpy")


def test_execute_command_interface_imports_with_expected_fields():
    """The colcon-generated action type carries the fields the node relies on."""
    from safety_controller_layer_interfaces.action import ExecuteCommand

    goal = ExecuteCommand.Goal()
    assert hasattr(goal, "heading_degree")
    assert hasattr(goal, "distance_m")

    result = ExecuteCommand.Result()
    assert hasattr(result, "success")
    assert hasattr(result, "message")

    feedback = ExecuteCommand.Feedback()
    assert hasattr(feedback, "phase")


def test_node_module_imports():
    """The node module imports — i.e. rclpy and the generated interface resolve."""
    from safety_controller_layer import safety_controller_node

    assert hasattr(safety_controller_node, "SafetyControllerNode")
    assert callable(safety_controller_node.main)


def test_node_constructs_and_registers_action_server():
    """Constructing the node wires its publishers, subscriptions and the
    ExecuteCommand action server without raising."""
    import rclpy
    from safety_controller_layer.safety_controller_node import SafetyControllerNode

    rclpy.init()
    try:
        node = SafetyControllerNode()
        assert node._action_server is not None
        node.destroy_node()
    finally:
        rclpy.shutdown()
