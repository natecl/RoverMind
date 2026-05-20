"""launch_testing integration test for the missing-sensor abort path.

The safety_controller_node executable is launched with NO /imu or /odom
publisher anywhere on the graph. A valid ExecuteCommand goal must therefore be
aborted (the controller's get_yaw call raises immediately) rather than hang --
verifying the node's sensor-wiring error handling end-to-end.

This needs its own launch description: the node caches sensor values once
received, so "no sensor data" can only be tested against a freshly launched
node process.

Runnable only inside a built + sourced ROS2 workspace; skipped off the robot
where rclpy is not installed.
"""

import threading
import time
import unittest

import pytest

# ROS2 is not installable on the dev laptop; skip cleanly when absent.
rclpy = pytest.importorskip("rclpy")

import launch
import launch_ros.actions
import launch_testing.actions
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from safety_controller_layer_interfaces.action import ExecuteCommand


@pytest.mark.launch_test
def generate_test_description():
    """Launch only the node under test -- no sensor publishers at all."""
    node_under_test = launch_ros.actions.Node(
        package="safety_controller_layer",
        executable="safety_controller_node",
        name="safety_controller",
        output="screen",
    )
    return (
        launch.LaunchDescription([
            node_under_test,
            launch_testing.actions.ReadyToTest(),
        ]),
        {},
    )


class TestMissingSensorsAbort(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.client_node = rclpy.create_node("missing_sensors_test_client")
        cls.action_client = ActionClient(
            cls.client_node, ExecuteCommand, "execute_command")
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.client_node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.spin_thread.join(timeout=5.0)
        cls.action_client.destroy()
        cls.client_node.destroy_node()
        rclpy.shutdown()

    def _await(self, future, timeout, what):
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        self.assertTrue(future.done(), f"{what} did not complete within {timeout}s")
        return future.result()

    def test_goal_is_aborted_when_no_sensor_data(self):
        """With no /imu ever published, a valid goal aborts instead of hanging."""
        self.assertTrue(
            self.action_client.wait_for_server(timeout_sec=10.0),
            "execute_command action server never became available")
        goal = ExecuteCommand.Goal()
        goal.heading_degree = 15.0
        goal.distance_m = 0.2
        goal_handle = self._await(
            self.action_client.send_goal_async(goal), 10.0, "goal acceptance")
        self.assertTrue(goal_handle.accepted)
        response = self._await(
            goal_handle.get_result_async(), 15.0, "goal result")
        self.assertEqual(response.status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(response.result.success)
        self.assertIn("IMU", response.result.message)
