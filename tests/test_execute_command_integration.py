"""launch_testing integration tests for the ExecuteCommand action server.

These exercise the full ROS2 stack: the safety_controller_node executable runs
as a real launched process, a FakeRover node closes the control loop by
integrating /cmd_vel into /imu and /odom, and an action client drives a goal
end-to-end and asserts on the result and feedback.

Runnable only inside a built + sourced ROS2 workspace:
    colcon test --packages-select safety_controller_layer
    # or directly:
    launch_test tests/test_execute_command_integration.py

Off the robot, where rclpy is not installed, the whole module is skipped.
"""

import math
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
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Imu

from safety_controller_layer_interfaces.action import ExecuteCommand


@pytest.mark.launch_test
def generate_test_description():
    """Launch the node under test, then signal launch_testing to begin."""
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


class FakeRover(Node):
    """Closes the control loop: integrates /cmd_vel into yaw + position and
    republishes them as /imu and /odom at 50 Hz, giving the node's controller
    live sensor feedback to converge against."""

    def __init__(self):
        super().__init__("fake_rover")
        self._yaw = 0.0
        self._x = 0.0
        self._y = 0.0
        self._linear = 0.0
        self._angular = 0.0
        self._last_tick = time.monotonic()
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.imu_pub = self.create_publisher(Imu, "/imu", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.create_timer(0.02, self._tick)

    def _on_cmd_vel(self, msg):
        self._linear = msg.linear.x
        self._angular = msg.angular.z

    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        self._yaw += self._angular * dt
        self._x += self._linear * math.cos(self._yaw) * dt
        self._y += self._linear * math.sin(self._yaw) * dt

        imu = Imu()
        # Pure-yaw quaternion; the node's quaternion_to_yaw recovers _yaw.
        imu.orientation.z = math.sin(self._yaw / 2.0)
        imu.orientation.w = math.cos(self._yaw / 2.0)
        self.imu_pub.publish(imu)

        odom = Odometry()
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        self.odom_pub.publish(odom)


class TestExecuteCommandIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.fake_rover = FakeRover()
        cls.client_node = rclpy.create_node("execute_command_test_client")
        cls.action_client = ActionClient(
            cls.client_node, ExecuteCommand, "execute_command")
        cls.executor = SingleThreadedExecutor()
        cls.executor.add_node(cls.fake_rover)
        cls.executor.add_node(cls.client_node)
        cls.spin_thread = threading.Thread(target=cls.executor.spin, daemon=True)
        cls.spin_thread.start()

    @classmethod
    def tearDownClass(cls):
        cls.executor.shutdown()
        cls.spin_thread.join(timeout=5.0)
        cls.action_client.destroy()
        cls.fake_rover.destroy_node()
        cls.client_node.destroy_node()
        rclpy.shutdown()

    # --- helpers ---------------------------------------------------------

    def _await(self, future, timeout, what):
        """Block until a future resolves (the background executor drives it)."""
        deadline = time.time() + timeout
        while not future.done() and time.time() < deadline:
            time.sleep(0.05)
        self.assertTrue(future.done(), f"{what} did not complete within {timeout}s")
        return future.result()

    def _wait_until(self, predicate, timeout, message):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if predicate():
                return
            time.sleep(0.1)
        self.fail(message)

    def _run_command(self, heading_degree, distance_m, timeout):
        self.assertTrue(
            self.action_client.wait_for_server(timeout_sec=10.0),
            "execute_command action server never became available")
        goal = ExecuteCommand.Goal()
        goal.heading_degree = float(heading_degree)
        goal.distance_m = float(distance_m)
        phases = []
        send_future = self.action_client.send_goal_async(
            goal, feedback_callback=lambda fb: phases.append(fb.feedback.phase))
        goal_handle = self._await(send_future, 10.0, "goal acceptance")
        self.assertTrue(goal_handle.accepted, "server did not accept the goal")
        response = self._await(
            goal_handle.get_result_async(), timeout, "goal result")
        return response.status, response.result, phases

    # --- tests -----------------------------------------------------------

    def test_action_server_is_available(self):
        """The launched node brings up the execute_command action server."""
        self.assertTrue(
            self.action_client.wait_for_server(timeout_sec=10.0),
            "execute_command action server never became available")

    def test_full_maneuver_succeeds(self):
        """A valid goal rotates then drives the (fake) rover to completion,
        reporting 'rotating' then 'driving' feedback and a SUCCEEDED result."""
        # The controller reads cached /imu + /odom on its first tick, so make
        # sure the node has subscribed to FakeRover's topics first.
        self._wait_until(
            lambda: (self.fake_rover.imu_pub.get_subscription_count() > 0
                     and self.fake_rover.odom_pub.get_subscription_count() > 0),
            timeout=15.0,
            message="node never subscribed to /imu and /odom")
        time.sleep(0.5)  # let sensor messages populate the node's cache

        status, result, phases = self._run_command(10.0, 0.5, timeout=25.0)

        self.assertEqual(
            status, GoalStatus.STATUS_SUCCEEDED,
            f"maneuver did not succeed: {result.message}")
        self.assertTrue(result.success)
        self.assertIn("rotating", phases)
        self.assertIn("driving", phases)
        self.assertLess(
            phases.index("rotating"), phases.index("driving"),
            "feedback reported 'driving' before 'rotating'")

    def test_negative_distance_goal_is_aborted(self):
        """A negative distance is rejected end-to-end with an ABORTED result."""
        status, result, _ = self._run_command(0.0, -0.5, timeout=10.0)
        self.assertEqual(status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(result.success)
        self.assertIn("non-negative", result.message)
