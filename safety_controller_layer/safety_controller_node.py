"""ROS2 action server wrapping SafetyController.

Exposes an `execute_command` ExecuteCommand action: the goal carries a relative
heading change (degrees) and a forward distance (metres). The server turns the
rover to the new heading then drives it forward, publishing /cmd_vel_raw throughout
and emitting "rotating" / "driving" feedback.

A goal can be cancelled mid-maneuver: the controller's should_abort hook polls
the active goal handle's cancel flag on every ~20 Hz control tick, stops the
rover, and reports the goal as canceled.

Subscribes /imu (sensor_msgs/Imu) and /odom (nav_msgs/Odometry), publishes
/cmd_vel_raw (geometry_msgs/Twist).

Threading: a MultiThreadedExecutor runs the action execute callback -- which
blocks inside the SafetyController control loops -- concurrently with the
IMU/odom subscription callbacks, so the controller always reads fresh sensor
data. All callbacks share a ReentrantCallbackGroup. Cached sensor values are
single floats / a tuple reference, so the cross-thread read is GIL-atomic.

This module is intentionally untested locally: rclpy is not installable on the
development machine, and the ExecuteCommand action interface must be generated
by a ROS2 build (colcon) before this file can even be imported. See
safety_controller_layer_interfaces/action/ExecuteCommand.action.
"""

import math
import sys
import time
from typing import Optional, Tuple

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from safety_controller_layer.control_math import (
    ControllerCancelledError,
    ControllerParams,
    ControllerTimeoutError,
    SafetyController,
)

# The ExecuteCommand action interface lives in the safety_controller_layer_interfaces
# ament_cmake package. That package is scaffolded (package.xml + CMakeLists.txt)
# but must be built with `colcon build` in a ROS2 workspace before this import
# resolves -- the generated interface is not importable from source alone.
from safety_controller_layer_interfaces.action import ExecuteCommand

CMD_VEL_TOPIC = "/cmd_vel_raw"
IMU_TOPIC = "/imu"
ODOM_TOPIC = "/odom"
ACTION_NAME = "execute_command"


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Extract yaw from a quaternion (ZYX Euler convention, no external deps)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class SafetyControllerNode(Node):
    """ROS2 node exposing SafetyController as an ExecuteCommand action server."""

    def __init__(self, params: Optional[ControllerParams] = None):
        super().__init__("safety_controller")
        self._params = params or ControllerParams()
        self._latest_yaw: Optional[float] = None
        self._latest_xy: Optional[Tuple[float, float]] = None
        # Set while a goal is executing; the should_abort hook polls its
        # cancel flag. None when idle.
        self._active_goal_handle = None

        # A ReentrantCallbackGroup lets the IMU/odom subscription callbacks run
        # on other MultiThreadedExecutor threads while the action callback is
        # blocked inside a SafetyController control loop.
        self._callback_group = ReentrantCallbackGroup()

        self._cmd_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(
            Imu, IMU_TOPIC, self._on_imu, 10, callback_group=self._callback_group
        )
        self.create_subscription(
            Odometry, ODOM_TOPIC, self._on_odom, 10,
            callback_group=self._callback_group,
        )

        self._controller = SafetyController(
            params=self._params,
            get_yaw=self._read_yaw,
            get_position=self._read_position,
            publish_twist=self._publish_twist,
            sleep=time.sleep,
            now=time.monotonic,
            should_abort=self._is_cancel_requested,
        )

        self._action_server = ActionServer(
            self,
            ExecuteCommand,
            ACTION_NAME,
            execute_callback=self._execute_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._callback_group,
        )

    def _on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        self._latest_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        self._latest_xy = (p.x, p.y)

    def _read_yaw(self) -> float:
        if self._latest_yaw is None:
            raise RuntimeError(f"No IMU message received on {IMU_TOPIC} yet")
        return self._latest_yaw

    def _read_position(self) -> Tuple[float, float]:
        if self._latest_xy is None:
            raise RuntimeError(f"No odometry message received on {ODOM_TOPIC} yet")
        return self._latest_xy

    def _publish_twist(self, linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)

    def _cancel_callback(self, goal_handle) -> CancelResponse:
        # Accept every cancel request; the control loop polls the flag via
        # _is_cancel_requested and stops the rover at the next tick.
        return CancelResponse.ACCEPT

    def _is_cancel_requested(self) -> bool:
        handle = self._active_goal_handle
        return handle is not None and handle.is_cancel_requested

    def _execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = ExecuteCommand.Result()
        if goal.distance_m < 0.0:
            goal_handle.abort()
            result.success = False
            result.message = (
                f"rejected: distance_m must be non-negative, got {goal.distance_m}"
            )
            self.get_logger().warn(result.message)
            return result
        self._active_goal_handle = goal_handle
        feedback = ExecuteCommand.Feedback()
        try:
            feedback.phase = "rotating"
            goal_handle.publish_feedback(feedback)
            self._controller.rotate_to_heading(math.radians(goal.heading_degree))

            feedback.phase = "driving"
            goal_handle.publish_feedback(feedback)
            self._controller.drive_distance(goal.distance_m)
        except ControllerCancelledError as exc:
            self._publish_twist(0.0, 0.0)
            goal_handle.canceled()
            result.success = False
            result.message = f"canceled: {exc}"
            self.get_logger().warn(result.message)
            return result
        except (ControllerTimeoutError, RuntimeError) as exc:
            self._publish_twist(0.0, 0.0)
            goal_handle.abort()
            result.success = False
            result.message = f"aborted: {exc}"
            self.get_logger().warn(result.message)
            return result
        finally:
            self._active_goal_handle = None

        self._publish_twist(0.0, 0.0)
        goal_handle.succeed()
        result.success = True
        result.message = (
            f"completed: turned {goal.heading_degree:.1f} deg, "
            f"drove {goal.distance_m:.2f} m"
        )
        self.get_logger().info(result.message)
        return result


def main(argv=None) -> int:
    rclpy.init(args=argv)
    node = SafetyControllerNode()
    # At least 3 threads: the blocking execute callback plus the /imu and
    # /odom subscription callbacks must all run concurrently. The default
    # (cpu_count) can be 1-2 on small embedded boards, which would starve the
    # sensor callbacks while a maneuver runs.
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    node.get_logger().info(
        f"SafetyControllerNode up; ExecuteCommand action server ready on "
        f"'{ACTION_NAME}'."
    )
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted; stopping rover.")
    finally:
        try:
            node._publish_twist(0.0, 0.0)
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
