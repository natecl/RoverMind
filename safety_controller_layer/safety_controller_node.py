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
