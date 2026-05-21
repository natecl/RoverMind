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
