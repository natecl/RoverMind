#!/usr/bin/env python3
"""Manually publish Twist commands to /cmd_vel_raw for rover bring-up testing.

Prompts for linear (m/s), angular (rad/s), and duration (s), publishes the
Twist at 10 Hz for the requested duration, then sends a zero-velocity stop.
Loops until Ctrl-C.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


PUBLISH_RATE_HZ = 10.0
CMD_VEL_TOPIC = "/cmd_vel_raw"


class TestDrivePublisher(Node):
    def __init__(self):
        super().__init__("test_drive")
        self.pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)

    def drive(self, linear: float, angular: float, duration: float) -> None:
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular

        period = 1.0 / PUBLISH_RATE_HZ
        end_time = time.monotonic() + duration
        self.get_logger().info(
            f"Driving linear={linear:.2f} m/s, angular={angular:.2f} rad/s "
            f"for {duration:.2f} s"
        )
        while time.monotonic() < end_time:
            self.pub.publish(msg)
            time.sleep(period)

        self.stop()

    def stop(self) -> None:
        self.pub.publish(Twist())
        self.get_logger().info("Stopped (zero Twist published).")


def prompt_float(label: str) -> float:
    while True:
        raw = input(f"{label}: ").strip()
        try:
            return float(raw)
        except ValueError:
            print(f"  '{raw}' is not a number, try again.")


def main():
    rclpy.init()
    node = TestDrivePublisher()
    print(f"Publishing to {CMD_VEL_TOPIC}. Ctrl-C to exit.")
    try:
        while True:
            print("\n--- New command ---")
            linear = prompt_float("linear (m/s)")
            angular = prompt_float("angular (rad/s)")
            duration = prompt_float("duration (s)")
            if duration <= 0:
                print("  duration must be > 0, skipping.")
                continue
            node.drive(linear, angular, duration)
    except (KeyboardInterrupt, EOFError):
        print("\nInterrupted, stopping rover.")
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
