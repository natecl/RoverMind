"""QoS smoke test for the emergency-brake node's /scan subscription.

The YDLidar publishes /scan with BEST_EFFORT reliability (sensor-data QoS). A
RELIABLE subscriber is *incompatible* with a BEST_EFFORT publisher, so the AEB
would silently receive no scans and brake forever on the stale-scan fail-safe.
This guards that the /scan subscription requests BEST_EFFORT.

Runnable only inside a built + sourced ROS2 workspace (i.e. on the rover); rclpy
is not installable on the dev laptop, so the whole module skips there -- same
pattern as test_ros_package_loads.py.
"""

import pytest

# ROS2 is not installable on the dev laptop; skip cleanly when absent.
rclpy = pytest.importorskip("rclpy")

from rclpy.qos import ReliabilityPolicy  # noqa: E402

from safety_controller_layer.aeb_node import EmergencyBrakeNode, SCAN_TOPIC  # noqa: E402


@pytest.fixture
def rclpy_context():
    rclpy.init()
    try:
        yield
    finally:
        rclpy.shutdown()


def _find_subscription(node, topic):
    for sub in node.subscriptions:
        if sub.topic_name == topic:
            return sub
    return None


def test_scan_subscription_is_best_effort(rclpy_context):
    """AEB must subscribe to /scan with BEST_EFFORT to match the LiDAR publisher."""
    node = EmergencyBrakeNode()
    try:
        scan_sub = _find_subscription(node, SCAN_TOPIC)
        assert scan_sub is not None, f"no subscription found on {SCAN_TOPIC}"
        assert scan_sub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT, (
            "AEB /scan subscription is not BEST_EFFORT; it will not receive the "
            "LiDAR's BEST_EFFORT scans and will brake forever."
        )
    finally:
        node.destroy_node()
