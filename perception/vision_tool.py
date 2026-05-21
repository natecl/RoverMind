"""capture_and_analyze: grab a frame, ask Moondream2, return a SceneObservation.

The capture function and the Moondream2 client are injected so the whole
orchestration is testable with fakes. The real rclpy capture lives here too
(ros_capture_fn) but is hardware-only.
"""

from perception.depth_math import depth_to_distance_bucket, sample_depth_patch
from perception.scene_parsing import build_observation, parse_yes_no

_VISIBLE_Q = "Is there a {target} in this image? Answer yes or no."
_DIRECTION_Q = (
    "Is the {target} on the left, in the center, or on the right of the image?"
)
_DISTANCE_Q = "Is the {target} close, at a medium distance, or far away?"


class FrameCaptureError(RuntimeError):
    """Raised when no camera frame can be obtained within the timeout."""


def capture_and_analyze(target, *, capture_fn, moondream, depth_fn=None):
    """Capture one frame, ask Moondream2 about `target`, return a SceneObservation.

    `capture_fn()` returns an image (or raises FrameCaptureError). `moondream`
    has `ask(image, question) -> str` and `point(image, target) -> (x, y)|None`.

    When `depth_fn` is given, distance is read from the depth camera: Moondream2
    points at the target, the depth patch at that point is sampled, and a metric
    bucket is computed. If the target cannot be pointed at, or the depth patch
    is all-invalid, the tool falls back to asking Moondream2 for the distance.
    """
    image = capture_fn()
    visible_answer = moondream.ask(image, _VISIBLE_Q.format(target=target))
    if not parse_yes_no(visible_answer):
        return build_observation(target, visible_answer, "", "")
    direction_answer = moondream.ask(image, _DIRECTION_Q.format(target=target))

    if depth_fn is not None:
        point = moondream.point(image, target)
        if point is not None:
            x_norm, y_norm = point
            samples = sample_depth_patch(depth_fn(), x_norm, y_norm)
            bucket, distance_m = depth_to_distance_bucket(samples)
            if bucket is not None:
                return build_observation(
                    target, visible_answer, direction_answer, "",
                    distance_override=bucket, distance_m=distance_m,
                    distance_source="depth",
                )

    distance_answer = moondream.ask(image, _DISTANCE_Q.format(target=target))
    return build_observation(
        target, visible_answer, direction_answer, distance_answer,
    )


def ros_capture_fn(topic: str = "/camera/color/image_raw",
                   timeout_s: float = 5.0):
    """Grab one frame off the camera topic as a PIL RGB image.

    Hardware-only: needs a sourced ROS2 environment. Raises FrameCaptureError
    if no frame arrives within `timeout_s`.
    """
    import time

    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from PIL import Image
    from rclpy.node import Node
    from sensor_msgs.msg import Image as RosImage

    rclpy.init()
    node = Node("vision_tool_capture")
    bridge = CvBridge()
    received = {}

    def _on_image(msg):
        received["msg"] = msg

    node.create_subscription(RosImage, topic, _on_image, 10)
    try:
        deadline = time.monotonic() + timeout_s
        while "msg" not in received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if "msg" not in received:
            raise FrameCaptureError(
                f"no frame on {topic} within {timeout_s:.1f}s"
            )
        bgr = bridge.imgmsg_to_cv2(received["msg"], desired_encoding="bgr8")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return Image.fromarray(rgb)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def ros_depth_capture_fn(topic: str = "/camera/depth/image_raw",
                         timeout_s: float = 5.0):
    """Grab one depth frame as a height x width grid of millimetres.

    Hardware-only: needs a sourced ROS2 environment. Raises FrameCaptureError
    if no frame arrives within `timeout_s`. The depth topic must be registered
    (aligned) to the colour image so a colour-image point indexes the same
    pixel in the depth image.
    """
    import time

    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image as RosImage

    rclpy.init()
    node = Node("vision_tool_depth_capture")
    bridge = CvBridge()
    received = {}

    def _on_depth(msg):
        received["msg"] = msg

    node.create_subscription(RosImage, topic, _on_depth, 10)
    try:
        deadline = time.monotonic() + timeout_s
        while "msg" not in received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if "msg" not in received:
            raise FrameCaptureError(
                f"no depth frame on {topic} within {timeout_s:.1f}s"
            )
        # passthrough keeps the raw 16-bit millimetre values.
        depth = bridge.imgmsg_to_cv2(received["msg"], desired_encoding="passthrough")
        return depth
    finally:
        node.destroy_node()
        rclpy.shutdown()
