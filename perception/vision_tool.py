"""capture_and_analyze: grab a frame, ask Moondream2, return a SceneObservation.

The capture function and the Moondream2 client are injected so the whole
orchestration is testable with fakes. The real rclpy capture lives here too
(ros_capture_fn) but is hardware-only.
"""

from perception.scene_parsing import build_observation, parse_yes_no

_VISIBLE_Q = "Is there a {target} in this image? Answer yes or no."
_DIRECTION_Q = (
    "Is the {target} on the left, in the center, or on the right of the image?"
)
_DISTANCE_Q = "Is the {target} close, at a medium distance, or far away?"


class FrameCaptureError(RuntimeError):
    """Raised when no camera frame can be obtained within the timeout."""


def capture_and_analyze(target, *, capture_fn, moondream):
    """Capture one frame, ask Moondream2 about `target`, return a SceneObservation.

    `capture_fn()` returns an image (or raises FrameCaptureError). `moondream`
    is an object with `ask(image, question) -> str`. When the target is not
    visible, the direction/distance questions are skipped.
    """
    image = capture_fn()
    visible_answer = moondream.ask(image, _VISIBLE_Q.format(target=target))
    if not parse_yes_no(visible_answer):
        return build_observation(target, visible_answer, "", "")
    direction_answer = moondream.ask(image, _DIRECTION_Q.format(target=target))
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
