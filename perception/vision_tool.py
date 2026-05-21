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
    is an object with `ask(image, question) -> str`.
    """
    image = capture_fn()
    visible_answer = moondream.ask(image, _VISIBLE_Q.format(target=target))
    direction_answer = moondream.ask(image, _DIRECTION_Q.format(target=target))
    distance_answer = moondream.ask(image, _DISTANCE_Q.format(target=target))
    return build_observation(
        target, visible_answer, direction_answer, distance_answer,
    )
