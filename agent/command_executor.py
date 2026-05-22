"""Sync Python wrapper around the ExecuteCommand ROS2 action.

Pure-logic helpers (validate_command, ExecuteResult) are laptop-testable.
The real ActionClient call lives in CommandExecutor below and imports rclpy
at module level — that class is verified on the rover, not unit-tested
locally. Tests that exercise CommandExecutor must `pytest.importorskip("rclpy")`.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class ExecuteResult:
    """Outcome of one execute_command call."""

    success: bool
    message: str


def validate_command(heading_deg: float, distance_m: float) -> None:
    """Raise ValueError if the command would be unsafe or malformed.

    The ROS2 action server itself rejects negative distances, but validating
    here gives the agent a fast, local failure rather than waiting for the
    action result. Non-finite inputs are rejected outright — they almost
    always indicate a bug in the upstream resolver.
    """
    if not math.isfinite(heading_deg):
        raise ValueError(f"heading_deg must be finite, got {heading_deg!r}")
    if not math.isfinite(distance_m):
        raise ValueError(f"distance_m must be finite, got {distance_m!r}")
    if distance_m < 0.0:
        raise ValueError(
            f"distance_m must be non-negative, got {distance_m!r}"
        )
