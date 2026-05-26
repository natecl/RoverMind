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


# --- Hardware-only ROS2 action client ------------------------------------
#
# Everything below imports rclpy and the generated ExecuteCommand interface
# and is therefore untested on the dev laptop. Mirrors the pattern in
# safety_controller_layer/safety_controller_node.py.

import time
from typing import Optional

try:
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from safety_controller_layer_interfaces.action import ExecuteCommand
    _RCLPY_AVAILABLE = True
except ImportError:  # rclpy unavailable on dev laptop / off-rover
    rclpy = None  # type: ignore[assignment]
    ActionClient = None  # type: ignore[assignment]
    Node = None  # type: ignore[assignment]
    ExecuteCommand = None  # type: ignore[assignment]
    _RCLPY_AVAILABLE = False


ACTION_NAME = "execute_command"
DEFAULT_SERVER_TIMEOUT_S = 5.0
# Worst-case action runtime: max_timeout_s for rotate + max_timeout_s for drive
# (each capped at 30s in ControllerParams) plus margin. Sized generously so we
# only ever trip on a node death/hang, not on legitimately slow maneuvers.
DEFAULT_GOAL_TIMEOUT_S = 70.0


class CommandExecutorError(RuntimeError):
    """Raised when the action server is unreachable or aborts a goal."""


class CommandExecutor:
    """Owns an rclpy Node + ActionClient; exposes a sync execute() call.

    Constructed once per process. `execute(heading_deg, distance_m)` blocks
    until the action server returns a result and yields an `ExecuteResult`.
    Cancellation, retries, and async fan-out are intentionally not exposed —
    the LangGraph agent runs one tool call at a time.

    A goal-side timeout (`goal_timeout_s`) bounds the result wait: if the
    action server process dies after accepting a goal, the underlying future
    never resolves; without a timeout the agent would hang forever. The
    timeout is sized for the worst legitimate maneuver, so tripping it always
    indicates an external failure.
    """

    def __init__(self, node: Optional[Node] = None,
                 server_timeout_s: float = DEFAULT_SERVER_TIMEOUT_S,
                 goal_timeout_s: float = DEFAULT_GOAL_TIMEOUT_S):
        if not _RCLPY_AVAILABLE:
            raise RuntimeError(
                "rclpy is not available; CommandExecutor only works inside a sourced ROS2 environment"
            )
        if not rclpy.ok():
            rclpy.init()
        self._owns_node = node is None
        self._node = node or Node("rovermind_command_executor")
        self._client = ActionClient(self._node, ExecuteCommand, ACTION_NAME)
        self._server_timeout_s = server_timeout_s
        self._goal_timeout_s = goal_timeout_s

    def execute(self, heading_deg: float, distance_m: float) -> ExecuteResult:
        validate_command(heading_deg, distance_m)
        if not self._client.wait_for_server(timeout_sec=self._server_timeout_s):
            raise CommandExecutorError(
                f"action server '{ACTION_NAME}' not available within "
                f"{self._server_timeout_s:.1f}s"
            )
        goal = ExecuteCommand.Goal()
        goal.heading_degree = float(heading_deg)
        goal.distance_m = float(distance_m)

        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            self._node, send_future, timeout_sec=self._server_timeout_s)
        if not send_future.done():
            return ExecuteResult(
                success=False,
                message=(
                    f"goal acceptance timed out after "
                    f"{self._server_timeout_s:.1f}s (server unresponsive?)"
                ),
            )
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            return ExecuteResult(success=False, message="goal rejected")

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self._node, result_future, timeout_sec=self._goal_timeout_s)
        if not result_future.done():
            return ExecuteResult(
                success=False,
                message=(
                    f"goal result timed out after "
                    f"{self._goal_timeout_s:.1f}s (action server died?)"
                ),
            )
        wrapped = result_future.result()
        if wrapped is None:
            return ExecuteResult(success=False, message="no result returned")
        action_result = wrapped.result
        return ExecuteResult(
            success=bool(action_result.success),
            message=str(action_result.message),
        )

    def close(self) -> None:
        if self._owns_node:
            try:
                self._node.destroy_node()
            except Exception:
                pass
            try:
                rclpy.shutdown()
            except Exception:
                pass


_default_executor: Optional[CommandExecutor] = None


def execute_command(heading_deg: float, distance_m: float) -> ExecuteResult:
    """Process-singleton convenience for short scripts.

    The first call constructs a CommandExecutor (which initializes rclpy and
    creates a Node). Subsequent calls reuse it. For long-running processes,
    construct a CommandExecutor explicitly so its lifecycle is visible.
    """
    global _default_executor
    if _default_executor is None:
        _default_executor = CommandExecutor()
    return _default_executor.execute(heading_deg, distance_m)
