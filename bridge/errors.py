"""Exceptions surfaced by BridgeClient.

CommandExecutorError and FrameCaptureError are re-exported from their existing
homes so call sites that catch them today (run_agent.py, future tooling) work
unchanged whether the executor is in-process or behind the bridge.
"""

from agent.command_executor import CommandExecutorError  # noqa: F401
from perception.vision_tool import FrameCaptureError      # noqa: F401


class BridgeUnreachable(RuntimeError):
    """Raised when the bridge socket cannot be opened or drops mid-call."""


class BridgeProtocolError(RuntimeError):
    """Raised when the bridge returns a malformed or id-mismatched reply."""
