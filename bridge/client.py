"""Mac-side client for bridge_server. Stdlib only — never imports rclpy or torch.

Exposes the same two callables run_agent.py already wires into build_graph,
so the agent code path is unchanged below the bridge seam.
"""

import socket
from typing import Optional
from urllib.parse import urlparse

from agent.command_executor import ExecuteResult, validate_command
from bridge.errors import (
    BridgeProtocolError,
    BridgeUnreachable,
    CommandExecutorError,
    FrameCaptureError,
)
from bridge.wire import decode_frame, encode_frame, MalformedFrameError


_DEFAULT_TIMEOUT_S = 75.0  # > DEFAULT_GOAL_TIMEOUT_S in command_executor.py


class BridgeClient:
    def __init__(self, url: str, timeout_s: float = _DEFAULT_TIMEOUT_S):
        parsed = urlparse(url)
        if parsed.scheme != "tcp" or not parsed.hostname or not parsed.port:
            raise ValueError(f"expected tcp://host:port, got {url!r}")
        self._addr = (parsed.hostname, parsed.port)
        self._timeout_s = timeout_s
        self._sock: Optional[socket.socket] = None
        self._stream = None
        self._next_id = 1

    def __enter__(self) -> "BridgeClient":
        try:
            self._sock = socket.create_connection(self._addr, timeout=5.0)
        except (ConnectionRefusedError, socket.timeout, OSError) as exc:
            raise BridgeUnreachable(
                f"could not connect to bridge at tcp://{self._addr[0]}:{self._addr[1]}: {exc}"
            ) from exc
        self._sock.settimeout(self._timeout_s)
        self._stream = self._sock.makefile("rwb", buffering=0)
        return self

    def __exit__(self, *exc_info) -> None:
        if self._stream is not None:
            try:
                self._stream.close()
            except OSError:
                pass
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass

    def ping(self) -> str:
        result = self._call("ping", {})
        if not isinstance(result, str):
            raise BridgeProtocolError(f"ping returned non-string: {result!r}")
        return result

    def execute_command(self, heading_deg: float, distance_m: float) -> ExecuteResult:
        validate_command(heading_deg, distance_m)
        result = self._call("execute_command",
                            {"heading_degree": float(heading_deg),
                             "distance_m": float(distance_m)})
        if not isinstance(result, dict) or "success" not in result or "message" not in result:
            raise BridgeProtocolError(f"malformed execute_command result: {result!r}")
        return ExecuteResult(success=bool(result["success"]), message=str(result["message"]))

    def _call(self, method: str, args: dict):
        if self._stream is None:
            raise BridgeUnreachable("client used outside of `with` block")
        request_id = self._next_id
        self._next_id += 1
        try:
            self._stream.write(encode_frame({"id": request_id, "method": method, "args": args}))
            reply = decode_frame(self._stream.read)
        except (OSError, MalformedFrameError) as exc:
            raise BridgeUnreachable(f"bridge connection lost: {exc}") from exc
        if not isinstance(reply, dict) or reply.get("id") != request_id:
            raise BridgeProtocolError(f"unexpected reply: {reply!r}")
        if reply.get("ok") is True:
            return reply["result"]
        error = reply.get("error", {})
        err_type = error.get("type", "")
        err_msg = error.get("message", "")
        if err_type == "ros_action_unavailable":
            raise CommandExecutorError(err_msg)
        if err_type == "vision_error":
            raise FrameCaptureError(err_msg)
        raise BridgeProtocolError(f"bridge error {err_type!r}: {err_msg}")
