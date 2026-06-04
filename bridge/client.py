"""Mac-side client for bridge_server. Stdlib only — never imports rclpy or torch.

Exposes the same two callables run_agent.py already wires into build_graph,
so the agent code path is unchanged below the bridge seam.
"""

import socket
import time
from typing import Callable, Optional
from urllib.parse import urlparse

from agent.command_executor import ExecuteResult, validate_command
from bridge.errors import (
    BridgeProtocolError,
    BridgeUnreachable,
    CommandExecutorError,
    FrameCaptureError,
)
from bridge.wire import decode_frame, encode_frame, MalformedFrameError, scene_observation_from_dict


_DEFAULT_TIMEOUT_S = 75.0  # > DEFAULT_GOAL_TIMEOUT_S in command_executor.py


class BridgeClient:
    def __init__(self, url: str, timeout_s: float = _DEFAULT_TIMEOUT_S,
                 timing_sink: Optional[Callable[[str, str, float], None]] = None):
        parsed = urlparse(url)
        if parsed.scheme != "tcp" or not parsed.hostname or not parsed.port:
            raise ValueError(f"expected tcp://host:port, got {url!r}")
        self._addr = (parsed.hostname, parsed.port)
        self._timeout_s = timeout_s
        self._sock: Optional[socket.socket] = None
        self._stream = None
        self._next_id = 1
        # Optional latency sink: called (method, metric_name, milliseconds) for
        # each timing value the server reports plus the client-measured round-trip.
        self._timing_sink = timing_sink

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

    def capture_and_analyze(self, target: str):
        result = self._call("capture_and_analyze", {"target": str(target)})
        if not isinstance(result, dict):
            raise BridgeProtocolError(f"malformed capture_and_analyze result: {result!r}")
        return scene_observation_from_dict(result)

    def _call(self, method: str, args: dict):
        if self._stream is None:
            raise BridgeUnreachable("client used outside of `with` block")
        request_id = self._next_id
        self._next_id += 1
        start = time.perf_counter()
        try:
            self._stream.write(encode_frame({"id": request_id, "method": method, "args": args}))
            reply = decode_frame(self._stream.read)
        except (OSError, MalformedFrameError) as exc:
            raise BridgeUnreachable(f"bridge connection lost: {exc}") from exc
        round_trip_ms = (time.perf_counter() - start) * 1000.0
        if not isinstance(reply, dict) or reply.get("id") != request_id:
            raise BridgeProtocolError(f"unexpected reply: {reply!r}")
        if reply.get("ok") is True:
            self._forward_timing(method, round_trip_ms, reply.get("timing"))
            return reply["result"]
        error = reply.get("error", {})
        err_type = error.get("type", "")
        err_msg = error.get("message", "")
        if err_type == "ros_action_unavailable":
            raise CommandExecutorError(err_msg)
        if err_type == "vision_error":
            raise FrameCaptureError(err_msg)
        raise BridgeProtocolError(f"bridge error {err_type!r}: {err_msg}")

    def _forward_timing(self, method: str, round_trip_ms: float, timing) -> None:
        """Push the round-trip + the server's `timing` block to the sink.

        No-op without a sink or a `timing` key (an old server), so the agent
        path is unchanged when the rover hasn't been updated. Nested dicts
        (e.g. the per-Moondream-call `vlm` breakdown) flatten **one level** to
        dotted names — the server only ever sends depth-1, and a deeper dict
        would be dropped by `_emit` (it isn't numeric), not crash.

        `transport_overhead_ms` is emitted only when `server_ms` is present and
        numeric; the real server always sets it, so this is just defensive.
        """
        if self._timing_sink is None or not isinstance(timing, dict):
            return
        self._emit(method, "round_trip_ms", round_trip_ms)
        for name, value in timing.items():
            if isinstance(value, dict):
                for sub, subv in value.items():
                    self._emit(method, f"{name}.{sub}", subv)
            else:
                self._emit(method, name, value)
        server_ms = timing.get("server_ms")
        if isinstance(server_ms, (int, float)):
            self._emit(method, "transport_overhead_ms",
                       max(0.0, round_trip_ms - float(server_ms)))

    def _emit(self, method: str, name: str, value) -> None:
        """Forward one timing value, ignoring anything non-numeric.

        The reply comes off the wire from the rover; a malformed or
        version-mismatched server must not crash an otherwise-successful call.
        """
        try:
            self._timing_sink(method, name, float(value))
        except (TypeError, ValueError):
            pass
