"""Phase 2a: the bridge server attaches a `timing` block to its reply envelope.

These tests decode the raw reply envelope (not via BridgeClient) so they assert
exactly what the server puts on the wire: `timing.server_ms` for every dispatched
method, plus `action_ms` / `vlm_ms` for the two heavy handlers, on both the ok
and the error path.
"""

import socket
import threading
import time

import pytest
from PIL import Image

from agent.command_executor import CommandExecutorError, ExecuteResult
from bridge.bridge_server import BridgeServer
from bridge.wire import decode_frame, encode_frame


class _FakeExecutor:
    def __init__(self, *, raises=None, result=None):
        self.calls = []
        self._raises = raises
        self._result = result or ExecuteResult(success=True, message="ok")

    def execute(self, heading_deg, distance_m):
        self.calls.append((heading_deg, distance_m))
        if self._raises is not None:
            raise self._raises
        return self._result


class _FakeMoondream:
    def __init__(self, *, ask_replies, point_reply=None):
        self._replies = list(ask_replies)
        self._point = point_reply

    def ask(self, image, question):
        return self._replies.pop(0)

    def point(self, image, target):
        return self._point


def _start(**kwargs):
    server = BridgeServer(host="127.0.0.1", port=0, **kwargs)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    deadline = time.monotonic() + 2.0
    while server.bound_port is None and time.monotonic() < deadline:
        time.sleep(0.01)
    assert server.bound_port is not None
    return server, t


def _raw_call(port, request):
    s = socket.create_connection(("127.0.0.1", port), timeout=2.0)
    try:
        with s.makefile("rwb", buffering=0) as stream:
            stream.write(encode_frame(request))
            return decode_frame(stream.read)
    finally:
        s.close()


def test_execute_command_attaches_server_and_action_ms():
    exe = _FakeExecutor(result=ExecuteResult(True, "ok"))
    server, t = _start(command_executor=exe)
    try:
        reply = _raw_call(server.bound_port, {
            "id": 1, "method": "execute_command",
            "args": {"heading_degree": 30.0, "distance_m": 0.5},
        })
    finally:
        server.shutdown(); t.join(timeout=2.0)
    assert reply["ok"] is True
    timing = reply["timing"]
    assert timing["server_ms"] >= 0.0
    assert timing["action_ms"] >= 0.0
    assert timing["action_ms"] <= timing["server_ms"] + 1e-6


def test_capture_and_analyze_attaches_vlm_ms():
    md = _FakeMoondream(ask_replies=["yes", "left", "close"])
    server, t = _start(capture_fn=lambda: Image.new("RGB", (32, 32)),
                       moondream_factory=lambda: md)
    try:
        reply = _raw_call(server.bound_port, {
            "id": 1, "method": "capture_and_analyze",
            "args": {"target": "water bottle"},
        })
    finally:
        server.shutdown(); t.join(timeout=2.0)
    assert reply["ok"] is True
    timing = reply["timing"]
    assert timing["vlm_ms"] >= 0.0
    assert timing["vlm_ms"] <= timing["server_ms"] + 1e-6


def test_capture_and_analyze_attaches_per_call_vlm_breakdown():
    md = _FakeMoondream(ask_replies=["yes", "left", "close"], point_reply=None)
    server, t = _start(capture_fn=lambda: Image.new("RGB", (32, 32)),
                       moondream_factory=lambda: md)
    try:
        reply = _raw_call(server.bound_port, {
            "id": 1, "method": "capture_and_analyze",
            "args": {"target": "water bottle"},
        })
    finally:
        server.shutdown(); t.join(timeout=2.0)
    assert reply["ok"] is True
    vlm = reply["timing"]["vlm"]
    assert "ask_visible" in vlm
    assert "ask_direction" in vlm
    assert "ask_distance" in vlm
    assert all(v >= 0.0 for v in vlm.values())


def test_ping_has_server_ms_but_no_action_or_vlm():
    server, t = _start()
    try:
        reply = _raw_call(server.bound_port, {"id": 1, "method": "ping", "args": {}})
    finally:
        server.shutdown(); t.join(timeout=2.0)
    assert reply["ok"] is True
    assert "server_ms" in reply["timing"]
    assert "action_ms" not in reply["timing"]
    assert "vlm_ms" not in reply["timing"]


def test_error_reply_still_carries_timing():
    exe = _FakeExecutor(raises=CommandExecutorError("action server gone"))
    server, t = _start(command_executor=exe)
    try:
        reply = _raw_call(server.bound_port, {
            "id": 1, "method": "execute_command",
            "args": {"heading_degree": 0.0, "distance_m": 0.5},
        })
    finally:
        server.shutdown(); t.join(timeout=2.0)
    assert reply["ok"] is False
    assert reply["error"]["type"] == "ros_action_unavailable"
    # Failures are measured too: server_ms always, action_ms because the
    # executor was invoked (and timed) before it raised.
    assert "server_ms" in reply["timing"]
    assert "action_ms" in reply["timing"]
