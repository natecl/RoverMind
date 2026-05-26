import socket
import threading
import time

import numpy as np
import pytest
from PIL import Image

from bridge.bridge_server import BridgeServer
from bridge.client import BridgeClient
from perception.vision_tool import FrameCaptureError


@pytest.fixture
def running_server():
    """Start a BridgeServer with no rclpy/Moondream wired (ping-only)."""
    server = BridgeServer(host="127.0.0.1", port=0,
                         command_executor=None,
                         capture_fn=None, depth_fn=None, moondream_factory=None)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    # Wait until the listener is bound.
    deadline = time.monotonic() + 2.0
    while server.bound_port is None and time.monotonic() < deadline:
        time.sleep(0.01)
    assert server.bound_port is not None, "server failed to bind"
    yield server
    server.shutdown()
    t.join(timeout=2.0)


def test_ping_through_real_server(running_server):
    url = f"tcp://127.0.0.1:{running_server.bound_port}"
    with BridgeClient(url) as client:
        assert client.ping() == "pong"


def test_two_sequential_pings_use_distinct_ids(running_server):
    url = f"tcp://127.0.0.1:{running_server.bound_port}"
    with BridgeClient(url) as client:
        assert client.ping() == "pong"
        assert client.ping() == "pong"


from agent.command_executor import CommandExecutorError, ExecuteResult


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


def _start(command_executor):
    server = BridgeServer(host="127.0.0.1", port=0,
                         command_executor=command_executor,
                         capture_fn=None, depth_fn=None, moondream_factory=None)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    deadline = time.monotonic() + 2.0
    while server.bound_port is None and time.monotonic() < deadline:
        time.sleep(0.01)
    return server, t


def test_execute_command_dispatches_to_executor():
    exe = _FakeExecutor(result=ExecuteResult(success=True, message="completed"))
    server, t = _start(exe)
    try:
        with BridgeClient(f"tcp://127.0.0.1:{server.bound_port}") as client:
            result = client.execute_command(heading_deg=30.0, distance_m=0.5)
        assert exe.calls == [(30.0, 0.5)]
        assert result == ExecuteResult(success=True, message="completed")
    finally:
        server.shutdown(); t.join(timeout=2.0)


def test_execute_command_action_unavailable_returns_structured_error():
    exe = _FakeExecutor(raises=CommandExecutorError("action server gone"))
    server, t = _start(exe)
    try:
        with BridgeClient(f"tcp://127.0.0.1:{server.bound_port}") as client:
            with pytest.raises(CommandExecutorError, match="action server gone"):
                client.execute_command(heading_deg=0.0, distance_m=0.5)
    finally:
        server.shutdown(); t.join(timeout=2.0)


class _FakeMoondream:
    """Minimal fake matching MoondreamClient's `ask` + `point` surface."""
    def __init__(self, *, ask_replies, point_reply=None):
        self._replies = list(ask_replies)
        self._point = point_reply
    def ask(self, image, question):
        return self._replies.pop(0)
    def point(self, image, target):
        return self._point


def _start_vision(*, capture_fn, depth_fn=None, moondream=None):
    server = BridgeServer(host="127.0.0.1", port=0,
                         command_executor=None,
                         capture_fn=capture_fn,
                         depth_fn=depth_fn,
                         moondream_factory=(lambda: moondream) if moondream else None)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    deadline = time.monotonic() + 2.0
    while server.bound_port is None and time.monotonic() < deadline:
        time.sleep(0.01)
    return server, t


def test_capture_and_analyze_wires_capture_fn_and_moondream():
    fake_image = Image.new("RGB", (32, 32))
    md = _FakeMoondream(ask_replies=["yes", "left", "close"], point_reply=None)
    server, t = _start_vision(capture_fn=lambda: fake_image, moondream=md)
    try:
        with BridgeClient(f"tcp://127.0.0.1:{server.bound_port}") as client:
            obs = client.capture_and_analyze("water bottle")
        assert obs.target == "water bottle"
        assert obs.found is True
        assert obs.direction == "left"
        assert obs.distance == "close"
    finally:
        server.shutdown(); t.join(timeout=2.0)


def test_capture_and_analyze_maps_frame_capture_error():
    def boom():
        raise FrameCaptureError("no frame on /camera/color/image_raw within 5.0s")
    server, t = _start_vision(capture_fn=boom,
                              moondream=_FakeMoondream(ask_replies=[]))
    try:
        with BridgeClient(f"tcp://127.0.0.1:{server.bound_port}") as client:
            with pytest.raises(FrameCaptureError, match="no frame"):
                client.capture_and_analyze("water bottle")
    finally:
        server.shutdown(); t.join(timeout=2.0)
