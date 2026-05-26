import socket
import threading
import time

import pytest

from bridge.bridge_server import BridgeServer
from bridge.client import BridgeClient


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
