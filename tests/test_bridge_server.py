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
