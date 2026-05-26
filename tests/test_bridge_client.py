import socket
import threading

import pytest

from agent.command_executor import CommandExecutorError, ExecuteResult
from bridge.client import BridgeClient
from bridge.errors import BridgeUnreachable
from bridge.wire import decode_frame, encode_frame
from perception.scene_parsing import SceneObservation
from perception.vision_tool import FrameCaptureError


def _serve_one(listener, handler):
    """Accept one connection, hand its socket to `handler`, then close."""
    def run():
        client_sock, _ = listener.accept()
        try:
            with client_sock.makefile("rwb", buffering=0) as stream:
                handler(stream)
        finally:
            client_sock.close()
    t = threading.Thread(target=run, daemon=True)
    t.start()
    return t


@pytest.fixture
def loopback_server():
    listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listener.bind(("127.0.0.1", 0))
    listener.listen(1)
    yield listener
    listener.close()


def test_ping_round_trips(loopback_server):
    def handler(stream):
        req = decode_frame(stream.read)
        assert req["method"] == "ping"
        stream.write(encode_frame({"id": req["id"], "ok": True, "result": "pong"}))

    _serve_one(loopback_server, handler)
    host, port = loopback_server.getsockname()
    with BridgeClient(f"tcp://{host}:{port}") as client:
        assert client.ping() == "pong"


def test_connect_refused_raises_bridge_unreachable():
    # Bind, then close, so the port is guaranteed-free.
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    _, port = s.getsockname()
    s.close()
    with pytest.raises(BridgeUnreachable):
        BridgeClient(f"tcp://127.0.0.1:{port}").__enter__()


def test_execute_command_round_trips_result(loopback_server):
    def handler(stream):
        req = decode_frame(stream.read)
        assert req["method"] == "execute_command"
        assert req["args"] == {"heading_degree": 45.0, "distance_m": 1.0}
        stream.write(encode_frame({
            "id": req["id"], "ok": True,
            "result": {"success": True, "message": "completed"},
        }))

    _serve_one(loopback_server, handler)
    host, port = loopback_server.getsockname()
    with BridgeClient(f"tcp://{host}:{port}") as client:
        result = client.execute_command(heading_deg=45.0, distance_m=1.0)
    assert result == ExecuteResult(success=True, message="completed")


def test_execute_command_validates_before_send(loopback_server):
    # Handler asserts it is never invoked.
    def handler(stream):
        raise AssertionError("client should not have sent a request")

    _serve_one(loopback_server, handler)
    host, port = loopback_server.getsockname()
    with BridgeClient(f"tcp://{host}:{port}") as client:
        with pytest.raises(ValueError, match="distance_m must be non-negative"):
            client.execute_command(heading_deg=0.0, distance_m=-0.5)


def test_execute_command_maps_ros_unavailable_to_executor_error(loopback_server):
    def handler(stream):
        req = decode_frame(stream.read)
        stream.write(encode_frame({
            "id": req["id"], "ok": False,
            "error": {"type": "ros_action_unavailable",
                       "message": "action server 'execute_command' not available within 5.0s"},
        }))

    _serve_one(loopback_server, handler)
    host, port = loopback_server.getsockname()
    with BridgeClient(f"tcp://{host}:{port}") as client:
        with pytest.raises(CommandExecutorError, match="not available"):
            client.execute_command(heading_deg=0.0, distance_m=0.5)


def test_capture_and_analyze_round_trips_scene_observation(loopback_server):
    def handler(stream):
        req = decode_frame(stream.read)
        assert req["method"] == "capture_and_analyze"
        assert req["args"] == {"target": "water bottle"}
        stream.write(encode_frame({
            "id": req["id"], "ok": True,
            "result": {
                "target": "water bottle", "found": True,
                "direction": "left", "distance": "close",
                "should_stop": True, "raw_answers": {"visible": "yes"},
                "distance_m": 0.42, "distance_source": "depth",
            },
        }))

    _serve_one(loopback_server, handler)
    host, port = loopback_server.getsockname()
    with BridgeClient(f"tcp://{host}:{port}") as client:
        obs = client.capture_and_analyze("water bottle")
    assert obs.target == "water bottle"
    assert obs.found is True
    assert obs.direction == "left"
    assert obs.distance_source == "depth"


def test_capture_and_analyze_maps_vision_error(loopback_server):
    def handler(stream):
        req = decode_frame(stream.read)
        stream.write(encode_frame({
            "id": req["id"], "ok": False,
            "error": {"type": "vision_error",
                       "message": "no frame on /camera/color/image_raw within 5.0s"},
        }))

    _serve_one(loopback_server, handler)
    host, port = loopback_server.getsockname()
    with BridgeClient(f"tcp://{host}:{port}") as client:
        with pytest.raises(FrameCaptureError, match="no frame"):
            client.capture_and_analyze("water bottle")
