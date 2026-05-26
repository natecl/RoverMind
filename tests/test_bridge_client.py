import socket
import threading

import pytest

from bridge.client import BridgeClient
from bridge.errors import BridgeUnreachable
from bridge.wire import decode_frame, encode_frame


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
