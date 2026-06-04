"""Phase 2a: BridgeClient forwards reply-envelope timing to a timing_sink.

The client extracts the server's `timing` block (and computes
`transport_overhead_ms = round_trip − server_ms`) and pushes each value to the
injected sink as (method, name, ms). With no `timing` key on the envelope
(an old server), the sink is left untouched — proving back-compat.
"""

import socket
import threading

import pytest

from bridge.client import BridgeClient
from bridge.wire import decode_frame, encode_frame


def _serve_one(listener, handler):
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


def test_call_forwards_envelope_timing_to_sink(loopback_server):
    def handler(stream):
        req = decode_frame(stream.read)
        stream.write(encode_frame({
            "id": req["id"], "ok": True,
            "result": {"success": True, "message": "completed"},
            "timing": {"server_ms": 5.0, "action_ms": 4.0},
        }))

    _serve_one(loopback_server, handler)
    host, port = loopback_server.getsockname()
    sink = []
    with BridgeClient(f"tcp://{host}:{port}",
                      timing_sink=lambda m, n, v: sink.append((m, n, v))) as client:
        client.execute_command(heading_deg=10.0, distance_m=0.5)

    by_name = {n: v for (m, n, v) in sink}
    assert all(m == "execute_command" for (m, n, v) in sink)
    assert by_name["server_ms"] == 5.0
    assert by_name["action_ms"] == 4.0
    assert "round_trip_ms" in by_name
    assert "transport_overhead_ms" in by_name
    # round_trip >= server_ms in the normal case, so overhead is non-negative.
    assert by_name["transport_overhead_ms"] >= 0.0


def test_call_forwards_nested_timing_dict(loopback_server):
    def handler(stream):
        req = decode_frame(stream.read)
        stream.write(encode_frame({
            "id": req["id"], "ok": True,
            "result": {"target": "x", "found": False, "should_stop": False,
                       "raw_answers": {}},
            "timing": {"server_ms": 9.0, "vlm_ms": 8.0,
                       "vlm": {"ask_visible": 3.0, "ask_direction": 5.0}},
        }))

    _serve_one(loopback_server, handler)
    host, port = loopback_server.getsockname()
    sink = []
    with BridgeClient(f"tcp://{host}:{port}",
                      timing_sink=lambda m, n, v: sink.append((m, n, v))) as client:
        client.capture_and_analyze("x")

    by_name = {n: v for (m, n, v) in sink}
    assert by_name["vlm.ask_visible"] == 3.0
    assert by_name["vlm.ask_direction"] == 5.0
    assert by_name["vlm_ms"] == 8.0


def test_call_without_timing_key_records_nothing(loopback_server):
    def handler(stream):
        req = decode_frame(stream.read)
        stream.write(encode_frame({
            "id": req["id"], "ok": True,
            "result": {"success": True, "message": "ok"},
        }))

    _serve_one(loopback_server, handler)
    host, port = loopback_server.getsockname()
    sink = []
    with BridgeClient(f"tcp://{host}:{port}",
                      timing_sink=lambda m, n, v: sink.append((m, n, v))) as client:
        client.execute_command(heading_deg=0.0, distance_m=0.5)
    assert sink == []


def test_transport_overhead_clamped_nonnegative(loopback_server):
    def handler(stream):
        req = decode_frame(stream.read)
        stream.write(encode_frame({
            "id": req["id"], "ok": True,
            "result": {"success": True, "message": "ok"},
            "timing": {"server_ms": 999999.0},  # absurdly larger than round-trip
        }))

    _serve_one(loopback_server, handler)
    host, port = loopback_server.getsockname()
    sink = []
    with BridgeClient(f"tcp://{host}:{port}",
                      timing_sink=lambda m, n, v: sink.append((m, n, v))) as client:
        client.execute_command(heading_deg=0.0, distance_m=0.5)
    by_name = {n: v for (m, n, v) in sink}
    assert by_name["transport_overhead_ms"] == 0.0


def test_non_numeric_timing_values_do_not_crash_the_call(loopback_server):
    def handler(stream):
        req = decode_frame(stream.read)
        stream.write(encode_frame({
            "id": req["id"], "ok": True,
            "result": {"success": True, "message": "ok"},
            # A malformed / version-mismatched server: non-numeric + over-nested.
            "timing": {"server_ms": 5.0, "action_ms": "n/a",
                       "vlm": {"ask": {"too": "deep"}}},
        }))

    _serve_one(loopback_server, handler)
    host, port = loopback_server.getsockname()
    sink = []
    with BridgeClient(f"tcp://{host}:{port}",
                      timing_sink=lambda m, n, v: sink.append((m, n, v))) as client:
        result = client.execute_command(heading_deg=0.0, distance_m=0.5)  # must not raise
    assert result.success is True
    by_name = {n: v for (m, n, v) in sink}
    # Numeric values still recorded; un-coercible ones silently dropped.
    assert by_name["server_ms"] == 5.0
    assert "action_ms" not in by_name
    assert "vlm.ask" not in by_name


def test_no_sink_is_a_noop(loopback_server):
    def handler(stream):
        req = decode_frame(stream.read)
        stream.write(encode_frame({
            "id": req["id"], "ok": True,
            "result": {"success": True, "message": "ok"},
            "timing": {"server_ms": 5.0},
        }))

    _serve_one(loopback_server, handler)
    host, port = loopback_server.getsockname()
    # No timing_sink: must not raise.
    with BridgeClient(f"tcp://{host}:{port}") as client:
        result = client.execute_command(heading_deg=0.0, distance_m=0.5)
    assert result.success is True
