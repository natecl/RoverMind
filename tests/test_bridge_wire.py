import json
import struct

from bridge.wire import encode_frame


def test_encode_frame_prefixes_length():
    frame = encode_frame({"id": 1, "method": "ping", "args": {}})
    # First 4 bytes are big-endian uint32 length of the rest.
    (length,) = struct.unpack(">I", frame[:4])
    assert length == len(frame) - 4
    # Payload round-trips through json.
    assert json.loads(frame[4:].decode("utf-8")) == {
        "id": 1, "method": "ping", "args": {}
    }


def test_encode_frame_rejects_non_serializable():
    import pytest
    with pytest.raises(TypeError):
        encode_frame({"value": object()})


import io

import pytest

from bridge.wire import decode_frame, MalformedFrameError, encode_frame


def test_decode_frame_round_trips_encoded_payload():
    obj = {"id": 42, "ok": True, "result": {"x": 1.5}}
    buf = io.BytesIO(encode_frame(obj))
    assert decode_frame(buf.read) == obj


def test_decode_frame_raises_on_truncated_payload():
    # Declare 10 bytes but provide 3.
    buf = io.BytesIO(struct.pack(">I", 10) + b"abc")
    with pytest.raises(MalformedFrameError, match="truncated"):
        decode_frame(buf.read)


def test_decode_frame_raises_on_bad_json():
    bad = struct.pack(">I", 3) + b"{,}"
    buf = io.BytesIO(bad)
    with pytest.raises(MalformedFrameError, match="json"):
        decode_frame(buf.read)


def test_decode_frame_raises_on_closed_socket():
    # `read` returns empty before length header is complete.
    buf = io.BytesIO(b"")
    with pytest.raises(MalformedFrameError, match="closed"):
        decode_frame(buf.read)


def test_decode_frame_handles_short_reads():
    """Real sockets can short-read; decode_frame must loop until n bytes or EOF.

    Without the internal loop, the very first read(4) for the length header
    would return only 1 byte and the decoder would falsely raise 'closed'.
    """
    src = io.BytesIO(encode_frame({"id": 99, "method": "ping", "args": {}}))

    def short_read(n):
        return src.read(1)  # at most 1 byte per call, regardless of n

    assert decode_frame(short_read) == {"id": 99, "method": "ping", "args": {}}
