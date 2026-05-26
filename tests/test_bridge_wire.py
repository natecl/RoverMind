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
