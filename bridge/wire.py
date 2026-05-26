"""Length-prefixed JSON framing used by both BridgeClient and bridge_server.

Wire format per message:
    [4 bytes big-endian uint32: payload length N][N bytes: UTF-8 JSON]

Pure stdlib; importable from both Python 3.8 (rover) and 3.10 (Mac) without
pulling in rclpy or torch.
"""

import json
import struct
from typing import Callable


class MalformedFrameError(RuntimeError):
    """Raised when a frame's declared length doesn't match its payload, or
    the payload isn't valid JSON."""


def encode_frame(obj: object) -> bytes:
    payload = json.dumps(obj).encode("utf-8")
    return struct.pack(">I", len(payload)) + payload


def decode_frame(read_exactly: Callable[[int], bytes]) -> object:
    """Read one frame.

    `read_exactly(n)` must return exactly n bytes or fewer if the stream
    closed. Raises MalformedFrameError on truncation, closed stream, or
    invalid JSON.
    """
    header = read_exactly(4)
    if len(header) < 4:
        raise MalformedFrameError("stream closed before length header complete")
    (length,) = struct.unpack(">I", header)
    payload = read_exactly(length)
    if len(payload) < length:
        raise MalformedFrameError(
            f"truncated payload: declared {length} bytes, got {len(payload)}"
        )
    try:
        return json.loads(payload.decode("utf-8"))
    except json.JSONDecodeError as exc:
        raise MalformedFrameError(f"invalid json payload: {exc}") from exc
