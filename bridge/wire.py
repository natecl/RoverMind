"""Length-prefixed JSON framing used by both BridgeClient and bridge_server.

Wire format per message:
    [4 bytes big-endian uint32: payload length N][N bytes: UTF-8 JSON]

Pure stdlib; importable from both Python 3.8 (rover) and 3.10 (Mac) without
pulling in rclpy or torch.
"""

import json
import struct


class MalformedFrameError(RuntimeError):
    """Raised when a frame's declared length doesn't match its payload, or
    the payload isn't valid JSON."""


def encode_frame(obj: object) -> bytes:
    payload = json.dumps(obj).encode("utf-8")
    return struct.pack(">I", len(payload)) + payload
