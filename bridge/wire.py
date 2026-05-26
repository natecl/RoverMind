"""Length-prefixed JSON framing used by both BridgeClient and bridge_server.

Wire format per message:
    [4 bytes big-endian uint32: payload length N][N bytes: UTF-8 JSON]

Pure stdlib; importable from both Python 3.8 (rover) and 3.10 (Mac) without
pulling in rclpy or torch.
"""

import json
import struct
from dataclasses import asdict
from typing import Callable


class MalformedFrameError(RuntimeError):
    """Raised when a frame's declared length doesn't match its payload, or
    the payload isn't valid JSON."""


def encode_frame(obj: object) -> bytes:
    payload = json.dumps(obj).encode("utf-8")
    return struct.pack(">I", len(payload)) + payload


def decode_frame(read: Callable[[int], bytes]) -> object:
    """Read one frame from `read`.

    `read(n)` is a stream read function that returns up to n bytes — it may
    short-read (e.g. socket reads do this when packets are fragmented). An
    empty return value is treated as EOF. Raises MalformedFrameError on
    closed stream, truncated payload, or invalid JSON.
    """
    header = _read_exactly(read, 4)
    if header is None:
        raise MalformedFrameError("stream closed before length header complete")
    (length,) = struct.unpack(">I", header)
    payload = _read_exactly(read, length)
    if payload is None:
        raise MalformedFrameError(
            f"truncated payload: declared {length} bytes, stream closed early"
        )
    try:
        return json.loads(payload.decode("utf-8"))
    except json.JSONDecodeError as exc:
        raise MalformedFrameError(f"invalid json payload: {exc}") from exc


def _read_exactly(read: Callable[[int], bytes], n: int):
    """Loop `read` until n bytes received, or return None on EOF.

    Necessary because Python's unbuffered SocketIO and raw socket reads can
    return fewer bytes than requested even when the stream is still open.
    """
    buf = bytearray()
    while len(buf) < n:
        chunk = read(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def scene_observation_to_dict(obs):
    """Serialize SceneObservation to a JSON-safe dict (just dataclasses.asdict).

    Kept here so the wire format is owned by one module; SceneObservation
    itself stays a pure dataclass with no JSON awareness.
    """
    return asdict(obs)


def scene_observation_from_dict(payload):
    from perception.scene_parsing import SceneObservation
    return SceneObservation(
        target=payload["target"],
        found=bool(payload["found"]),
        direction=payload.get("direction"),
        distance=payload.get("distance"),
        should_stop=bool(payload["should_stop"]),
        raw_answers=dict(payload.get("raw_answers", {})),
        distance_m=payload.get("distance_m"),
        distance_source=payload.get("distance_source", "vlm"),
    )
