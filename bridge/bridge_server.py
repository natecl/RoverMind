"""Rover-side bridge server.

Single-threaded socket loop — one connected client at a time, one request in
flight. Methods are looked up in a dispatch table built at construction; the
rclpy CommandExecutor and the MoondreamClient are constructed lazily on the
first call that needs them, so ping-only smoke tests don't pay startup cost.

Run on the rover with:
    python3.8 bridge/bridge_server.py --bind 127.0.0.1:9000
"""

import argparse
import socket
import sys
from typing import Callable, Optional

from bridge.wire import decode_frame, encode_frame, MalformedFrameError


class _BridgeError(Exception):
    """Internal: typed error a handler can raise to produce a structured reply."""
    def __init__(self, type_: str, message: str):
        super().__init__(f"{type_}: {message}")
        self.type = type_
        self.message = message


class BridgeServer:
    def __init__(self,
                 host: str,
                 port: int,
                 command_executor=None,
                 capture_fn: Optional[Callable] = None,
                 depth_fn: Optional[Callable] = None,
                 moondream_factory: Optional[Callable] = None):
        self._host = host
        self._port = port
        self._listener: Optional[socket.socket] = None
        self._shutdown = False
        self._command_executor = command_executor
        self._capture_fn = capture_fn
        self._depth_fn = depth_fn
        self._moondream_factory = moondream_factory
        self._moondream = None
        self.bound_port: Optional[int] = None
        self._methods = {"ping": self._ping}
        self._methods["execute_command"] = self._execute_command

    def serve_forever(self) -> None:
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind((self._host, self._port))
        self._listener.listen(1)
        self.bound_port = self._listener.getsockname()[1]
        self._listener.settimeout(0.25)
        try:
            while not self._shutdown:
                try:
                    client_sock, _ = self._listener.accept()
                except socket.timeout:
                    continue
                try:
                    self._serve_one_connection(client_sock)
                finally:
                    client_sock.close()
        finally:
            self._listener.close()
            self._listener = None

    def shutdown(self) -> None:
        self._shutdown = True

    def _serve_one_connection(self, client_sock: socket.socket) -> None:
        with client_sock.makefile("rwb", buffering=0) as stream:
            while not self._shutdown:
                try:
                    request = decode_frame(stream.read)
                except MalformedFrameError:
                    return  # client went away or sent garbage; just close
                reply = self._handle(request)
                try:
                    stream.write(encode_frame(reply))
                except OSError:
                    return

    def _handle(self, request) -> dict:
        if not isinstance(request, dict) or "id" not in request:
            return {"id": None, "ok": False,
                    "error": {"type": "malformed_request", "message": str(request)[:200]}}
        request_id = request["id"]
        method = request.get("method")
        args = request.get("args", {}) or {}
        handler = self._methods.get(method)
        if handler is None:
            return {"id": request_id, "ok": False,
                    "error": {"type": "unknown_method", "message": method or ""}}
        try:
            result = handler(**args)
        except _BridgeError as exc:
            return {"id": request_id, "ok": False,
                    "error": {"type": exc.type, "message": exc.message}}
        except Exception as exc:
            return {"id": request_id, "ok": False,
                    "error": {"type": "internal_error", "message": f"{type(exc).__name__}: {exc}"}}
        return {"id": request_id, "ok": True, "result": result}

    def _ping(self) -> str:
        return "pong"

    def _execute_command(self, heading_degree: float, distance_m: float):
        from agent.command_executor import CommandExecutorError, ExecuteResult
        if self._command_executor is None:
            # Lazy-import rclpy + construct the real executor on first call.
            from agent.command_executor import CommandExecutor
            self._command_executor = CommandExecutor()
        try:
            result = self._command_executor.execute(heading_deg=heading_degree,
                                                    distance_m=distance_m)
        except CommandExecutorError as exc:
            raise _BridgeError("ros_action_unavailable", str(exc)) from exc
        return {"success": bool(result.success), "message": str(result.message)}


def _parse_bind(spec: str):
    host, _, port = spec.partition(":")
    return host, int(port)


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="RoverMind Python 3.8 ROS bridge")
    parser.add_argument("--bind", default="127.0.0.1:9000",
                        help="host:port to listen on (default 127.0.0.1:9000)")
    ns = parser.parse_args(argv)
    host, port = _parse_bind(ns.bind)
    server = BridgeServer(host=host, port=port)
    print(f"[bridge] listening on tcp://{host}:{port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("[bridge] shutting down", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
