# bridge/ — TCP RPC across the Python 3.8 / 3.10 boundary

Lets the Mac agent (3.10) call rover functions (3.8, rclpy + Moondream) as if local. See ADR
`context/decisions/0002-tcp-bridge-py38-py310.md` for *why*.

## Key files

- `bridge_server.py` — **rover side, Python 3.8.** Single-threaded socket loop. Dispatch table:
  `ping`, `execute_command(heading_degree, distance_m)`, `capture_and_analyze(target)`.
  Lazy-constructs `CommandExecutor`, capture fns, and `MoondreamClient` on first use. Entry:
  `main()` with `--bind host:port`. Run: `python3.8 bridge/bridge_server.py --bind 127.0.0.1:9000`.
- `client.py` — **Mac side, Python 3.10.** `BridgeClient(url, timeout_s)` context manager with
  `.ping()`, `.execute_command(...) → ExecuteResult`, `.capture_and_analyze(...) → SceneObservation`.
  Validates locally before sending. Its two methods are what the agent graph injects.
- `wire.py` — the framing: `encode_frame`/`decode_frame` = 4-byte big-endian length + UTF-8 JSON.
  Plus `scene_observation_to_dict` / `_from_dict`. **Pure stdlib, version-portable** (imported by
  both 3.8 and 3.10) — keep it dependency-free.
- `errors.py` — re-exports `CommandExecutorError`, `FrameCaptureError` (so call sites are identical
  in-process vs bridged) + adds `BridgeUnreachable`, `BridgeProtocolError`. See `context/ERRORS.md`.

## Invariants

- The wire contract is **two methods**. Anything crossing it must be JSON-serializable — extend
  `wire.py` if you add a type. `SceneObservation` / `ExecuteResult` are the current payloads.
- Server imports the real rover deps; client must not. Don't add ROS/ML imports to `client.py`/`wire.py`.
- A change here or to anything the server imports (`agent/command_executor.py`,
  `perception/vision_tool.py`) needs a **rover repo sync + bridge restart** to take effect.

## Latency timing envelope

- The server attaches `reply["timing"]` (a sibling of `result`, **not** inside the domain object)
  to every dispatched reply: `server_ms` (total handler wall-clock) always; `action_ms`
  (`_execute_command`) and `vlm_ms` + a per-Moondream-call `vlm` dict (`_capture_and_analyze`) when
  those handlers run. Timings ride in the envelope so `SceneObservation`/`ExecuteResult` stay
  timing-free. `bridge_server._pending_timing` is the per-request scratch (single-threaded → safe).
- `BridgeClient(timing_sink=...)` measures the round-trip, reads `reply["timing"]`, and forwards
  each value as `(method, name, ms)` plus `transport_overhead_ms = max(0, round_trip − server_ms)`.
  **Back-compat:** an old server omits `timing`, the client records nothing — the agent path is
  unchanged. Adding `timing` is additive: result-parsers ignore unknown envelope keys.
- This is a **server change → rover sync + bridge restart** to take effect on hardware.

## Tests

`tests/test_bridge_wire.py` (pure framing), `test_bridge_client.py` (against an in-process fake
server), `test_bridge_server.py` (injected executor/vision), `test_run_agent_bridge_wiring.py`,
`test_bridge_server_timing.py` + `test_bridge_client_timing.py` (the timing envelope).
