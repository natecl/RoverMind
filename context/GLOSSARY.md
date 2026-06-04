# Glossary — shared vocabulary

Terms and data types that cross module boundaries. Each entry names its home file; the code
is the source of truth.

## Core data types

- **`SceneObservation`** (`perception/scene_parsing.py`, frozen dataclass) — the result of one
  `look`. Fields: `target` (str), `found` (bool), `direction` (`"left"|"center"|"right"|None`),
  `distance` (`"close"|"medium"|"far"|None`), `should_stop` (bool), `raw_answers` (dict of the
  VLM's raw text), `distance_m` (float|None), `distance_source` (`"depth"|"vlm"`). Serialized
  across the bridge via `bridge/wire.py`.

- **`RoverState`** (`agent/state.py`, TypedDict) — the LangGraph agent's state. Fields:
  `messages` (LLM chat history), `task` (the raw command), `target` (extracted, e.g.
  "water bottle"), `last_observation` (`SceneObservation|None`), `step_count` (int),
  `status` (`"running"|"arrived"|"failed_max_steps"|"aborted"`), `status_message`. `messages`
  uses the `add_messages` reducer (nodes return only new messages).

- **`ExecuteResult`** (`agent/command_executor.py`, dataclass) — `success` (bool),
  `message` (str). Returned by every motion command.

- **`ExecuteCommand`** (`safety_controller_layer_interfaces/action/ExecuteCommand.action`) —
  the ROS2 action. **Goal:** `heading_degree` (float64, CCW positive), `distance_m` (float64,
  ≥0). **Result:** `success` (bool), `message` (str). **Feedback:** `phase` (`"rotating"|"driving"`).

## Vocabulary the VLM and prompt share

- **Direction buckets:** `left` / `center` / `right`. Parsed by `scene_parsing.parse_direction`.
  **"center" wins** — any mention of middle/center (even "slightly right of center") is treated
  as center → drive forward. Only a *clear* side → turn. (This was a real bug fix; see
  `AGENT_WORKFLOW.md` field notes.)
- **Distance buckets:** `close` / `medium` / `far`. From depth when available
  (`depth_math.depth_to_distance_bucket`), else the VLM's verbal estimate.
- **`should_stop`** — true when the target is centered AND close; ends the run via `stop`.

## Agent

- **The five tools** (`agent/tools.py`): `look(target)` (perceive → `SceneObservation`),
  `turn(direction, magnitude)`, `forward(distance)`, `search()` (rotate ~45° to scout),
  `stop(reason)` (end the task). Buckets → numbers via `agent/action_resolvers.py`.
- **Node loop** (`agent/nodes.py`): `init → reason → act → check`, looping while `status` is running.

## Safety / control

- **AEB (Autonomous Emergency Braking)** — `aeb_node` + `aeb_math.py`. A forward-arc lidar
  velocity gate that also **relays `/cmd_vel_raw → /cmd_vel`** (so it's mandatory for motion).
- **Hysteresis** — the brake trips when an obstacle is within `trigger_distance_m` (0.40 m) and
  only releases after the path is clear past `release_distance_m` (0.60 m) for `release_dwell_s`
  (0.5 s). The dead band prevents on/off chatter.
- **Rotate-then-drive** — `control_math.SafetyController` rotates to the heading (P-control,
  `heading_kp`) then drives the distance (open-loop), each with a computed timeout budget.

## Parameter bundles (frozen dataclasses)

- **`AgentParams` / `ActionParams`** (`agent/params.py`) — LLM + turn/forward/search magnitudes,
  loaded from `config/params.yaml` by `agent/config_loader.py`.
- **`ControllerParams`** (`safety_controller_layer/control_math.py`) — speed clamps, heading
  tolerance/gain, loop rate, timeout margins.
- **`AebParams`** (`safety_controller_layer/aeb_math.py`) — trigger/release distances, dwell,
  forward arc, output rate.

## Infra

- **The bridge** — `bridge/client.py` (Mac) ↔ `bridge/bridge_server.py` (rover), speaking
  length-prefixed JSON (`bridge/wire.py`) over an SSH-tunnelled TCP socket on port 9000.
- **Moondream2** — the local ~1.8B VLM (`perception/moondream_client.py`); `.ask(image, q)` and
  `.point(image, target)`. Runs on the rover's Jetson GPU; no cloud, no API key.
