# Architecture — how the pieces fit

For *why* the design is shaped this way, see `decisions/` and the README's "Key Design
Decisions". This file is the **map**: data flow, nodes/topics, and the two boundaries that
matter most. Source of truth is the code — paths are given so you can check.

## One sentence

A natural-language task ("drive to the water bottle") drives a LangGraph agent loop on the
**Mac** that, over a TCP bridge, asks the **rover** to perceive (Moondream2 VLM) and to move
(a real-time ROS2 controller gated by emergency braking).

## End-to-end flow

```
"drive to the water bottle"
  │
  ▼  scripts/run_agent.py  (Mac, Python 3.10 venv)
  agent/graph.py loop:  init → reason → act → check → (running? reason : END)
  agent/tools.py  five tools: look · turn · forward · search · stop
  │
  │  bridge/client.py  ── TCP length-prefixed JSON (bridge/wire.py) over SSH tunnel :9000 ──►
  ▼
  bridge/bridge_server.py  (rover, Python 3.8)   dispatches two methods:
    ├─ execute_command(heading_deg, distance_m)
    │     agent/command_executor.py → ROS2 ExecuteCommand action client
    │       └─► safety_controller_node  (rotate-then-drive, P-control + timeouts)
    │             publishes /cmd_vel_raw
    │               └─► aeb_node  (lidar forward-arc brake, hysteresis)
    │                     publishes /cmd_vel ──► limo_base motors
    └─ capture_and_analyze(target)
          perception/vision_tool.py → Moondream2 (.ask/.point) + optional depth
            └─► perception/scene_parsing.py builds a SceneObservation (returned to the Mac)
  │
  ▼  sensors (/imu /odom /camera/*  on the rover) feed the next controller move and the next look()
```

## ROS2 nodes & topics

| Node | Subscribes | Publishes | Action server | Role |
|------|-----------|-----------|---------------|------|
| `safety_controller_node` | `/imu`, `/odom` | `/cmd_vel_raw` | `execute_command` | rotate-to-heading then drive-distance; P-control + timeout safety |
| `aeb_node` | `/cmd_vel_raw`, `/scan` | `/cmd_vel` | — | forward-arc emergency brake; **the relay that actually reaches the motors** |
| `limo_bringup` (external) | `/cmd_vel` | `/imu`, `/odom`, `/scan`, `/camera/*` | — | LIMO base driver + sensors (separate launch) |

`bridge_server.py` and `run_agent.py` are **not** ROS2 nodes — they're plain Python on
either side of the TCP bridge.

> **Critical invariant:** `safety_controller` publishes only `/cmd_vel_raw`; `limo_base`
> subscribes only to `/cmd_vel`; `aeb_node` is the bridge between them. Running with
> `use_aeb:=false` leaves `/cmd_vel` with **0 publishers** → no motion. **Always run AEB on.**
> See ADR `decisions/0003-aeb-is-the-cmd-vel-relay.md`.

## The two boundaries that shape everything

1. **The bridge boundary (process / Python version).** The Mac runs Python 3.10 with **no
   ROS or ML imports**; the rover runs Python 3.8 with rclpy + torch + Moondream. Everything
   that needs hardware lives behind `bridge/`. A change to perception/parsing or the controller
   needs a **rover deploy + bridge restart**; a change to `agent/prompts.py` only needs to be on
   the Mac. See ADR `0002`.

2. **The pure/impure split (testability).** `*_math.py`, `perception/scene_parsing.py`, and
   `safety_controller_layer/control_math.py` are **pure** — no I/O, laptop-testable, where the
   real logic lives. The `*_node.py` / `*_server.py` wrappers do the I/O (rclpy, sockets,
   cameras) and are hardware-only. Logic is injected into wrappers via callables (dependency
   injection), so tests use fakes. Put new logic in the pure layer.

## Data crossing the bridge

`SceneObservation` and `ExecuteResult` are serialized to/from dicts over the wire
(`bridge/wire.py`). Their fields are the contract — see `GLOSSARY.md`.
