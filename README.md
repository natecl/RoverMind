# RoverMind
# LIMO VLM Agent — Agentic Vision-Language Navigation for the LIMO Pro Rover

A hybrid agentic navigation system that enables a [LIMO Agilex Pro](https://www.agilex.ai/chassis/5) rover to execute natural language driving commands using a Vision-Language Model (VLM) powered agent built with [LangGraph](https://github.com/langchain-ai/langgraph). A user says *"drive to the water bottle"*, and the rover autonomously locates, navigates to, and stops at the target object.

The system uses a **two-layer hybrid architecture**: a VLM agent loop handles high-level perception and reasoning (~1 Hz), while a real-time controller handles low-level motor execution (~10–30 Hz). This separation ensures the rover remains safe and responsive even during the agent's multi-second reasoning steps.

> **AI agents / new contributors:** start at [`CLAUDE.md`](CLAUDE.md) (orientation + golden rules),
> then [`context/`](context/) for architecture, glossary, errors, environment, and decision records.
> Each major module has its own `CLAUDE.md`. This README is a human-facing overview and may lag the
> code — `context/` and the module `CLAUDE.md` files are the maintained, agent-facing source.

## Architecture

```
 "Drive to the water bottle"
              │
              ▼
┌──────────────────────────────────────┐
│        LangGraph Agent Loop          │
│                                      │
│   OBSERVE ──→ REASON ──→ ACT        │
│      ↑                    │          │
│      └──── CHECK ←────────┘          │
│               │                      │
│            arrived                   │
│               ▼                      │
│             STOP                     │
└──────────────┬───────────────────────┘
               │  heading + distance
               ▼
┌──────────────────────────────────────┐
│     Real-Time Controller (ROS2)      │
│     PID · speed clamping · safety    │
│   Publishes /cmd_vel_raw @ 10-30Hz   │
└──────────────────────────────────────┘
               │  /cmd_vel_raw
               ▼
┌──────────────────────────────────────┐
│    Emergency Braking Gate (ROS2)     │
│   lidar /scan · forward-arc check    │
│  zeroes forward speed · pub /cmd_vel │
└──────────────────────────────────────┘
               │  /cmd_vel
               ▼
          LIMO Pro Rover
```

### Why Hybrid?

| Concern | Full Agent | Hybrid |
|---|---|---|
| Latency between decisions | ~1.5–3s uncontrolled driving | Controller holds smooth trajectory |
| Safety | LLM can output unsafe speeds | Controller clamps all outputs |
| Determinism | Non-deterministic steering | Predictable low-level execution |
| Debugging | Chain-of-thought log reading | PID error plots + reasoning logs |
| Demo reliability | ~60% success rate | ~95% success rate |

## Project Structure

```
RoverMind/
├── README.md
├── requirements.txt
├── config/
│   └── params.yaml                       # LLM + action-resolver tunables (loaded by the agent)
├── agent/
│   ├── graph.py                          # LangGraph state machine definition
│   ├── state.py                          # RoverState schema
│   ├── nodes.py                          # Graph nodes: init, reason, act, check
│   ├── tools.py                          # Agent tools: look, turn, forward, search, stop
│   └── command_executor.py               # Sync wrapper around the ExecuteCommand ROS2 action
├── perception/
│   ├── scene_parsing.py                  # SceneObservation + pure answer parsers
│   ├── depth_math.py                     # depth-sample → metric distance helpers
│   ├── moondream_client.py               # local Moondream2 VLM wrapper
│   └── vision_tool.py                    # capture_and_analyze orchestration + ROS capture
├── safety_controller_layer/              # ROS2 ament_python package
│   ├── control_math.py                   # SafetyController (pure logic, unit-tested)
│   ├── safety_controller_node.py         # ExecuteCommand action server (rotate-then-drive)
│   ├── aeb_math.py                       # Forward-arc brake logic (pure, unit-tested)
│   └── aeb_node.py                       # Lidar-gated /cmd_vel_raw → /cmd_vel republisher
├── safety_controller_layer_interfaces/   # ROS2 ament_cmake package (action definitions)
│   └── action/ExecuteCommand.action
├── scripts/
│   ├── run_agent.py                      # Main entry point
│   ├── test_agent_static.py              # Agent loop against fake observations
│   └── test_vision.py                    # Test VLM perception independently
└── launch/
    └── rovermind.launch.py               # Brings up safety_controller_node + aeb_node
```

## Tech Stack

- **Agent Framework:** LangGraph (stateful agent graph with cycles, checkpointing, tool use)
- **VLM Backbone:** Moondream2 (~1.8B, runs locally on the Jetson Orin Nano)
- **Robotics Middleware:** ROS2 Foxy
- **Hardware:** LIMO Agilex Pro with NVIDIA Jetson Orin Nano (8GB)
- **Controller:** Proportional/PID control with safety clamping
- **Language:** Python 3.10+

## Agent State Schema

The LangGraph agent maintains the following state across reasoning steps:

```python
class RoverState(TypedDict):
    messages: Annotated[list[BaseMessage], add_messages]  # LLM chat history
    task: str                # Original natural-language command
    target: str              # Extracted target ("water bottle")
    last_observation: Optional[SceneObservation]          # latest structured scene
    step_count: int          # Number of observe-act cycles completed
    status: Literal["running", "arrived", "failed_max_steps", "aborted"]
    status_message: str      # Human-readable terminal detail
```

## Agent Tools

The agent has access to five tools (built in `agent/tools.py`):

| Tool | Description | Returns |
|---|---|---|
| `look(target)` | Captures a camera frame and asks the local Moondream2 VLM where the target is (left/center/right) and how far (close/medium/far). Call before every move. | Formatted observation string (stores a `SceneObservation`) |
| `turn(direction, magnitude)` | Turn in place — `direction` left/right, `magnitude` small (~30°) / large (~60°). | Confirmation string |
| `forward(distance)` | Drive forward — `distance` short (~0.3 m) / medium (~0.6 m). | Confirmation string |
| `search()` | Rotate ~45° in place to look for the target when `look` reports not-found. | Confirmation string |
| `stop(reason)` | Stop the rover and end the task once centered and close. | Final status message |

Magnitudes resolve to numbers via `agent/action_resolvers.py` using `config/params.yaml`.

## Setup

The agent runs on **your Mac**; ROS2, Moondream2, and the bridge run **on the rover**. The two
talk over an SSH-tunnelled TCP bridge, which lets the Mac stay on Python 3.10 (no ROS/ML imports)
while the rover stays on its Python 3.8 ROS2 stack — see
[`context/decisions/0002-tcp-bridge-py38-py310.md`](context/decisions/0002-tcp-bridge-py38-py310.md).

### Prerequisites

- **Rover:** LIMO Agilex Pro with Jetson Orin Nano (8 GB, JetPack 6.x), ROS2 Foxy, Python 3.8.
  Moondream2 weights (auto-download from Hugging Face on first run, or a local snapshot via
  `MOONDREAM_MODEL_PATH`). No API key needed — the VLM runs locally on the Orin.
- **Developer Mac:** Python 3.10+, an OpenAI API key, SSH access to the rover.

### Installation

**On your Mac** (the agent side):

```bash
git clone https://github.com/natecl/RoverMind.git
cd RoverMind
python3 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env          # then put your OPENAI_API_KEY in .env (gitignored)
```

**On the rover** (the ROS2 + perception side — full, verified procedure in
[`LIMO_WORKFLOW.md`](LIMO_WORKFLOW.md)):

```bash
git clone https://github.com/natecl/RoverMind.git ~/RoverMind
# Symlink the two ROS2 packages into a colcon workspace and build (LIMO_WORKFLOW.md §1):
#   safety_controller_layer (ament_python) + safety_controller_layer_interfaces (ament_cmake)
pip3 install -r ~/RoverMind/requirements.txt --break-system-packages
# Set the LIMO to four-wheel differential mode via the physical mode switch.
```

### Configuration

Edit `config/params.yaml` to tune the agent and its action magnitudes (loaded by
`agent/config_loader.py` into `AgentParams` / `ActionParams`):

```yaml
agent:
  llm_provider: openai
  llm_model: gpt-4o-mini
  llm_temperature: 0.0
  max_steps: 20            # safety limit on reasoning cycles

actions:
  turn_small_deg: 30.0     # `turn(..., "small")`
  turn_large_deg: 60.0     # `turn(..., "large")`
  search_deg: 45.0         # `search()` rotation
  forward_short_m: 0.3     # `forward("short")`
  forward_medium_m: 0.6    # `forward("medium")`
```

Controller limits (speed clamps, heading gain/tolerance) live in
`safety_controller_layer/control_math.py` (`ControllerParams`); the emergency-brake
distances live in `aeb_math.py` (`AebParams`). ROS topic names are fixed in the nodes.

### Running

Bring up the stack **on the rover**, open the tunnel, then run the agent **on your Mac**. The
detailed, field-verified procedure (SSH multiplexing, detached launchers, smoke gates) is in
[`AGENT_WORKFLOW.md`](AGENT_WORKFLOW.md) and [`LIMO_WORKFLOW.md`](LIMO_WORKFLOW.md); the short version:

```bash
# ON THE ROVER (one terminal each; source ROS2 + the overlay first):
ros2 launch limo_bringup limo_start.launch.py                       # base + lidar + /odom /imu
ros2 launch safety_controller_layer rovermind.launch.py            # controller + AEB (keep AEB ON)
ros2 launch orbbec_camera dabai_dcw2.launch.py                     # camera (a SEPARATE launch)
python3.8 ~/RoverMind/bridge/bridge_server.py --bind 127.0.0.1:9000   # the Python 3.8 bridge

# ON YOUR MAC:
scripts/rover_connect.sh --open        # find the rover by MAC and open the SSH tunnel (-L 9000:localhost:9000)
source .venv/bin/activate
python scripts/run_agent.py "drive to the water bottle"   # OPENAI_API_KEY auto-loaded from .env
#   add --bridge tcp://localhost:9000 to point at a non-default tunnel
```

> ⚠️ **Always launch with AEB on** (no `use_aeb:=false`). `aeb_node` is the only thing republishing
> `/cmd_vel_raw → /cmd_vel`, so disabling it leaves the motors with **no publisher** and the rover
> won't move. See [`context/decisions/0003-aeb-is-the-cmd-vel-relay.md`](context/decisions/0003-aeb-is-the-cmd-vel-relay.md).

## Build Phases

### Phase 1 — Controller Layer ✅
Get the rover driving reliably from Python. Publish to `/cmd_vel`, implement `execute_command(heading, distance)` with speed clamping. Test with hardcoded commands. No AI.

### Phase 2 — Vision Tool ✅
`capture_and_analyze(target)` captures a camera frame, asks a local Moondream2 VLM where the target is and how far away it is, and returns a structured `SceneObservation`. Distance uses the depth camera with a VLM fallback. Verified via `scripts/test_vision.py`.

### Phase 3 — LangGraph Agent ✅
Build the state graph with observe → reason → act → check nodes. Test the reasoning loop with static images before connecting to the live rover.

### Phase 4 — Integration & Tuning
Connect all layers end-to-end. Tune prompts for reliable spatial descriptions. Add step limits and timeout behavior. Record demo video.

## Roadmap

- [x] Project architecture design
- [x] Phase 1: Controller layer with PID and safety clamping
- [x] Phase 2: Vision tool — `capture_and_analyze` with local Moondream2
- [x] Phase 3: LangGraph agent state machine (OpenAI-backed ReAct loop with look/turn/forward/search/stop tools)
- [ ] Phase 4: End-to-end integration and prompt tuning
- [x] Autonomous emergency braking — lidar forward-arc velocity gate
- [ ] Obstacle awareness via LIMO Pro's onboard lidar
- [ ] Multi-step task execution ("go to X, then come back")
- [x] Edge VLM — vision tool runs Moondream2 locally on the Orin (no cloud)
- [ ] Real-time web dashboard showing agent reasoning chain
- [ ] Voice command input via microphone

## Key Design Decisions

**Why LangGraph over vanilla LangChain AgentExecutor?**
AgentExecutor is a black-box ReAct loop. LangGraph gives explicit control over the execution graph, letting us inject safety checks between steps, handle the async timing between agent and controller, and implement structured termination conditions. For a physical robot, that control is non-negotiable.

**Why a local VLM (Moondream2), not cloud?**
Moondream2 (~1.8B) runs entirely on the Orin Nano — no network dependency, no API key, no per-call latency or cost. It is small enough to leave memory headroom for the rest of the stack while still being a genuinely conversational VLM. The hybrid architecture runs perception at ~1 Hz, so its ~1.5–3 s inference is well within budget.

**Why hybrid instead of full agentic control?**
At ~1.5–3 seconds per VLM call, the rover drives roughly half a metre blind between decisions. The controller layer keeps the rover safe during reasoning gaps. It also makes the system debuggable — PID error plots for motor issues, reasoning logs for agent issues — rather than having everything tangled in one opaque loop.

## License

MIT