# RoverMind
# LIMO VLM Agent — Agentic Vision-Language Navigation for the LIMO Pro Rover

A hybrid agentic navigation system that enables a [LIMO Agilex Pro](https://www.agilex.ai/chassis/5) rover to execute natural language driving commands using a Vision-Language Model (VLM) powered agent built with [LangGraph](https://github.com/langchain-ai/langgraph). A user says *"drive to the water bottle"*, and the rover autonomously locates, navigates to, and stops at the target object.

The system uses a **two-layer hybrid architecture**: a VLM agent loop handles high-level perception and reasoning (~1 Hz), while a real-time controller handles low-level motor execution (~10–30 Hz). This separation ensures the rover remains safe and responsive even during the agent's multi-second reasoning steps.

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
    task: str                # Original natural language command
    target_object: str       # Extracted target ("water bottle")
    observation: str         # Latest VLM scene description
    reasoning: str           # Agent's current reasoning
    last_command: dict       # {"heading": float, "distance": float}
    step_count: int          # Number of observe-act cycles completed
    status: Literal["searching", "approaching", "arrived", "failed"]
```

## Agent Tools

The agent has access to three tools:

| Tool | Description | Returns |
|---|---|---|
| `capture_and_analyze(target)` | Captures a camera frame and asks the local Moondream2 VLM where the target object is (left/center/right) and how far away it is (close/medium/far) | Structured `SceneObservation` |
| `move(heading_degrees, distance_meters)` | Issues a high-level movement command to the controller layer. Values are clamped for safety | Confirmation string |
| `stop_and_report(message)` | Stops the rover and terminates the agent loop | Final status message |

## Setup

### Prerequisites

- LIMO Agilex Pro with Jetson Orin Nano running JetPack 6.x
- ROS2 Foxy installed
- Python 3.10+
- The Moondream2 model weights (downloaded automatically from Hugging Face on first run)

### Installation

```bash
# SSH into the LIMO Pro
ssh agilex@<rover-ip>

# Clone the repo
git clone https://github.com/<your-username>/limo-vlm-agent.git
cd limo-vlm-agent

# Install Python dependencies
pip install -r requirements.txt --break-system-packages

# The Moondream2 VLM runs locally on the Orin — no API key needed.
# Its weights download from Hugging Face on the first run.

# Set the LIMO to four-wheel differential mode (simplest control)
# (use the physical mode switch on the rover)
```

### Configuration

Edit `config/params.yaml` to match your setup:

```yaml
topics:
  rgb_image: "/camera/color/image_raw"
  scan: "/scan"                 # 2D lidar, watched by the emergency braking gate
  cmd_vel_raw: "/cmd_vel_raw"   # controller output, into the braking gate
  cmd_vel: "/cmd_vel"           # braking gate output, to the LIMO base driver

controller:
  max_linear_speed: 0.3       # m/s — do not exceed for indoor use
  max_angular_speed: 0.5      # rad/s
  stop_distance: 0.4          # meters from target to stop

agent:
  vlm_model: "vikhyatk/moondream2"   # local VLM, runs on the Orin
  vlm_revision: "2025-06-21"         # pinned Moondream2 release
  max_steps: 20               # safety limit on reasoning cycles
  reasoning_interval: 2.0     # seconds between agent steps
```

### Running

```bash
# Terminal 1 — start the LIMO base drivers
ros2 launch limo_bringup limo_start.launch.py

# Terminal 2 — start the RoverMind nodes (safety controller + emergency braking gate)
ros2 launch safety_controller_layer rovermind.launch.py

# Terminal 3 — start the agent
export OPENAI_API_KEY=sk-...
python scripts/run_agent.py "drive to the water bottle"
```

For bring-up debugging you can skip the braking gate with
`ros2 launch safety_controller_layer rovermind.launch.py use_aeb:=false`.

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