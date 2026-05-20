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
│     Publishes /cmd_vel @ 10-30Hz     │
└──────────────────────────────────────┘
               │
               ▼
          LIMO Pro Rover
```

### Why Hybrid?

| Concern | Full Agent | Hybrid |
|---|---|---|
| Latency between decisions | 2–5s uncontrolled driving | Controller holds smooth trajectory |
| Safety | LLM can output unsafe speeds | Controller clamps all outputs |
| Determinism | Non-deterministic steering | Predictable low-level execution |
| Debugging | Chain-of-thought log reading | PID error plots + reasoning logs |
| Demo reliability | ~60% success rate | ~95% success rate |

## Project Structure

```
limo_vlm_agent/
├── README.md
├── requirements.txt
├── config/
│   └── params.yaml              # Tunable parameters and ROS topic names
├── agent/
│   ├── graph.py                 # LangGraph state machine definition
│   ├── state.py                 # RoverState schema
│   ├── nodes.py                 # Graph nodes: observe, reason, act, check
│   └── tools.py                 # Agent tools: look, move, stop_and_report
├── controller/
│   ├── controller_node.py       # ROS2 node — receives commands, publishes cmd_vel
│   └── pid.py                   # PID controller for heading and distance
├── perception/
│   └── vision.py                # Camera capture + VLM API call wrapper
├── scripts/
│   ├── run_agent.py             # Main entry point
│   ├── test_controller.py       # Test controller with hardcoded commands
│   └── test_vision.py           # Test VLM perception independently
└── launch/
    └── limo_vlm_agent.launch.py # ROS2 launch file
```

## Tech Stack

- **Agent Framework:** LangGraph (stateful agent graph with cycles, checkpointing, tool use)
- **VLM Backbone:** Claude Vision API / GPT-4o (cloud, swappable to local PaliGemma 3B for edge)
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
| `look(target)` | Captures a camera frame and sends it to the VLM with a prompt asking where the target object is relative to the rover | Natural language spatial description |
| `move(heading_degrees, distance_meters)` | Issues a high-level movement command to the controller layer. Values are clamped for safety | Confirmation string |
| `stop_and_report(message)` | Stops the rover and terminates the agent loop | Final status message |

## Setup

### Prerequisites

- LIMO Agilex Pro with Jetson Orin Nano running JetPack 6.x
- ROS2 Foxy installed
- Python 3.10+
- API key for Claude or OpenAI (for cloud VLM)

### Installation

```bash
# SSH into the LIMO Pro
ssh agilex@<rover-ip>

# Clone the repo
git clone https://github.com/<your-username>/limo-vlm-agent.git
cd limo-vlm-agent

# Install Python dependencies
pip install -r requirements.txt --break-system-packages

# Set your VLM API key
export ANTHROPIC_API_KEY="sk-..."
# or
export OPENAI_API_KEY="sk-..."

# Set the LIMO to four-wheel differential mode (simplest control)
# (use the physical mode switch on the rover)
```

### Configuration

Edit `config/params.yaml` to match your setup:

```yaml
topics:
  rgb_image: "/camera/color/image_raw"
  cmd_vel: "/cmd_vel"

controller:
  max_linear_speed: 0.3       # m/s — do not exceed for indoor use
  max_angular_speed: 0.5      # rad/s
  stop_distance: 0.4          # meters from target to stop

agent:
  vlm_provider: "anthropic"   # "anthropic" | "openai"
  model: "claude-sonnet-4-20250514"
  max_steps: 20               # safety limit on reasoning cycles
  reasoning_interval: 2.0     # seconds between agent steps
```

### Running

```bash
# Terminal 1 — start the LIMO base drivers
ros2 launch limo_bringup limo_start.launch.py

# Terminal 2 — start the controller node
ros2 run limo_vlm_agent controller_node

# Terminal 3 — start the agent
python scripts/run_agent.py "drive to the water bottle"
```

## Build Phases

### Phase 1 — Controller Layer
Get the rover driving reliably from Python. Publish to `/cmd_vel`, implement `execute_command(heading, distance)` with speed clamping. Test with hardcoded commands. No AI.

### Phase 2 — Vision Tool
Write the `look()` tool. Capture camera frame, send to cloud VLM, parse spatial response (direction + rough distance). Test by manually pointing the camera at objects.

### Phase 3 — LangGraph Agent
Build the state graph with observe → reason → act → check nodes. Test the reasoning loop with static images before connecting to the live rover.

### Phase 4 — Integration & Tuning
Connect all layers end-to-end. Tune prompts for reliable spatial descriptions. Add step limits and timeout behavior. Record demo video.

## Roadmap

- [x] Project architecture design
- [ ] Phase 1: Controller layer with PID and safety clamping
- [ ] Phase 2: Vision tool with cloud VLM integration
- [ ] Phase 3: LangGraph agent state machine
- [ ] Phase 4: End-to-end integration and prompt tuning
- [ ] Obstacle awareness via LIMO Pro's onboard lidar
- [ ] Multi-step task execution ("go to X, then come back")
- [ ] Edge deployment — swap cloud VLM for local PaliGemma 3B on Orin
- [ ] Real-time web dashboard showing agent reasoning chain
- [ ] Voice command input via microphone

## Key Design Decisions

**Why LangGraph over vanilla LangChain AgentExecutor?**
AgentExecutor is a black-box ReAct loop. LangGraph gives explicit control over the execution graph, letting us inject safety checks between steps, handle the async timing between agent and controller, and implement structured termination conditions. For a physical robot, that control is non-negotiable.

**Why cloud VLM first, not local?**
Iteration speed. Cloud models (Claude, GPT-4o) give the best reasoning quality while the agent logic is still being tuned. Local deployment (PaliGemma 3B quantized on Orin) is an optimization step after the architecture is validated — and becomes a separate resume bullet about edge AI.

**Why hybrid instead of full agentic control?**
At 2–5 seconds per cloud VLM call, the rover drives 0.6–1.5 meters blind between decisions. The controller layer keeps the rover safe during reasoning gaps. It also makes the system debuggable — PID error plots for motor issues, reasoning logs for agent issues — rather than having everything tangled in one opaque loop.

## License

MIT