# agent/ — LangGraph reasoning loop (runs on the Mac)

The autonomous agent: turn a natural-language task into look→reason→move cycles. **Mac side,
Python 3.10, no ROS/ML imports.** Hardware is reached through injected callables (see `bridge/`).

## Key files

- `graph.py` — `build_graph(llm, execute_command, capture_and_analyze, agent_params, action_params)`
  wires `START → init → reason → act → check → (running? reason : END)`. **The entry point.**
- `nodes.py` — the four nodes + `should_continue`. `act_node` dispatches the tool the LLM picked;
  `look` stores the `SceneObservation`, `stop` sets `status="arrived"`; `check_node` enforces `max_steps`.
- `tools.py` — `build_tools(execute_command, capture_and_analyze, params)` → the five `@tool`s:
  `look · turn · forward · search · stop`. Tools call the injected callables; they don't do I/O themselves.
- `action_resolvers.py` — **pure** bucket→number resolvers (`turn_degrees`, `forward_meters`,
  `search_degrees`). Laptop-testable; put motion-magnitude logic here.
- `state.py` — `RoverState` TypedDict + `extract_target(task)` (regex "drive to X" → target).
- `command_executor.py` — `validate_command()` (pure) + `CommandExecutor` (rclpy ActionClient,
  **hardware-only**). Returns `ExecuteResult`.
- `config_loader.py` / `params.py` — load `config/params.yaml` → `AgentParams`, `ActionParams`.
- `llm.py` — `build_llm(params)` → ChatOpenAI (needs `OPENAI_API_KEY`).
- `prompts.py` — `SYSTEM_PROMPT`: the tool vocabulary + strategy. **Mac-local — editing it needs
  no rover deploy.** Prefer forward progress; turn only when the target is clearly to one side.

## How it connects

`scripts/run_agent.py` builds the LLM + a `BridgeClient`, passes the client's `execute_command`
and `capture_and_analyze` into `build_graph`, and invokes with the task. The same graph runs with
**fakes** in tests / `scripts/test_agent_static.py`. See `context/ARCHITECTURE.md`.

## Gotchas

- Direction parsing lives in `perception/scene_parsing.py`, not here — "center" includes slight
  offsets (→ drive forward). The prompt must stay aligned with that.
- Don't import rclpy/torch here. Keep new logic in the pure files (`action_resolvers`, `state`).

## Tests

`tests/test_graph_*.py`, `test_nodes.py`, `test_tools.py`, `test_action_resolvers.py`,
`test_state.py`, `test_command_executor_pure.py`, `test_config_loader.py`, `test_params.py`, `test_llm.py`.
