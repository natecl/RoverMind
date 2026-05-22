"""Run the LangGraph agent on the live rover.

Usage:
    python scripts/run_agent.py "drive to the water bottle"

Requires:
    - safety_controller_node running (provides /execute_command action).
    - LIMO base drivers running (so /cmd_vel actually moves the rover).
    - emergency-braking gate running.
    - OPENAI_API_KEY exported.
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from agent.command_executor import execute_command  # noqa: E402
from agent.config_loader import load_params  # noqa: E402
from agent.graph import build_graph  # noqa: E402
from agent.llm import build_llm  # noqa: E402
from perception.moondream_client import MoondreamClient  # noqa: E402
from perception.vision_tool import (  # noqa: E402
    capture_and_analyze as _capture_and_analyze,
    ros_capture_fn,
    ros_depth_capture_fn,
)


def main() -> int:
    if len(sys.argv) < 2:
        print("usage: run_agent.py <natural-language task>", file=sys.stderr)
        return 2
    task = " ".join(sys.argv[1:])

    agent_params, action_params = load_params(REPO_ROOT / "config" / "params.yaml")
    llm = build_llm(agent_params)

    moondream = MoondreamClient()

    def capture_and_analyze(target: str):
        # The vision tool's signature is capture_and_analyze(target, *, capture_fn,
        # moondream, depth_fn). Bind hardware-specific args here.
        return _capture_and_analyze(
            target,
            capture_fn=ros_capture_fn,
            moondream=moondream,
            depth_fn=ros_depth_capture_fn,
        )

    graph = build_graph(
        llm=llm,
        execute_command=execute_command,
        capture_and_analyze=capture_and_analyze,
        agent_params=agent_params,
        action_params=action_params,
    )

    final = graph.invoke({"task": task})

    print(f"\n=== Run complete ===")
    print(f"status:         {final['status']}")
    print(f"status_message: {final['status_message']}")
    print(f"steps:          {final['step_count']}")
    print(f"target:         {final['target']}")
    return 0 if final["status"] == "arrived" else 1


if __name__ == "__main__":
    raise SystemExit(main())
