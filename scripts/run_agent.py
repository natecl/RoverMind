"""Run the LangGraph agent on your Mac, driving the rover via the Python 3.8
bridge over an SSH-tunneled TCP socket.

Usage:
    python scripts/run_agent.py "drive to the water bottle"
    python scripts/run_agent.py --bridge tcp://localhost:9000 "<task>"

Requires (on the rover):
    - limo_bringup limo_start.launch.py
    - safety_controller_layer rovermind.launch.py
    - bridge/bridge_server.py listening on the matching port

Requires (on this Mac):
    - SSH tunnel: ssh -L 9000:localhost:9000 agilex@<rover-ip>
    - OPENAI_API_KEY exported
"""

import argparse
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from agent.config_loader import load_params  # noqa: E402
from agent.graph import build_graph  # noqa: E402
from agent.llm import build_llm  # noqa: E402
from bridge.client import BridgeClient  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="RoverMind agent (bridge mode)")
    parser.add_argument("--bridge", default="tcp://localhost:9000",
                        help="Bridge URL (default: tcp://localhost:9000)")
    parser.add_argument("task", nargs="+", help="Natural-language task")
    ns = parser.parse_args()
    task = " ".join(ns.task)

    agent_params, action_params = load_params(REPO_ROOT / "config" / "params.yaml")
    llm = build_llm(agent_params)

    with BridgeClient(ns.bridge) as bridge:
        graph = build_graph(
            llm=llm,
            execute_command=bridge.execute_command,
            capture_and_analyze=bridge.capture_and_analyze,
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
