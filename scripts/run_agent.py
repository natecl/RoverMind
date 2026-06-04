"""Run the LangGraph agent on your Mac, driving the rover via the Python 3.8
bridge over an SSH-tunneled TCP socket.

Usage:
    python scripts/run_agent.py "drive to the water bottle"
    python scripts/run_agent.py --bridge tcp://localhost:9000 "<task>"
    python scripts/run_agent.py --benchmark-json run.json "<task>"
    python scripts/run_agent.py --no-report "<task>"

Requires (on the rover):
    - limo_bringup limo_start.launch.py
    - safety_controller_layer rovermind.launch.py
    - bridge/bridge_server.py listening on the matching port

Requires (on this Mac):
    - SSH tunnel: ssh -L 9000:localhost:9000 agilex@<rover-ip>
    - OPENAI_API_KEY exported
"""

import argparse
import json
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

# Load OPENAI_API_KEY (and any other secrets) from the gitignored .env, if present.
# An already-exported env var still wins (override=False).
from dotenv import load_dotenv  # noqa: E402

load_dotenv(REPO_ROOT / ".env")

from agent.config_loader import load_params  # noqa: E402
from agent.graph import build_graph  # noqa: E402
from agent.latency import LatencyCollector, TimingLLM, summarize, timed  # noqa: E402
from agent.llm import build_llm  # noqa: E402
from bridge.client import BridgeClient  # noqa: E402
from bridge.errors import BridgeUnreachable  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="RoverMind agent (bridge mode)")
    parser.add_argument("--bridge", default="tcp://localhost:9000",
                        help="Bridge URL (default: tcp://localhost:9000)")
    parser.add_argument("--benchmark-json", default=None, metavar="PATH",
                        help="Write the full latency report (events + aggregates) to PATH as JSON")
    parser.add_argument("--no-report", action="store_true",
                        help="Suppress the end-of-run latency console report")
    parser.add_argument("task", nargs="+", help="Natural-language task")
    ns = parser.parse_args()
    task = " ".join(ns.task)

    agent_params, action_params = load_params(REPO_ROOT / "config" / "params.yaml")
    llm = build_llm(agent_params)

    # Latency benchmarking: collection is always on (perf_counter is ~free).
    # The LLM is wrapped to time `reason`; the two bridge callables are wrapped
    # to time their round-trips; the BridgeClient forwards the rover's per-RPC
    # decomposition (server/action/VLM/transport) into the same collector.
    collector = LatencyCollector()

    try:
        try:
            with BridgeClient(ns.bridge, timing_sink=collector.record_metric) as bridge:
                graph = build_graph(
                    llm=TimingLLM(llm, collector),
                    execute_command=timed(bridge.execute_command, collector, "move",
                                          label_fn=lambda h, d: f"h={h:.0f},d={d:.2f}"),
                    capture_and_analyze=timed(bridge.capture_and_analyze, collector, "look",
                                              label_fn=lambda target: str(target)),
                    agent_params=agent_params,
                    action_params=action_params,
                )
                with collector.span("run", "graph"):
                    final = graph.invoke({"task": task})
        except BridgeUnreachable as exc:
            print(f"error: could not reach the bridge at {ns.bridge}: {exc}", file=sys.stderr)
            print("hint: did you run `ssh -L 9000:localhost:9000 agilex@<rover-ip>` "
                  "and start `python3.8 bridge/bridge_server.py` on the rover?",
                  file=sys.stderr)
            return 2
    finally:
        # Emit the latency report even if the run crashed mid-way — partial
        # benchmark data is often the most interesting. Skip it when nothing
        # was collected (e.g. the bridge was never reached).
        if collector.events or collector.metrics:
            _emit_report(summarize(collector), ns)

    print(f"\n=== Run complete ===")
    print(f"status:         {final['status']}")
    print(f"status_message: {final['status_message']}")
    print(f"steps:          {final['step_count']}")
    print(f"target:         {final['target']}")
    return 0 if final["status"] == "arrived" else 1


def _emit_report(report, ns) -> None:
    if not ns.no_report:
        print()
        print(report.render())
    if ns.benchmark_json:
        Path(ns.benchmark_json).write_text(json.dumps(report.to_dict(), indent=2))
        print(f"\nlatency report written to {ns.benchmark_json}")


if __name__ == "__main__":
    raise SystemExit(main())
