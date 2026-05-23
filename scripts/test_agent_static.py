"""Static-image rehearsal for the LangGraph agent.

Replaces ros_capture_fn with a sequence of PIL images loaded from disk so
the agent can be exercised end-to-end on a laptop. Real Moondream2, real
OpenAI; fake execute_command that just logs the move it would have made.

Usage:
    python scripts/test_agent_static.py <task> <frame1> [<frame2> ...]

Each `look` call consumes the next frame in order; once exhausted, the
last frame repeats.
"""

import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from PIL import Image  # noqa: E402

from agent.command_executor import ExecuteResult  # noqa: E402
from agent.config_loader import load_params  # noqa: E402
from agent.graph import build_graph  # noqa: E402
from agent.llm import build_llm  # noqa: E402
from perception.moondream_client import MoondreamClient  # noqa: E402
from perception.vision_tool import capture_and_analyze as _capture_and_analyze  # noqa: E402


def main() -> int:
    if len(sys.argv) < 2:
        print(
            "usage: test_agent_static.py <task> <frame1> [<frame2> ...]",
            file=sys.stderr,
        )
        return 2
    task = sys.argv[1]
    frame_paths = [Path(p) for p in sys.argv[2:]]
    frames = [Image.open(p).convert("RGB") for p in frame_paths]
    if not frames:
        print("at least one frame is required", file=sys.stderr)
        return 2

    cursor = {"i": 0}

    def stub_capture_fn():
        i = min(cursor["i"], len(frames) - 1)
        cursor["i"] += 1
        return frames[i]

    moondream = MoondreamClient()

    def capture_and_analyze(target: str):
        # depth_fn is deliberately omitted: the rehearsal frames are flat JPGs
        # with no aligned depth stream, so the perception layer falls back to
        # the Moondream2-only distance bucket. The live-rover script
        # (scripts/run_agent.py) passes depth_fn=ros_depth_capture_fn so the
        # depth camera is used in production.
        return _capture_and_analyze(
            target, capture_fn=stub_capture_fn, moondream=moondream,
        )

    def fake_execute(heading_deg: float, distance_m: float) -> ExecuteResult:
        print(f"  [fake move] heading={heading_deg:+.1f}deg, distance={distance_m:.2f}m")
        return ExecuteResult(True, "ok")

    agent_params, action_params = load_params(REPO_ROOT / "config" / "params.yaml")
    llm = build_llm(agent_params)

    graph = build_graph(
        llm=llm,
        execute_command=fake_execute,
        capture_and_analyze=capture_and_analyze,
        agent_params=agent_params,
        action_params=action_params,
    )

    final = graph.invoke({"task": task})

    print(f"\n=== Rehearsal complete ===")
    print(f"status:         {final['status']}")
    print(f"status_message: {final['status_message']}")
    print(f"steps:          {final['step_count']}")
    return 0 if final["status"] == "arrived" else 1


if __name__ == "__main__":
    raise SystemExit(main())
