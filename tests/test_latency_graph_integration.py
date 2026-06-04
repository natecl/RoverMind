"""Integration test: the latency collector observing a real graph run.

Mirrors tests/test_graph_happy_path.py but wraps the LLM in TimingLLM and the
two bridge callables in timed(...), then asserts the collector saw exactly the
expected reason/look/move events. Uses the real perf_counter clock — durations
are tiny but monotonic, so only counts and segmentation are asserted.
"""

from langchain_core.messages import AIMessage

from agent.command_executor import ExecuteResult
from agent.graph import build_graph
from agent.latency import LatencyCollector, TimingLLM, summarize, timed
from agent.params import ActionParams, AgentParams
from perception.scene_parsing import SceneObservation


def _found(direction="center", distance="medium", should_stop=False):
    return SceneObservation(
        target="water bottle", found=True, direction=direction,
        distance=distance, should_stop=should_stop,
        raw_answers={"visible": "Yes.", "direction": direction, "distance": distance},
    )


def _ai(name, args, call_id="x"):
    return AIMessage(content="", tool_calls=[{"id": call_id, "name": name, "args": args}])


class ScriptedLLM:
    def __init__(self, responses):
        self._responses = list(responses)

    def invoke(self, messages):
        return self._responses.pop(0)

    def bind_tools(self, tools, **kw):
        return self


def test_graph_run_records_reason_look_move_events():
    c = LatencyCollector()
    fx_calls = []

    def fake_execute(h, d):
        fx_calls.append((h, d))
        return ExecuteResult(True, "ok")

    observations = [
        _found(direction="left", distance="medium"),
        _found(direction="center", distance="medium"),
        _found(direction="center", distance="close", should_stop=True),
    ]

    def fake_capture(target):
        return observations.pop(0)

    llm = TimingLLM(ScriptedLLM([
        _ai("look", {"target": "water bottle"}, "c1"),
        _ai("turn", {"direction": "left", "magnitude": "small"}, "c2"),
        _ai("look", {"target": "water bottle"}, "c3"),
        _ai("forward", {"distance": "medium"}, "c4"),
        _ai("look", {"target": "water bottle"}, "c5"),
        _ai("stop", {"reason": "arrived"}, "c6"),
    ]), c)

    graph = build_graph(
        llm=llm,
        execute_command=timed(fake_execute, c, "move",
                              label_fn=lambda h, d: f"h={h:.0f},d={d:.2f}"),
        capture_and_analyze=timed(fake_capture, c, "look",
                                  label_fn=lambda target: str(target)),
        agent_params=AgentParams(max_steps=20),
        action_params=ActionParams(),
    )

    final = graph.invoke({"task": "drive to the water bottle"})

    assert final["status"] == "arrived"
    assert final["step_count"] == 6

    cats = [e.category for e in c.events]
    assert cats.count("reason") == 6
    assert cats.count("look") == 3
    assert cats.count("move") == 2   # turn + forward; stop does no I/O

    d = summarize(c).to_dict()
    assert len(d["steps"]) == 6
    # The stop step (last) has a reason span but no bridge call.
    assert d["steps"][-1]["bridge"] is None
