from langchain_core.messages import AIMessage

from agent.command_executor import ExecuteResult
from agent.graph import build_graph
from agent.params import ActionParams, AgentParams
from perception.scene_parsing import SceneObservation


def _found():
    return SceneObservation(
        target="water bottle", found=True, direction="left",
        distance="medium", should_stop=False,
        raw_answers={"visible": "Yes.", "direction": "left", "distance": "medium"},
    )


def _ai(name, args, call_id="x"):
    return AIMessage(
        content="",
        tool_calls=[{"id": call_id, "name": name, "args": args}],
    )


class ScriptedLLM:
    def __init__(self, responses):
        self._responses = list(responses)

    def invoke(self, messages):
        if not self._responses:
            # Default to "look" forever; lets the max_steps test loop.
            return _ai("look", {"target": "water bottle"}, "ck")
        return self._responses.pop(0)

    def bind_tools(self, tools, **kw):
        return self


def test_max_steps_backstop_fires_when_llm_never_stops():
    def fake_execute(h, d):
        return ExecuteResult(True, "ok")

    def fake_capture(target):
        return _found()

    graph = build_graph(
        llm=ScriptedLLM([]),  # always "look"
        execute_command=fake_execute,
        capture_and_analyze=fake_capture,
        agent_params=AgentParams(max_steps=5),
        action_params=ActionParams(),
    )

    final = graph.invoke({"task": "drive to the water bottle"})

    assert final["status"] == "failed_max_steps"
    assert final["step_count"] == 5
    assert "max steps" in final["status_message"]


def test_aborts_when_llm_raises():
    class ExplodingLLM:
        def invoke(self, messages):
            raise RuntimeError("openai api down")

        def bind_tools(self, tools, **kw):
            return self

    graph = build_graph(
        llm=ExplodingLLM(),
        execute_command=lambda h, d: ExecuteResult(True, "ok"),
        capture_and_analyze=lambda t: _found(),
        agent_params=AgentParams(max_steps=20),
        action_params=ActionParams(),
    )

    final = graph.invoke({"task": "drive to the water bottle"})

    assert final["status"] == "aborted"
    assert "openai api down" in final["status_message"]


def test_move_failure_surfaces_as_tool_message_and_loop_continues():
    fx_calls = []

    def fake_execute(h, d):
        fx_calls.append((h, d))
        # First move fails; subsequent moves succeed.
        if len(fx_calls) == 1:
            return ExecuteResult(False, "aborted: AEB triggered")
        return ExecuteResult(True, "ok")

    def fake_capture(target):
        return _found()

    llm = ScriptedLLM([
        _ai("forward", {"distance": "short"}, "c1"),
        _ai("look", {"target": "water bottle"}, "c2"),
        _ai("stop", {"reason": "good enough"}, "c3"),
    ])

    graph = build_graph(
        llm=llm,
        execute_command=fake_execute,
        capture_and_analyze=fake_capture,
        agent_params=AgentParams(max_steps=20),
        action_params=ActionParams(),
    )

    final = graph.invoke({"task": "drive to the water bottle"})

    assert final["status"] == "arrived"
    # The failed move's ToolMessage must include the AEB string so the
    # LLM (real or scripted) can see what happened.
    assert any(
        "AEB triggered" in getattr(m, "content", "")
        for m in final["messages"]
    )
