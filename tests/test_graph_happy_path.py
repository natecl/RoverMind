from langchain_core.messages import AIMessage, HumanMessage, ToolMessage

from agent.command_executor import ExecuteResult
from agent.graph import build_graph
from agent.params import ActionParams, AgentParams
from perception.scene_parsing import SceneObservation


def _found(direction="center", distance="medium", should_stop=False):
    return SceneObservation(
        target="water bottle", found=True, direction=direction,
        distance=distance, should_stop=should_stop,
        raw_answers={"visible": "Yes.", "direction": direction, "distance": distance},
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
        return self._responses.pop(0)

    def bind_tools(self, tools, **kw):
        # graph.py uses llm.bind_tools(...) for the real LLM; scripted
        # LLM ignores tool binding because it returns prebaked tool calls.
        return self


def test_happy_path_look_turn_look_forward_look_stop():
    fx_calls = []

    def fake_execute(h, d):
        fx_calls.append((h, d))
        return ExecuteResult(True, "ok")

    observations = [
        _found(direction="left", distance="medium"),   # 1st look
        _found(direction="center", distance="medium"), # 2nd look (after turn)
        _found(direction="center", distance="close",   # 3rd look (after forward)
               should_stop=True),
    ]

    def fake_capture(target):
        return observations.pop(0)

    llm = ScriptedLLM([
        _ai("look", {"target": "water bottle"}, "c1"),
        _ai("turn", {"direction": "left", "magnitude": "small"}, "c2"),
        _ai("look", {"target": "water bottle"}, "c3"),
        _ai("forward", {"distance": "medium"}, "c4"),
        _ai("look", {"target": "water bottle"}, "c5"),
        _ai("stop", {"reason": "arrived at the water bottle"}, "c6"),
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
    assert final["status_message"] == "arrived at the water bottle"
    assert final["target"] == "water bottle"
    assert fx_calls == [(30.0, 0.0), (0.0, 0.6)]
    # Three looks → three observations recorded; last one is should_stop.
    assert final["last_observation"].should_stop is True

    # Sanity-check the messages contain six AIMessages and six ToolMessages
    # plus the system + user (8 + system + human = 14 total).
    kinds = [type(m).__name__ for m in final["messages"]]
    assert kinds.count("AIMessage") == 6
    assert kinds.count("ToolMessage") == 6
