from langchain_core.messages import HumanMessage, SystemMessage

from agent.nodes import init_node


def test_init_extracts_target_and_seeds_state():
    initial = {"task": "drive to the water bottle"}
    out = init_node(initial)

    assert out["target"] == "water bottle"
    assert out["step_count"] == 0
    assert out["status"] == "running"
    assert out["status_message"] == ""
    assert out["last_observation"] is None
    # Two messages: system prompt + user task.
    msgs = out["messages"]
    assert len(msgs) == 2
    assert isinstance(msgs[0], SystemMessage)
    assert "five tools" in msgs[0].content.lower()
    assert isinstance(msgs[1], HumanMessage)
    assert msgs[1].content == "drive to the water bottle"


def test_init_preserves_original_task_string():
    out = init_node({"task": "find the laptop please"})
    assert out["task"] == "find the laptop please"
    assert out["target"] == "laptop please"


from typing import List

from langchain_core.messages import AIMessage, ToolMessage

from agent.command_executor import ExecuteResult
from agent.nodes import act_node
from agent.params import ActionParams
from agent.tools import build_tools
from perception.scene_parsing import SceneObservation


def _fake_executor():
    calls: List[tuple] = []

    def fn(h, d):
        calls.append((h, d))
        return ExecuteResult(True, "ok")

    fn.calls = calls  # type: ignore[attr-defined]
    return fn


def _found_obs():
    return SceneObservation(
        target="water bottle", found=True, direction="center",
        distance="medium", should_stop=False,
        raw_answers={"visible": "Yes.", "direction": "center", "distance": "medium"},
    )


def _ai_with_tool_call(name, args):
    return AIMessage(
        content="",
        tool_calls=[{"id": "call_1", "name": name, "args": args}],
    )


def test_act_dispatches_turn_and_returns_tool_message():
    fx = _fake_executor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    state = {
        "messages": [_ai_with_tool_call("turn", {"direction": "left", "magnitude": "small"})],
        "status": "running",
        "last_observation": None,
        "status_message": "",
    }

    out = act_node(state, tool_bundle=bundle)

    assert fx.calls == [(30.0, 0.0)]  # type: ignore[attr-defined]
    msgs = out["messages"]
    assert len(msgs) == 1
    assert isinstance(msgs[0], ToolMessage)
    assert msgs[0].tool_call_id == "call_1"
    assert "turn complete" in msgs[0].content
    assert out["status"] == "running"


def test_act_dispatches_look_and_records_last_observation():
    obs = _found_obs()
    fx = _fake_executor()
    bundle = build_tools(fx, lambda t: obs, ActionParams())
    state = {
        "messages": [_ai_with_tool_call("look", {"target": "water bottle"})],
        "status": "running",
        "last_observation": None,
        "status_message": "",
    }

    out = act_node(state, tool_bundle=bundle)

    assert out["last_observation"] == obs
    assert isinstance(out["messages"][0], ToolMessage)
    assert "target found" in out["messages"][0].content


def test_act_on_stop_sets_terminal_status():
    fx = _fake_executor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    state = {
        "messages": [_ai_with_tool_call("stop", {"reason": "arrived"})],
        "status": "running",
        "last_observation": None,
        "status_message": "",
    }

    out = act_node(state, tool_bundle=bundle)

    assert out["status"] == "arrived"
    assert out["status_message"] == "arrived"
    assert isinstance(out["messages"][0], ToolMessage)
    assert "stop acknowledged" in out["messages"][0].content


def test_act_aborts_when_latest_message_has_no_tool_call():
    fx = _fake_executor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    state = {
        "messages": [AIMessage(content="hello")],
        "status": "running",
        "last_observation": None,
        "status_message": "",
    }

    out = act_node(state, tool_bundle=bundle)

    assert out["status"] == "aborted"
    assert "no tool call" in out["status_message"]
