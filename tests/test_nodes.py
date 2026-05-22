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
