from typing import List, Tuple

import pytest

from agent.command_executor import ExecuteResult
from agent.params import ActionParams
from agent.tools import build_tools
from perception.scene_parsing import SceneObservation


class FakeExecutor:
    """Records every (heading_deg, distance_m) call. Returns canned results."""

    def __init__(self, result: ExecuteResult = ExecuteResult(True, "ok")):
        self.calls: List[Tuple[float, float]] = []
        self._result = result

    def __call__(self, heading_deg: float, distance_m: float) -> ExecuteResult:
        self.calls.append((heading_deg, distance_m))
        return self._result


def _found_obs(direction="center", distance="medium", should_stop=False):
    return SceneObservation(
        target="water bottle",
        found=True,
        direction=direction,
        distance=distance,
        should_stop=should_stop,
        raw_answers={"visible": "Yes.", "direction": direction, "distance": distance},
    )


def _not_found_obs():
    return SceneObservation(
        target="water bottle", found=False, direction=None, distance=None,
        should_stop=False,
        raw_answers={"visible": "No.", "direction": "", "distance": ""},
    )


def _by_name(tools, name):
    for t in tools:
        if t.name == name:
            return t
    raise AssertionError(f"no tool named {name!r}")


def test_build_tools_returns_bundle_with_five_named_tools():
    bundle = build_tools(
        execute_command=FakeExecutor(),
        capture_and_analyze=lambda target: _found_obs(),
        params=ActionParams(),
    )
    names = sorted(t.name for t in bundle.tools)
    assert names == ["forward", "look", "search", "stop", "turn"]
    assert bundle.look_observation_holder == {"obs": None}


def test_look_calls_capture_and_returns_formatted_string():
    calls = []

    def fake_capture(target):
        calls.append(target)
        return _found_obs(direction="left", distance="medium")

    bundle = build_tools(FakeExecutor(), fake_capture, ActionParams())
    look = _by_name(bundle.tools, "look")

    result = look.invoke({"target": "water bottle"})

    assert calls == ["water bottle"]
    assert result == "target found at left, medium distance"


def test_look_records_observation_in_holder():
    obs = _found_obs(direction="right", distance="far")
    bundle = build_tools(FakeExecutor(), lambda t: obs, ActionParams())
    look = _by_name(bundle.tools, "look")
    look.invoke({"target": "water bottle"})
    assert bundle.look_observation_holder["obs"] == obs


def test_look_when_not_found():
    bundle = build_tools(
        FakeExecutor(), lambda target: _not_found_obs(), ActionParams(),
    )
    look = _by_name(bundle.tools, "look")
    assert look.invoke({"target": "water bottle"}) == "target not found"


def test_look_surfaces_capture_errors_as_strings_and_clears_holder():
    def bad_capture(target):
        raise RuntimeError("no camera frame")

    bundle = build_tools(FakeExecutor(), bad_capture, ActionParams())
    look = _by_name(bundle.tools, "look")
    bundle.look_observation_holder["obs"] = "stale"  # simulate prior look
    assert look.invoke({"target": "water bottle"}) == (
        "look failed: no camera frame"
    )
    # A failed look must NOT leave stale data behind.
    assert bundle.look_observation_holder["obs"] is None


def test_turn_left_small_dispatches_positive_30_zero_distance():
    fx = FakeExecutor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    turn = _by_name(bundle.tools, "turn")

    msg = turn.invoke({"direction": "left", "magnitude": "small"})

    assert fx.calls == [(30.0, 0.0)]
    assert "turn complete" in msg


def test_turn_right_large_dispatches_negative_60_zero_distance():
    fx = FakeExecutor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    turn = _by_name(bundle.tools, "turn")
    turn.invoke({"direction": "right", "magnitude": "large"})
    assert fx.calls == [(-60.0, 0.0)]


def test_forward_short_dispatches_zero_30cm():
    fx = FakeExecutor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    forward = _by_name(bundle.tools, "forward")
    forward.invoke({"distance": "short"})
    assert fx.calls == [(0.0, 0.3)]


def test_forward_medium_dispatches_zero_60cm():
    fx = FakeExecutor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    forward = _by_name(bundle.tools, "forward")
    forward.invoke({"distance": "medium"})
    assert fx.calls == [(0.0, 0.6)]


def test_search_dispatches_45_degrees_zero_distance():
    fx = FakeExecutor()
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    search = _by_name(bundle.tools, "search")
    search.invoke({})
    assert fx.calls == [(45.0, 0.0)]


def test_movement_tool_surfaces_action_failure_as_string():
    fx = FakeExecutor(result=ExecuteResult(False, "aborted: AEB"))
    bundle = build_tools(fx, lambda t: _found_obs(), ActionParams())
    forward = _by_name(bundle.tools, "forward")
    msg = forward.invoke({"distance": "short"})
    assert "move failed" in msg and "AEB" in msg


def test_stop_returns_string_acknowledgement():
    bundle = build_tools(FakeExecutor(), lambda t: _found_obs(), ActionParams())
    stop = _by_name(bundle.tools, "stop")
    msg = stop.invoke({"reason": "arrived at water bottle"})
    assert msg == "stop acknowledged: arrived at water bottle"
