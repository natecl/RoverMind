from agent.observation_formatter import format_observation
from perception.scene_parsing import SceneObservation


def _obs(**kw):
    base = dict(
        target="water bottle", found=False, direction=None, distance=None,
        should_stop=False, raw_answers={"visible": "", "direction": "", "distance": ""},
    )
    base.update(kw)
    return SceneObservation(**base)


def test_format_not_found():
    obs = _obs(found=False)
    assert format_observation(obs) == "target not found"


def test_format_found_with_direction_and_distance():
    obs = _obs(
        found=True, direction="left", distance="medium",
        raw_answers={"visible": "Yes.", "direction": "left", "distance": "medium"},
    )
    assert format_observation(obs) == (
        "target found at left, medium distance"
    )


def test_format_found_close_includes_arrival_hint():
    obs = _obs(
        found=True, direction="center", distance="close", should_stop=True,
        raw_answers={"visible": "Yes.", "direction": "center", "distance": "close"},
    )
    assert format_observation(obs) == (
        "target found at center, close distance (arrived: call stop)"
    )


def test_format_found_unclear_direction():
    obs = _obs(
        found=True, direction=None, distance="medium",
        raw_answers={"visible": "Yes.", "direction": "?", "distance": "medium"},
    )
    assert format_observation(obs) == (
        "target found at unknown direction, medium distance"
    )


def test_format_found_unclear_distance():
    obs = _obs(
        found=True, direction="right", distance=None,
        raw_answers={"visible": "Yes.", "direction": "right", "distance": "?"},
    )
    assert format_observation(obs) == (
        "target found at right, unknown distance"
    )
