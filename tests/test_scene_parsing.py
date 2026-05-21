import dataclasses

import pytest

from perception.scene_parsing import SceneObservation


def test_scene_observation_holds_all_fields():
    obs = SceneObservation(
        target="water bottle",
        found=True,
        direction="left",
        distance="close",
        should_stop=True,
        raw_answers={"visible": "Yes."},
    )
    assert obs.target == "water bottle"
    assert obs.found is True
    assert obs.direction == "left"
    assert obs.distance == "close"
    assert obs.should_stop is True
    assert obs.raw_answers == {"visible": "Yes."}


def test_scene_observation_is_frozen():
    obs = SceneObservation(
        target="bottle", found=False, direction=None, distance=None,
        should_stop=False, raw_answers={},
    )
    with pytest.raises(dataclasses.FrozenInstanceError):
        obs.found = True
