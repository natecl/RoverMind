import dataclasses

import pytest

from perception.scene_parsing import (
    SceneObservation,
    build_observation,
    parse_direction,
    parse_distance,
    parse_yes_no,
)


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


@pytest.mark.parametrize("answer,expected", [
    ("Yes.", True),
    ("Yes, there is a water bottle on the table.", True),
    ("No.", False),
    ("No, I don't see one.", False),
    ("There is no bottle visible.", False),
    ("", False),
    ("I cannot tell.", False),
    ("Yes, but it is no longer clearly visible.", False),  # both -> safe default
])
def test_parse_yes_no(answer, expected):
    assert parse_yes_no(answer) is expected


@pytest.mark.parametrize("answer,expected", [
    ("It is on the left.", "left"),
    ("The bottle is in the center of the image.", "center"),
    ("It's in the middle.", "center"),
    ("Located in the centre.", "center"),
    ("To the right.", "right"),
    ("It could be on the left or the right.", None),  # ambiguous -> None
    ("", None),
    ("I am not sure where it is.", None),
])
def test_parse_direction(answer, expected):
    assert parse_direction(answer) == expected


@pytest.mark.parametrize("answer,expected", [
    ("It is close.", "close"),
    ("The bottle is near the rover.", "close"),
    ("At a medium distance.", "medium"),
    ("It is a moderate distance away.", "medium"),
    ("Far away.", "far"),
    ("It looks distant.", "far"),
    ("It is hard to tell if it is close or far.", None),  # ambiguous -> None
    ("", None),
])
def test_parse_distance(answer, expected):
    assert parse_distance(answer) == expected


def test_build_observation_found_and_close_sets_should_stop():
    obs = build_observation("water bottle", "Yes.", "On the left.", "It is close.")
    assert obs.found is True
    assert obs.direction == "left"
    assert obs.distance == "close"
    assert obs.should_stop is True
    assert obs.raw_answers["visible"] == "Yes."


def test_build_observation_found_but_far_does_not_stop():
    obs = build_observation("water bottle", "Yes.", "On the right.", "Far away.")
    assert obs.found is True
    assert obs.distance == "far"
    assert obs.should_stop is False


def test_build_observation_not_found_zeroes_fields():
    obs = build_observation("water bottle", "No.", "", "")
    assert obs.found is False
    assert obs.direction is None
    assert obs.distance is None
    assert obs.should_stop is False
