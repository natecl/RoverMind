import pytest

from agent.action_resolvers import (
    forward_meters,
    search_degrees,
    turn_degrees,
)
from agent.params import ActionParams


def test_turn_left_small_returns_positive_30():
    # CCW positive convention matches ExecuteCommand.action; left = CCW.
    assert turn_degrees("left", "small", ActionParams()) == 30.0


def test_turn_right_small_returns_negative_30():
    assert turn_degrees("right", "small", ActionParams()) == -30.0


def test_turn_left_large_returns_positive_60():
    assert turn_degrees("left", "large", ActionParams()) == 60.0


def test_turn_right_large_returns_negative_60():
    assert turn_degrees("right", "large", ActionParams()) == -60.0


def test_turn_rejects_unknown_direction():
    with pytest.raises(ValueError, match="direction"):
        turn_degrees("up", "small", ActionParams())


def test_turn_rejects_unknown_magnitude():
    with pytest.raises(ValueError, match="magnitude"):
        turn_degrees("left", "tiny", ActionParams())


def test_forward_short_returns_0_3_m():
    assert forward_meters("short", ActionParams()) == 0.3


def test_forward_medium_returns_0_6_m():
    assert forward_meters("medium", ActionParams()) == 0.6


def test_forward_rejects_unknown_distance():
    with pytest.raises(ValueError, match="distance"):
        forward_meters("huge", ActionParams())


def test_search_returns_configured_degrees():
    assert search_degrees(ActionParams()) == 45.0
    assert search_degrees(ActionParams(search_deg=60.0)) == 60.0
