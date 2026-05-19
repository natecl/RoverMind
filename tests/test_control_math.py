import dataclasses

from safety_controller_layer.control_math import ControllerParams


def test_controller_params_defaults_match_spec():
    p = ControllerParams()
    assert p.max_linear == 0.3
    assert p.max_angular == 0.5
    assert p.heading_tolerance_rad == 0.035
    assert p.heading_kp == 1.0
    assert p.loop_rate_hz == 20.0


def test_controller_params_is_immutable():
    p = ControllerParams()
    try:
        p.max_linear = 0.9  # type: ignore[misc]
    except dataclasses.FrozenInstanceError:
        return
    raise AssertionError("ControllerParams should be frozen")
