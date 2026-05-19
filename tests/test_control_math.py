import dataclasses
import math

from safety_controller_layer.control_math import ControllerParams, proportional_turn, wrap_angle


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


def test_wrap_angle_within_range_unchanged():
    assert wrap_angle(0.0) == 0.0
    assert wrap_angle(1.0) == 1.0
    assert wrap_angle(-1.0) == -1.0


def test_wrap_angle_handles_full_rotation():
    assert math.isclose(wrap_angle(2 * math.pi), 0.0, abs_tol=1e-9)
    assert math.isclose(wrap_angle(-2 * math.pi), 0.0, abs_tol=1e-9)


def test_wrap_angle_picks_short_path_past_pi():
    # 3*pi/2 (270 deg) should wrap to -pi/2 (-90 deg) — go the short way
    assert math.isclose(wrap_angle(3 * math.pi / 2), -math.pi / 2, abs_tol=1e-9)


def test_proportional_turn_sign_matches_error():
    p = ControllerParams()
    assert proportional_turn(error_rad=0.5, params=p) > 0
    assert proportional_turn(error_rad=-0.5, params=p) < 0


def test_proportional_turn_clamps_to_max_angular():
    p = ControllerParams(max_angular=0.5, heading_kp=1.0)
    assert proportional_turn(error_rad=10.0, params=p) == 0.5
    assert proportional_turn(error_rad=-10.0, params=p) == -0.5


def test_proportional_turn_unclamped_in_linear_region():
    p = ControllerParams(max_angular=0.5, heading_kp=1.0)
    assert math.isclose(proportional_turn(error_rad=0.2, params=p), 0.2)


def test_proportional_turn_respects_higher_kp():
    p = ControllerParams(max_angular=10.0, heading_kp=2.0)
    assert math.isclose(proportional_turn(error_rad=0.5, params=p), 1.0)


from safety_controller_layer.control_math import displacement


def test_displacement_zero_for_same_point():
    assert displacement((0.0, 0.0), (0.0, 0.0)) == 0.0


def test_displacement_pure_x():
    assert math.isclose(displacement((0.0, 0.0), (1.5, 0.0)), 1.5)


def test_displacement_diagonal_uses_euclidean():
    assert math.isclose(displacement((0.0, 0.0), (3.0, 4.0)), 5.0)


def test_displacement_is_symmetric():
    assert math.isclose(
        displacement((1.0, 2.0), (4.0, 6.0)),
        displacement((4.0, 6.0), (1.0, 2.0)),
    )
