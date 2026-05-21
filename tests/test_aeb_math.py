import math

import pytest

from safety_controller_layer.aeb_math import AebParams


def test_aeb_params_defaults_match_spec():
    p = AebParams()
    assert p.trigger_distance_m == 0.40
    assert p.release_distance_m == 0.60
    assert p.release_dwell_s == 0.5
    assert p.forward_arc_deg == 60.0
    assert p.output_rate_hz == 20.0
    assert p.command_timeout_s == 0.5
    assert p.scan_timeout_s == 1.0


def test_aeb_params_rejects_release_not_greater_than_trigger():
    with pytest.raises(ValueError):
        AebParams(trigger_distance_m=0.5, release_distance_m=0.5)
    with pytest.raises(ValueError):
        AebParams(trigger_distance_m=0.5, release_distance_m=0.4)


from safety_controller_layer.aeb_math import gate_twist


def test_gate_twist_zeroes_forward_when_braking():
    assert gate_twist(0.3, 0.0, braking=True) == (0.0, 0.0)


def test_gate_twist_passes_rotation_when_braking():
    assert gate_twist(0.0, 0.5, braking=True) == (0.0, 0.5)


def test_gate_twist_passes_reverse_when_braking():
    assert gate_twist(-0.2, 0.0, braking=True) == (-0.2, 0.0)


def test_gate_twist_passes_everything_when_not_braking():
    assert gate_twist(0.3, 0.5, braking=False) == (0.3, 0.5)


from safety_controller_layer.aeb_math import BrakeStateMachine


def test_brake_state_machine_starts_not_braking():
    sm = BrakeStateMachine(AebParams())
    assert sm.braking is False


def test_brake_state_machine_trips_below_trigger_distance():
    sm = BrakeStateMachine(AebParams())
    assert sm.update(min_range=0.30, now=0.0) is True
    assert sm.braking is True


def test_brake_state_machine_stays_clear_above_trigger_distance():
    sm = BrakeStateMachine(AebParams())
    assert sm.update(min_range=1.0, now=0.0) is False
    assert sm.braking is False
