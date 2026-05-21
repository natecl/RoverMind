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


def test_brake_holds_in_dead_band():
    sm = BrakeStateMachine(AebParams())
    sm.update(min_range=0.30, now=0.0)                   # trip
    assert sm.update(min_range=0.50, now=1.0) is True    # dead band -> hold


def test_brake_releases_after_dwell_past_release_distance():
    sm = BrakeStateMachine(AebParams())                  # dwell = 0.5 s
    sm.update(min_range=0.30, now=0.0)                   # trip
    assert sm.update(min_range=0.70, now=1.0) is True    # clear, dwell starts
    assert sm.update(min_range=0.70, now=1.4) is True    # 0.4 s < dwell -> hold
    assert sm.update(min_range=0.70, now=1.5) is False   # 0.5 s >= dwell -> release


def test_brake_does_not_release_before_dwell_elapses():
    sm = BrakeStateMachine(AebParams())
    sm.update(min_range=0.30, now=0.0)                   # trip
    sm.update(min_range=0.70, now=1.0)                   # clear, dwell starts
    assert sm.update(min_range=0.70, now=1.49) is True   # still within dwell


def test_brake_dwell_resets_when_obstacle_reenters_dead_band():
    sm = BrakeStateMachine(AebParams())                  # dwell = 0.5 s
    sm.update(min_range=0.30, now=0.0)                   # trip
    sm.update(min_range=0.70, now=1.0)                   # clear past release
    sm.update(min_range=0.50, now=1.2)                   # back into dead band
    assert sm.update(min_range=0.70, now=1.6) is True    # dwell restarts at 1.6
    assert sm.update(min_range=0.70, now=2.0) is True    # 0.4 s into restart
    assert sm.update(min_range=0.70, now=2.1) is False   # 0.5 s -> release


def test_brake_does_not_chatter_at_trigger_threshold():
    sm = BrakeStateMachine(AebParams())
    sm.update(min_range=0.30, now=0.0)                   # trip
    for t in range(1, 20):
        assert sm.update(min_range=0.41, now=float(t)) is True


def test_phase1_brake_and_release_scenario():
    """End-to-end Phase 1 slice: a sequence of obstacle distances drives the
    state machine, and the resulting brake flag gates a steady 0.3 m/s forward
    command."""
    params = AebParams()
    sm = BrakeStateMachine(params)
    forward_cmd = (0.3, 0.0)  # linear_x, angular_z

    # Far away -> command passes untouched.
    braking = sm.update(min_range=2.0, now=0.0)
    assert gate_twist(*forward_cmd, braking=braking) == (0.3, 0.0)

    # Obstacle inside trigger -> forward zeroed.
    braking = sm.update(min_range=0.35, now=0.1)
    assert gate_twist(*forward_cmd, braking=braking) == (0.0, 0.0)

    # While braked, a rotate command still passes.
    assert gate_twist(0.0, 0.5, braking=braking) == (0.0, 0.5)

    # Back away past release distance; brake holds until dwell elapses.
    braking = sm.update(min_range=0.80, now=0.2)
    assert gate_twist(*forward_cmd, braking=braking) == (0.0, 0.0)

    # After the dwell, brake releases and forward motion resumes.
    braking = sm.update(min_range=0.80, now=0.7)
    assert gate_twist(*forward_cmd, braking=braking) == (0.3, 0.0)


from safety_controller_layer.aeb_math import min_forward_range


def test_min_forward_range_finds_forward_obstacle():
    # 5 beams at -90, -45, 0, +45, +90 deg; only the 0 deg beam is in the arc
    ranges = [5.0, 5.0, 0.8, 5.0, 5.0]
    result = min_forward_range(
        ranges, angle_min=-math.pi / 2, angle_increment=math.pi / 4,
        range_min=0.05, arc_half_width_rad=math.radians(30),
    )
    assert result == 0.8


def test_min_forward_range_ignores_obstacle_outside_arc():
    # close obstacle at +45 deg, outside the +/-30 deg arc
    ranges = [5.0, 5.0, 5.0, 0.3, 5.0]
    result = min_forward_range(
        ranges, angle_min=-math.pi / 2, angle_increment=math.pi / 4,
        range_min=0.05, arc_half_width_rad=math.radians(30),
    )
    assert result == 5.0


def test_min_forward_range_filters_invalid_readings():
    # in-arc beams at -20, 0, +20 deg read inf, 0.0, 1.2 -> only 1.2 is valid
    ranges = [math.nan, math.inf, 0.0, 1.2, 5.0]
    result = min_forward_range(
        ranges, angle_min=-math.radians(40), angle_increment=math.radians(20),
        range_min=0.05, arc_half_width_rad=math.radians(30),
    )
    assert result == 1.2


def test_min_forward_range_filters_sub_range_min_readings():
    # 0.03 is below range_min (0.05) -> discarded; 2.0 is the answer
    ranges = [2.0, 0.03]
    result = min_forward_range(
        ranges, angle_min=-math.radians(10), angle_increment=math.radians(20),
        range_min=0.05, arc_half_width_rad=math.radians(30),
    )
    assert result == 2.0


def test_min_forward_range_returns_inf_when_arc_empty():
    # both in-arc beams invalid -> inf
    ranges = [math.nan, math.inf]
    result = min_forward_range(
        ranges, angle_min=-math.radians(10), angle_increment=math.radians(20),
        range_min=0.05, arc_half_width_rad=math.radians(30),
    )
    assert result == math.inf


def test_min_forward_range_handles_arc_straddling_zero():
    # 8 beams 45 deg apart starting at 0 rad; forward arc spans beam 0 and beam 7
    ranges = [0.9, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 0.7]
    result = min_forward_range(
        ranges, angle_min=0.0, angle_increment=math.radians(45),
        range_min=0.05, arc_half_width_rad=math.radians(50),
    )
    assert result == 0.7


def test_phase2_full_pipeline_brakes_on_lidar_obstacle():
    """End-to-end Phase 2 slice: a raw scan's ranges feed min_forward_range,
    whose output drives the state machine and gates the twist."""
    params = AebParams()
    sm = BrakeStateMachine(params)
    arc_half = math.radians(params.forward_arc_deg / 2.0)
    forward_cmd = (0.3, 0.0)

    # Scan with a clear forward arc (3 beams at -20, 0, +20 deg) -> command passes.
    clear_ranges = [3.0, 3.0, 3.0]
    min_range = min_forward_range(
        clear_ranges, angle_min=-math.radians(20),
        angle_increment=math.radians(20), range_min=0.05,
        arc_half_width_rad=arc_half,
    )
    braking = sm.update(min_range, now=0.0)
    assert gate_twist(*forward_cmd, braking=braking) == (0.3, 0.0)

    # Scan with an obstacle 0.25 m dead ahead -> forward zeroed.
    blocked_ranges = [3.0, 0.25, 3.0]
    min_range = min_forward_range(
        blocked_ranges, angle_min=-math.radians(20),
        angle_increment=math.radians(20), range_min=0.05,
        arc_half_width_rad=arc_half,
    )
    braking = sm.update(min_range, now=0.1)
    assert gate_twist(*forward_cmd, braking=braking) == (0.0, 0.0)
