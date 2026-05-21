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
