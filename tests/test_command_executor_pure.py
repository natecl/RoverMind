import pytest

from agent.command_executor import ExecuteResult, validate_command


def test_execute_result_holds_outcome():
    r = ExecuteResult(success=True, message="ok")
    assert r.success is True
    assert r.message == "ok"


def test_validate_command_accepts_zero_distance():
    # turning in place is a valid command (used by search and turn tools)
    validate_command(heading_deg=45.0, distance_m=0.0)


def test_validate_command_accepts_typical_move():
    validate_command(heading_deg=-30.0, distance_m=0.6)


def test_validate_command_rejects_negative_distance():
    with pytest.raises(ValueError, match="distance_m must be non-negative"):
        validate_command(heading_deg=0.0, distance_m=-0.1)


def test_validate_command_rejects_nan_or_inf():
    with pytest.raises(ValueError):
        validate_command(heading_deg=float("nan"), distance_m=0.0)
    with pytest.raises(ValueError):
        validate_command(heading_deg=0.0, distance_m=float("inf"))
