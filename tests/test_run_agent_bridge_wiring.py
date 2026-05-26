"""Verify run_agent.py wires BridgeClient into build_graph and never touches
rclpy/torch on the Mac path."""

import sys
import types
from unittest.mock import MagicMock, patch

import pytest


def _import_run_agent():
    # Re-import fresh so each test sees a clean module state.
    sys.modules.pop("scripts.run_agent", None)
    import scripts.run_agent as mod
    return mod


def test_main_constructs_bridge_client_and_wires_callables(monkeypatch):
    fake_client = MagicMock(name="BridgeClient_instance")
    fake_client.__enter__.return_value = fake_client
    fake_client.__exit__.return_value = False
    fake_client.execute_command = MagicMock(name="execute_command")
    fake_client.capture_and_analyze = MagicMock(name="capture_and_analyze")
    bridge_client_cls = MagicMock(return_value=fake_client)

    fake_llm = MagicMock(name="llm")
    fake_graph = MagicMock(name="graph")
    fake_graph.invoke.return_value = {
        "status": "arrived", "status_message": "ok", "step_count": 1, "target": "x",
    }
    build_graph = MagicMock(return_value=fake_graph)
    build_llm = MagicMock(return_value=fake_llm)
    load_params = MagicMock(return_value=(MagicMock(), MagicMock()))

    mod = _import_run_agent()
    monkeypatch.setattr(mod, "BridgeClient", bridge_client_cls, raising=False)
    monkeypatch.setattr(mod, "build_graph", build_graph)
    monkeypatch.setattr(mod, "build_llm", build_llm)
    monkeypatch.setattr(mod, "load_params", load_params)
    monkeypatch.setattr(sys, "argv",
                        ["run_agent.py", "--bridge", "tcp://localhost:9000", "drive somewhere"])

    rc = mod.main()

    assert rc == 0
    bridge_client_cls.assert_called_once_with("tcp://localhost:9000")
    # The two callables passed to build_graph must be the BridgeClient's methods.
    kwargs = build_graph.call_args.kwargs
    assert kwargs["execute_command"] is fake_client.execute_command
    assert kwargs["capture_and_analyze"] is fake_client.capture_and_analyze


def test_run_agent_does_not_import_rclpy_or_torch():
    # If run_agent.py still imports the heavy stuff on the Mac path, this fails.
    sys.modules.pop("scripts.run_agent", None)
    sys.modules.pop("rclpy", None)
    sys.modules.pop("torch", None)
    # Inject sentinels so an accidental import raises immediately.
    fake_rclpy = types.ModuleType("rclpy"); fake_rclpy.__getattr__ = lambda name: (_ for _ in ()).throw(AssertionError(f"run_agent imported rclpy.{name}"))
    fake_torch = types.ModuleType("torch"); fake_torch.__getattr__ = lambda name: (_ for _ in ()).throw(AssertionError(f"run_agent imported torch.{name}"))
    sys.modules["rclpy"] = fake_rclpy
    sys.modules["torch"] = fake_torch
    try:
        import scripts.run_agent  # noqa: F401
    finally:
        sys.modules.pop("rclpy", None)
        sys.modules.pop("torch", None)


def test_bridge_unreachable_returns_friendly_error(monkeypatch, capsys):
    """If BridgeClient.__enter__ raises BridgeUnreachable, main() should print
    a clear message to stderr and return 2 (not a raw traceback)."""
    from bridge.errors import BridgeUnreachable

    mod = _import_run_agent()

    def fail_to_connect(self):
        raise BridgeUnreachable("connection refused (port 9000)")

    fake_bridge_client = MagicMock(name="BridgeClient_instance")
    fake_bridge_client.__enter__ = fail_to_connect
    bridge_client_cls = MagicMock(return_value=fake_bridge_client)

    monkeypatch.setattr(mod, "BridgeClient", bridge_client_cls)
    monkeypatch.setattr(mod, "build_graph", MagicMock())
    monkeypatch.setattr(mod, "build_llm", MagicMock())
    monkeypatch.setattr(mod, "load_params",
                        MagicMock(return_value=(MagicMock(), MagicMock())))
    monkeypatch.setattr(sys, "argv",
                        ["run_agent.py", "--bridge", "tcp://localhost:9000", "drive"])

    rc = mod.main()

    assert rc == 2
    err = capsys.readouterr().err
    assert "could not reach the bridge" in err
    assert "tcp://localhost:9000" in err
    assert "ssh -L" in err  # the hint should mention the SSH tunnel
