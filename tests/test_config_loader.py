from pathlib import Path

import pytest

from agent.config_loader import load_params
from agent.params import ActionParams, AgentParams


def _write(tmp_path: Path, content: str) -> Path:
    path = tmp_path / "params.yaml"
    path.write_text(content)
    return path


def test_loads_default_repo_yaml(tmp_path):
    yaml_text = (
        "agent:\n"
        "  llm_provider: openai\n"
        "  llm_model: gpt-4o-mini\n"
        "  llm_temperature: 0.0\n"
        "  max_steps: 20\n"
        "actions:\n"
        "  turn_small_deg: 30.0\n"
        "  turn_large_deg: 60.0\n"
        "  search_deg: 45.0\n"
        "  forward_short_m: 0.3\n"
        "  forward_medium_m: 0.6\n"
    )
    agent_p, action_p = load_params(_write(tmp_path, yaml_text))
    assert agent_p == AgentParams()
    assert action_p == ActionParams()


def test_overrides_propagate(tmp_path):
    yaml_text = (
        "agent:\n"
        "  llm_model: gpt-4o\n"
        "  max_steps: 5\n"
        "actions:\n"
        "  turn_small_deg: 20.0\n"
    )
    agent_p, action_p = load_params(_write(tmp_path, yaml_text))
    assert agent_p.llm_model == "gpt-4o"
    assert agent_p.max_steps == 5
    # Unspecified fields keep defaults.
    assert agent_p.llm_provider == "openai"
    assert action_p.turn_small_deg == 20.0
    assert action_p.turn_large_deg == 60.0


def test_missing_file_raises_filenotfound(tmp_path):
    with pytest.raises(FileNotFoundError):
        load_params(tmp_path / "no_such.yaml")
