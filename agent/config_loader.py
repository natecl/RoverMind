"""Load AgentParams + ActionParams from a YAML file.

Unspecified keys fall back to the dataclass defaults, so the YAML can
contain only the values that differ from the defaults.
"""

from dataclasses import fields
from pathlib import Path
from typing import Tuple

import yaml

from agent.params import ActionParams, AgentParams


def _build(cls, data: dict):
    """Construct `cls` using only keys it knows about; others are silently dropped."""
    known = {f.name for f in fields(cls)}
    filtered = {k: v for k, v in data.items() if k in known}
    return cls(**filtered)


def load_params(path: Path) -> Tuple[AgentParams, ActionParams]:
    """Parse `path` and return (AgentParams, ActionParams)."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(f"params file not found: {path}")
    raw = yaml.safe_load(path.read_text()) or {}
    agent_data = raw.get("agent", {}) or {}
    actions_data = raw.get("actions", {}) or {}
    return _build(AgentParams, agent_data), _build(ActionParams, actions_data)
