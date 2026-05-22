"""Tunable parameters for the LangGraph agent.

Frozen dataclasses so the values are safe to share across nodes and tools.
Defaults match the design spec; production code loads overrides from
config/params.yaml via agent.config_loader.
"""

from dataclasses import dataclass


@dataclass(frozen=True)
class ActionParams:
    """Bucket → numeric resolution for the movement tools."""

    turn_small_deg: float = 30.0
    turn_large_deg: float = 60.0
    search_deg: float = 45.0
    forward_short_m: float = 0.3
    forward_medium_m: float = 0.6


@dataclass(frozen=True)
class AgentParams:
    """LLM and graph-level parameters."""

    llm_provider: str = "openai"
    llm_model: str = "gpt-4o-mini"
    llm_temperature: float = 0.0
    max_steps: int = 20
