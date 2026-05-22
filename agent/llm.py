"""Construct the ChatOpenAI instance the reason node will call.

Kept tiny and provider-aware so swapping vendors later is a one-file
change.
"""

import os

from agent.params import AgentParams


def build_llm(params: AgentParams):
    """Return a tool-bindable chat model configured per AgentParams."""
    if params.llm_provider != "openai":
        raise ValueError(
            f"unsupported llm_provider {params.llm_provider!r}; only 'openai' is wired"
        )
    if not os.environ.get("OPENAI_API_KEY"):
        raise RuntimeError(
            "OPENAI_API_KEY environment variable is required for the openai provider"
        )
    # Imported here so test environments without langchain-openai installed
    # can still import agent.llm to inspect errors.
    from langchain_openai import ChatOpenAI

    return ChatOpenAI(
        model=params.llm_model,
        temperature=params.llm_temperature,
    )
