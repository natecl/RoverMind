import os

import pytest

from agent.llm import build_llm
from agent.params import AgentParams


def test_build_llm_returns_a_chat_model_object(monkeypatch):
    monkeypatch.setenv("OPENAI_API_KEY", "sk-test-not-real")
    llm = build_llm(AgentParams())
    # ChatOpenAI exposes model_name and temperature attributes; this is a
    # smoke test that the constructor wiring took effect.
    assert getattr(llm, "model_name", None) == "gpt-4o-mini"
    assert getattr(llm, "temperature", None) == 0.0


def test_build_llm_requires_openai_key(monkeypatch):
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    with pytest.raises(RuntimeError, match="OPENAI_API_KEY"):
        build_llm(AgentParams())


def test_build_llm_rejects_unknown_provider(monkeypatch):
    monkeypatch.setenv("OPENAI_API_KEY", "sk-test-not-real")
    with pytest.raises(ValueError, match="provider"):
        build_llm(AgentParams(llm_provider="some-other-cloud"))
