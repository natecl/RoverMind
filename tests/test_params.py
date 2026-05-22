from agent.params import ActionParams, AgentParams


def test_action_params_has_documented_defaults():
    p = ActionParams()
    assert p.turn_small_deg == 30.0
    assert p.turn_large_deg == 60.0
    assert p.search_deg == 45.0
    assert p.forward_short_m == 0.3
    assert p.forward_medium_m == 0.6


def test_agent_params_has_documented_defaults():
    p = AgentParams()
    assert p.llm_provider == "openai"
    assert p.llm_model == "gpt-4o-mini"
    assert p.llm_temperature == 0.0
    assert p.max_steps == 20


def test_action_params_is_frozen():
    p = ActionParams()
    try:
        p.turn_small_deg = 99.0  # type: ignore[misc]
    except Exception:
        return
    raise AssertionError("ActionParams should be frozen")
