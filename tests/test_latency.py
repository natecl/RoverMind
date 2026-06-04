"""Unit tests for the Mac-side latency collector (agent/latency.py).

All timing is driven by an injected fake clock so durations are exact and
deterministic — no real wall-clock dependence.
"""

import json

import pytest

from agent.latency import (
    Event,
    LatencyCollector,
    TimingLLM,
    summarize,
    timed,
)


class FakeClock:
    """Returns successive values from a list on each call."""

    def __init__(self, values):
        self._values = list(values)
        self._i = 0

    def __call__(self):
        v = self._values[self._i]
        self._i += 1
        return v


class _StubLLM:
    """Minimal LLM stand-in: pops responses; bind_tools returns self."""

    def __init__(self, responses):
        self._responses = list(responses)
        self.bind_calls = []

    def invoke(self, messages):
        return self._responses.pop(0)

    def bind_tools(self, tools, **kw):
        self.bind_calls.append((tools, kw))
        return self


def test_span_records_duration():
    clock = FakeClock([10.0, 10.5])
    c = LatencyCollector(clock=clock)
    with c.span("reason", "llm"):
        pass
    assert len(c.events) == 1
    e = c.events[0]
    assert isinstance(e, Event)
    assert e.category == "reason"
    assert e.label == "llm"
    assert e.duration_ms == 500.0


def test_span_records_on_exception_and_reraises():
    clock = FakeClock([1.0, 1.25])
    c = LatencyCollector(clock=clock)
    with pytest.raises(ValueError):
        with c.span("look", "bottle"):
            raise ValueError("boom")
    assert len(c.events) == 1
    assert c.events[0].duration_ms == 250.0


def test_timing_llm_invoke_records_reason():
    clock = FakeClock([0.0, 1.0])
    c = LatencyCollector(clock=clock)
    llm = TimingLLM(_StubLLM(["A"]), c)
    out = llm.invoke(["msg"])
    assert out == "A"
    assert [e.category for e in c.events] == ["reason"]
    assert c.events[0].duration_ms == 1000.0


def test_bind_tools_returns_proxy_that_records():
    clock = FakeClock([0.0, 0.5])
    c = LatencyCollector(clock=clock)
    base = _StubLLM(["X"])
    bound = TimingLLM(base, c).bind_tools([], tool_choice="required")
    out = bound.invoke(["m"])
    assert out == "X"
    assert len(c.events) == 1
    assert c.events[0].category == "reason"
    # The underlying llm actually received the bind_tools call.
    assert base.bind_calls == [([], {"tool_choice": "required"})]


def test_timed_passes_through_and_labels():
    clock = FakeClock([0.0, 2.0])
    c = LatencyCollector(clock=clock)
    calls = []

    def fn(target):
        calls.append(target)
        return f"obs:{target}"

    wrapped = timed(fn, c, "look", label_fn=lambda target: str(target))
    out = wrapped("bottle")
    assert out == "obs:bottle"
    assert calls == ["bottle"]
    assert c.events[0].category == "look"
    assert c.events[0].label == "bottle"
    assert c.events[0].duration_ms == 2000.0


def test_timed_records_on_exception():
    clock = FakeClock([0.0, 0.1])
    c = LatencyCollector(clock=clock)

    def fn(h, d):
        raise RuntimeError("move failed")

    wrapped = timed(fn, c, "move")
    with pytest.raises(RuntimeError):
        wrapped(30.0, 0.0)
    assert len(c.events) == 1
    assert c.events[0].category == "move"


def test_summarize_per_category_aggregates():
    c = LatencyCollector()
    c.record("reason", "llm", 0.0, 1.0)   # 1000 ms
    c.record("reason", "llm", 2.0, 2.5)   # 500 ms
    c.record("move", "h=30", 3.0, 3.2)    # 200 ms
    d = summarize(c).to_dict()
    reason = d["categories"]["reason"]
    assert reason["count"] == 2
    assert reason["total_ms"] == 1500.0
    assert reason["mean_ms"] == 750.0
    assert reason["min_ms"] == 500.0
    assert reason["max_ms"] == 1000.0
    assert d["categories"]["move"]["count"] == 1


def test_summarize_segments_steps_at_reason_boundaries():
    c = LatencyCollector()
    c.record("reason", "llm", 0.0, 1.0)   # step 1
    c.record("look", "bottle", 1.0, 3.0)
    c.record("reason", "llm", 3.0, 4.0)   # step 2
    c.record("move", "h=30", 4.0, 5.0)
    c.record("reason", "llm", 5.0, 6.0)   # step 3 (stop): no bridge call
    d = summarize(c).to_dict()
    assert len(d["steps"]) == 3
    assert d["steps"][0]["duration_ms"] == 3000.0
    assert d["steps"][1]["duration_ms"] == 2000.0
    assert d["steps"][2]["duration_ms"] == 1000.0
    assert d["total_ms"] == 6000.0


def test_summarize_tolerates_step_without_bridge_event():
    c = LatencyCollector()
    c.record("reason", "llm", 0.0, 1.0)
    c.record("look", "bottle", 1.0, 3.0)
    c.record("reason", "llm", 3.0, 4.0)   # stop step, no bridge
    d = summarize(c).to_dict()
    assert d["steps"][0]["bridge"]["category"] == "look"
    assert d["steps"][1]["bridge"] is None


def test_summarize_flags_first_vlm_as_model_load():
    c = LatencyCollector()
    c.record("look", "bottle", 0.0, 30.0)   # model load: 30000 ms
    c.record("look", "bottle", 31.0, 47.0)  # 16000 ms
    c.record("look", "bottle", 48.0, 64.0)  # 16000 ms
    d = summarize(c).to_dict()
    assert d["vlm_model_load_ms"] == 30000.0
    ss = d["vlm_steady_state"]
    assert ss["count"] == 2
    assert ss["mean_ms"] == 16000.0
    # The generic per-category table still counts all three looks.
    assert d["categories"]["look"]["count"] == 3


def test_to_dict_is_json_serializable():
    c = LatencyCollector()
    c.record("reason", "llm", 0.0, 1.0)
    c.record("look", "bottle", 1.0, 3.0)
    json.dumps(summarize(c).to_dict())  # must not raise


def test_to_dict_includes_raw_events():
    c = LatencyCollector()
    c.record("reason", "llm", 0.0, 1.0)
    c.record("look", "bottle", 1.0, 3.0)
    d = summarize(c).to_dict()
    assert d["events"] == [
        {"category": "reason", "label": "llm", "duration_ms": 1000.0},
        {"category": "look", "label": "bottle", "duration_ms": 2000.0},
    ]


def test_render_returns_string():
    c = LatencyCollector()
    c.record("reason", "llm", 0.0, 1.0)
    out = summarize(c).render()
    assert isinstance(out, str)
    assert "reason" in out


def test_summarize_with_run_span_defines_total_and_scaffolding():
    # run_agent wraps graph.invoke in an outer "run" span; init/setup before
    # the first reason becomes scaffolding (total - sum of step durations).
    c = LatencyCollector()
    c.record("run", "graph", 0.0, 10.0)   # outer span dominates the timeline
    c.record("reason", "llm", 1.0, 2.0)   # first reason starts 1.0s in
    c.record("look", "x", 2.0, 9.0)
    d = summarize(c).to_dict()
    assert d["total_ms"] == 10000.0
    assert len(d["steps"]) == 1
    assert d["steps"][0]["duration_ms"] == 9000.0     # [1.0, 10.0]
    assert d["steps"][0]["bridge"]["category"] == "look"
    assert d["scaffolding_ms"] == 1000.0              # the pre-first-reason head


def test_summarize_includes_rover_decomposition():
    c = LatencyCollector()
    c.record("look", "bottle", 0.0, 16.0)
    c.record_metric("capture_and_analyze", "server_ms", 15800.0)
    c.record_metric("capture_and_analyze", "vlm_ms", 15600.0)
    c.record_metric("capture_and_analyze", "transport_overhead_ms", 200.0)
    c.record_metric("execute_command", "action_ms", 2400.0)
    d = summarize(c).to_dict()
    assert d["rover"]["server_ms"]["mean_ms"] == 15800.0
    assert d["rover"]["vlm_ms"]["count"] == 1
    assert d["rover"]["action_ms"]["mean_ms"] == 2400.0
    # Raw metrics are preserved for the JSON dump.
    assert {"method": "execute_command", "name": "action_ms", "ms": 2400.0} in d["metrics"]


def test_summarize_empty_collector():
    d = summarize(LatencyCollector()).to_dict()
    assert d["steps"] == []
    assert d["total_ms"] == 0.0
    assert d["categories"] == {}
