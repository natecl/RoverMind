"""Latency instrumentation for the agent pipeline (Mac side).

Pure stdlib — no langchain/rclpy/torch imports — so it stays laptop-testable
and cheap to keep always on. Everything is built on one primitive:
`LatencyCollector.span(category, label)`, a context manager that records the
elapsed time of the block it wraps (including the exception path).

The collector is wired in at the existing dependency-injection seam in
`scripts/run_agent.py`: the LLM is wrapped in `TimingLLM`, and the two bridge
callables (`execute_command`, `capture_and_analyze`) are wrapped in `timed(...)`.
The graph / nodes / tools are untouched.

Clock domains: durations come from a single injected `clock` (default
`time.perf_counter`). perf_counter deltas are only ever compared within one
host — the only cross-host quantity (transport overhead) is computed in
`bridge/client.py` from two same-clock deltas, not here.
"""

import time
from contextlib import contextmanager
from dataclasses import dataclass, field
from statistics import mean, median
from typing import Callable, Dict, List, Optional


@dataclass(frozen=True)
class Event:
    """One timed span on the collector's timeline."""

    category: str
    label: str
    start: float
    end: float

    @property
    def duration_ms(self) -> float:
        return (self.end - self.start) * 1000.0


@dataclass(frozen=True)
class Metric:
    """A named duration (ms) reported by the rover for one RPC.

    Unlike Event, a Metric carries no timeline position — it is a decomposition
    of a bridge round-trip (server compute, ROS action, VLM, transport).
    """

    method: str
    name: str
    ms: float


class LatencyCollector:
    """Accumulates timed Events and rover-reported Metrics.

    Single-threaded by design (the graph runs synchronously and the bridge is
    one request in flight), so no locking.
    """

    def __init__(self, clock: Callable[[], float] = time.perf_counter):
        self._clock = clock
        self.events: List[Event] = []
        self.metrics: List[Metric] = []

    def record(self, category: str, label: str, start: float, end: float) -> None:
        self.events.append(Event(category, label, start, end))

    def record_metric(self, method: str, name: str, ms: float) -> None:
        self.metrics.append(Metric(method, name, float(ms)))

    @contextmanager
    def span(self, category: str, label: str = ""):
        start = self._clock()
        try:
            yield
        finally:
            self.record(category, label, start, self._clock())


class TimingLLM:
    """Proxy that times `.invoke` and survives `.bind_tools`.

    graph.py binds tools *then* invokes the bound object, so `bind_tools` must
    return another TimingLLM wrapping the bound LLM — otherwise the timed proxy
    is discarded before any invoke happens. Anything else falls through to the
    wrapped LLM via `__getattr__`.
    """

    def __init__(self, llm, collector: LatencyCollector, label: str = "llm"):
        self._llm = llm
        self._collector = collector
        self._label = label

    def invoke(self, *args, **kwargs):
        with self._collector.span("reason", self._label):
            return self._llm.invoke(*args, **kwargs)

    def bind_tools(self, *args, **kwargs):
        return TimingLLM(
            self._llm.bind_tools(*args, **kwargs), self._collector, self._label
        )

    def __getattr__(self, name):
        return getattr(self._llm, name)


def timed(fn: Callable, collector: LatencyCollector, category: str,
          label_fn: Optional[Callable] = None) -> Callable:
    """Wrap a callable so each call is recorded as a `category` span.

    `label_fn(*args, **kwargs)` (when given) computes the event label from the
    call arguments — e.g. the look target or the move heading/distance.
    """

    def wrapper(*args, **kwargs):
        label = label_fn(*args, **kwargs) if label_fn is not None else ""
        with collector.span(category, label):
            return fn(*args, **kwargs)

    return wrapper


# --- Summary ---------------------------------------------------------------


def _aggregate(durations: List[float]) -> Dict[str, float]:
    return {
        "count": len(durations),
        "total_ms": round(sum(durations), 3),
        "mean_ms": round(mean(durations), 3),
        "min_ms": round(min(durations), 3),
        "max_ms": round(max(durations), 3),
        "p50_ms": round(median(durations), 3),
    }


@dataclass
class Summary:
    """Rendered view over a LatencyCollector's events + metrics."""

    categories: Dict[str, Dict[str, float]] = field(default_factory=dict)
    steps: List[dict] = field(default_factory=list)
    total_ms: float = 0.0
    scaffolding_ms: float = 0.0
    vlm_model_load_ms: Optional[float] = None
    vlm_steady_state: Optional[Dict[str, float]] = None
    rover: Dict[str, Dict[str, float]] = field(default_factory=dict)
    events: List[dict] = field(default_factory=list)
    metrics: List[dict] = field(default_factory=list)

    def to_dict(self) -> dict:
        return {
            "total_ms": self.total_ms,
            "scaffolding_ms": self.scaffolding_ms,
            "categories": self.categories,
            "steps": self.steps,
            "vlm_model_load_ms": self.vlm_model_load_ms,
            "vlm_steady_state": self.vlm_steady_state,
            "rover": self.rover,
            "events": self.events,
            "metrics": self.metrics,
        }

    def render(self) -> str:
        lines = ["=== Latency report ==="]
        lines.append(f"total: {self.total_ms:.1f} ms   "
                     f"scaffolding: {self.scaffolding_ms:.1f} ms   "
                     f"steps: {len(self.steps)}")
        lines.append("")
        lines.append("per-category (ms):")
        lines.append(f"  {'category':<12}{'count':>6}{'mean':>10}"
                     f"{'min':>10}{'max':>10}{'p50':>10}{'total':>12}")
        for name, agg in sorted(self.categories.items()):
            lines.append(f"  {name:<12}{agg['count']:>6}{agg['mean_ms']:>10.1f}"
                         f"{agg['min_ms']:>10.1f}{agg['max_ms']:>10.1f}"
                         f"{agg['p50_ms']:>10.1f}{agg['total_ms']:>12.1f}")
        if self.vlm_model_load_ms is not None:
            lines.append("")
            lines.append(f"VLM model-load (first look): {self.vlm_model_load_ms:.1f} ms")
            if self.vlm_steady_state is not None:
                ss = self.vlm_steady_state
                lines.append(f"VLM steady-state: n={ss['count']} "
                             f"mean={ss['mean_ms']:.1f} ms "
                             f"min={ss['min_ms']:.1f} max={ss['max_ms']:.1f}")
        if self.rover:
            lines.append("")
            lines.append("rover decomposition (ms):")
            for name, agg in sorted(self.rover.items()):
                lines.append(f"  {name:<22}{agg['count']:>6}"
                             f"{agg['mean_ms']:>10.1f}{agg['max_ms']:>10.1f}")
        lines.append("")
        lines.append("per-step (ms):")
        for s in self.steps:
            bridge = s["bridge"]
            btxt = (f"{bridge['category']} {bridge['duration_ms']:.1f}"
                    if bridge else "-")
            lines.append(f"  step {s['index']:>2}: total={s['duration_ms']:>8.1f}  "
                         f"reason={s['reason_ms']:>8.1f}  bridge={btxt}")
        return "\n".join(lines)


def summarize(collector: LatencyCollector) -> Summary:
    """Build a Summary from a collector's recorded events and metrics."""
    events = collector.events
    summary = Summary()
    summary.events = [
        {"category": e.category, "label": e.label,
         "duration_ms": round(e.duration_ms, 3)}
        for e in events
    ]
    summary.metrics = [
        {"method": m.method, "name": m.name, "ms": round(m.ms, 3)}
        for m in collector.metrics
    ]
    if not events:
        if collector.metrics:
            summary.rover = _summarize_metrics(collector.metrics)
        return summary

    # Per-category aggregates over every event.
    by_cat: Dict[str, List[float]] = {}
    for e in events:
        by_cat.setdefault(e.category, []).append(e.duration_ms)
    summary.categories = {name: _aggregate(d) for name, d in by_cat.items()}

    # Wall-clock total: from the first span start to the last span end.
    min_start = min(e.start for e in events)
    max_end = max(e.end for e in events)
    summary.total_ms = round((max_end - min_start) * 1000.0, 3)

    # Per-step segmentation: a step opens at each "reason" span start and runs
    # until the next reason start (the last runs to max_end). Everything before
    # the first reason (init) is "scaffolding".
    reasons = sorted((e for e in events if e.category == "reason"),
                     key=lambda e: e.start)
    boundaries = [r.start for r in reasons] + [max_end]
    for i, r in enumerate(reasons):
        lo, hi = boundaries[i], boundaries[i + 1]
        # The one bridge (non-reason) span that falls inside this step, if any.
        bridge = next(
            (e for e in events
             if e.category != "reason" and lo <= e.start < hi), None)
        summary.steps.append({
            "index": i,
            "duration_ms": round((hi - lo) * 1000.0, 3),
            "reason_ms": round(r.duration_ms, 3),
            "bridge": ({"category": bridge.category, "label": bridge.label,
                        "duration_ms": round(bridge.duration_ms, 3)}
                       if bridge else None),
        })
    if reasons:
        summary.scaffolding_ms = round(
            summary.total_ms - sum(s["duration_ms"] for s in summary.steps), 3)

    # VLM model-load outlier: the first "look" is dominated by model load
    # (~25-40 s), so surface it separately and aggregate the rest as steady state.
    looks = [e.duration_ms for e in events if e.category == "look"]
    if looks:
        summary.vlm_model_load_ms = round(looks[0], 3)
        if len(looks) > 1:
            summary.vlm_steady_state = _aggregate(looks[1:])

    if collector.metrics:
        summary.rover = _summarize_metrics(collector.metrics)
    return summary


def _summarize_metrics(metrics: List[Metric]) -> Dict[str, Dict[str, float]]:
    by_name: Dict[str, List[float]] = {}
    for m in metrics:
        by_name.setdefault(m.name, []).append(m.ms)
    return {name: _aggregate(d) for name, d in by_name.items()}
