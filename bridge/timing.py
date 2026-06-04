"""Per-call timing proxy for the Moondream client (rover side).

Pure stdlib, **importable on Python 3.8** (like wire.py) — no rclpy/torch and
no builtin-generic annotations evaluated at import. Wraps a MoondreamClient so
each `.ask`/`.point` is timed; the bridge server reads the recorded durations
and ships them back in the reply `timing` envelope.
"""

from __future__ import annotations

import time


def classify_question(question):
    """Map one of vision_tool's three questions to a stable label.

    Matches on distinctive substrings rather than exact wording so a small
    prompt tweak doesn't silently drop the label. Anything unrecognised is
    just "ask".
    """
    q = question.lower()
    if "yes or no" in q:
        return "ask_visible"
    if "left" in q and "right" in q:
        return "ask_direction"
    if "close" in q and "far" in q:
        return "ask_distance"
    return "ask"


class TimingMoondream:
    """Times each call to the wrapped Moondream client.

    `self.calls` is an ordered list of `{"op", "label", "ms"}`. Records on the
    exception path too. The wrapped object's return value passes through
    unchanged, so this is a transparent drop-in for `capture_and_analyze`.
    """

    def __init__(self, moondream, clock=time.perf_counter):
        self._moondream = moondream
        self._clock = clock
        self.calls = []

    def ask(self, image, question):
        start = self._clock()
        try:
            return self._moondream.ask(image, question)
        finally:
            self.calls.append({
                "op": "ask",
                "label": classify_question(question),
                "ms": (self._clock() - start) * 1000.0,
            })

    def point(self, image, target):
        start = self._clock()
        try:
            return self._moondream.point(image, target)
        finally:
            self.calls.append({
                "op": "point",
                "label": "point",
                "ms": (self._clock() - start) * 1000.0,
            })

    def aggregate(self):
        """Sum durations by label → {label: ms} for the timing envelope."""
        out = {}
        for c in self.calls:
            out[c["label"]] = out.get(c["label"], 0.0) + c["ms"]
        return out
