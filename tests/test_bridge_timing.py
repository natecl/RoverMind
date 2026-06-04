"""Phase 2b: TimingMoondream proxy times each Moondream call.

Pure unit tests driven by a fake clock — py3.8-safe (no builtin-generic
annotations), matching the constraint on bridge/timing.py itself.
"""

import pytest

from bridge.timing import TimingMoondream, classify_question


class FakeClock:
    def __init__(self, values):
        self._values = list(values)
        self._i = 0

    def __call__(self):
        v = self._values[self._i]
        self._i += 1
        return v


class _FakeMD:
    def __init__(self, ask_replies=None, point_reply=None):
        self._replies = list(ask_replies or [])
        self._point = point_reply
        self.seen = []

    def ask(self, image, question):
        self.seen.append(question)
        return self._replies.pop(0)

    def point(self, image, target):
        return self._point


def test_ask_records_classified_label_and_passes_through():
    clock = FakeClock([0.0, 1.0, 1.0, 3.0])
    md = _FakeMD(["yes", "left"])
    t = TimingMoondream(md, clock=clock)
    assert t.ask("img", "Is there a bottle in this image? Answer yes or no.") == "yes"
    assert t.ask("img", "Is the bottle on the left, in the center, or on the right?") == "left"
    assert [c["label"] for c in t.calls] == ["ask_visible", "ask_direction"]
    assert t.calls[0]["ms"] == 1000.0
    assert t.calls[1]["ms"] == 2000.0
    # The wrapped client actually received the questions.
    assert len(md.seen) == 2


def test_point_records_and_passes_through():
    clock = FakeClock([0.0, 0.5])
    md = _FakeMD(point_reply=(0.5, 0.5))
    t = TimingMoondream(md, clock=clock)
    assert t.point("img", "bottle") == (0.5, 0.5)
    assert t.calls[0]["op"] == "point"
    assert t.calls[0]["label"] == "point"
    assert t.calls[0]["ms"] == 500.0


def test_distance_question_classified():
    clock = FakeClock([0.0, 1.0])
    md = _FakeMD(["close"])
    t = TimingMoondream(md, clock=clock)
    t.ask("img", "Is the bottle close, at a medium distance, or far away?")
    assert t.calls[0]["label"] == "ask_distance"


def test_records_on_exception_with_unclassified_label():
    clock = FakeClock([0.0, 0.2])

    class Boom:
        def ask(self, image, question):
            raise RuntimeError("model exploded")

    t = TimingMoondream(Boom(), clock=clock)
    with pytest.raises(RuntimeError):
        t.ask("img", "something unrecognised")
    assert t.calls[0]["ms"] == 200.0
    assert t.calls[0]["label"] == "ask"


def test_classify_question():
    assert classify_question("Answer yes or no.") == "ask_visible"
    assert classify_question("left, center, or right?") == "ask_direction"
    assert classify_question("close, medium, or far away?") == "ask_distance"
    assert classify_question("anything else") == "ask"
