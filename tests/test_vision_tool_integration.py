import pytest

from perception.vision_tool import FrameCaptureError, capture_and_analyze


class FakeMoondream:
    """Returns scripted answers in order; records the questions asked."""

    def __init__(self, answers):
        self._answers = list(answers)
        self.questions = []

    def ask(self, image, question):
        self.questions.append(question)
        return self._answers[len(self.questions) - 1]


def test_capture_and_analyze_found_close():
    moondream = FakeMoondream(["Yes.", "On the left.", "It is close."])
    obs = capture_and_analyze(
        "water bottle",
        capture_fn=lambda: "FAKE_IMAGE",
        moondream=moondream,
    )
    assert obs.found is True
    assert obs.direction == "left"
    assert obs.distance == "close"
    assert obs.should_stop is True
    assert len(moondream.questions) == 3


def test_capture_and_analyze_propagates_capture_error():
    def boom():
        raise FrameCaptureError("no frame on /camera/color/image_raw")

    with pytest.raises(FrameCaptureError):
        capture_and_analyze(
            "water bottle", capture_fn=boom, moondream=FakeMoondream([]),
        )
