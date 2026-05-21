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


def test_capture_and_analyze_not_visible_skips_followup_questions():
    moondream = FakeMoondream(["No, there is no bottle.", "UNUSED", "UNUSED"])
    obs = capture_and_analyze(
        "water bottle",
        capture_fn=lambda: "FAKE_IMAGE",
        moondream=moondream,
    )
    assert obs.found is False
    assert obs.direction is None
    assert obs.distance is None
    assert obs.should_stop is False
    # The direction/distance questions must NOT be asked when not visible.
    assert len(moondream.questions) == 1


class FakeMoondreamWithPoint(FakeMoondream):
    """FakeMoondream that also answers point() with a fixed normalised point."""

    def __init__(self, answers, point=(0.5, 0.5)):
        super().__init__(answers)
        self._point = point

    def point(self, image, target):
        return self._point


def _uniform_depth(value_mm, width=20, height=20):
    return [[value_mm for _ in range(width)] for _ in range(height)]


def test_depth_path_uses_metric_distance():
    # Visible + direction asked; distance comes from depth, not the VLM.
    moondream = FakeMoondreamWithPoint(["Yes.", "On the left."])
    obs = capture_and_analyze(
        "water bottle",
        capture_fn=lambda: "FAKE_IMAGE",
        moondream=moondream,
        depth_fn=lambda: _uniform_depth(450),  # 0.45 m everywhere
    )
    assert obs.found is True
    assert obs.direction == "left"
    assert obs.distance == "close"
    assert obs.distance_source == "depth"
    assert obs.distance_m == 0.45
    assert obs.should_stop is True
    # The VLM distance question is NOT asked when depth succeeds.
    assert len(moondream.questions) == 2


def test_depth_path_falls_back_to_vlm_when_depth_invalid():
    # All-zero depth -> no valid sample -> fall back to the VLM distance answer.
    moondream = FakeMoondreamWithPoint(["Yes.", "On the right.", "Far away."])
    obs = capture_and_analyze(
        "water bottle",
        capture_fn=lambda: "FAKE_IMAGE",
        moondream=moondream,
        depth_fn=lambda: _uniform_depth(0),  # all holes
    )
    assert obs.found is True
    assert obs.distance == "far"
    assert obs.distance_source == "vlm"
    assert obs.distance_m is None
    assert len(moondream.questions) == 3  # VLM distance question was asked
