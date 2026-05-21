# Vision Tool (`capture_and_analyze`) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build the RoverMind agent's `capture_and_analyze(target)` vision tool — grab a camera frame, ask a local Moondream2 VLM where a named object is and how far away it is, and return structured data.

**Architecture:** Mirrors the repo's pattern — pure logic with zero ROS/ML imports (laptop-testable with `pytest`) plus thin hardware-only wrappers (manually verified on the rover). `capture_and_analyze` orchestrates an injected `capture_fn` and an injected Moondream2 client, so the whole flow is testable with fakes. Distance starts as a VLM guess and is later upgraded to a metric reading from the depth camera.

**Tech Stack:** Python 3.10+, Moondream2 (`vikhyatk/moondream2`, local on a Jetson Orin Nano), `transformers`/`torch`, ROS2 Foxy (`rclpy`, `cv_bridge`, `sensor_msgs`), `pytest`.

---

## File structure

**Created by this plan:**

- `perception/__init__.py` — package marker (plain importable Python package).
- `perception/scene_parsing.py` — pure logic: `SceneObservation` dataclass, answer parsers, `build_observation`. Zero ROS/ML imports.
- `perception/depth_math.py` — pure logic: depth-sample → metric-distance helpers (Phase 5). Zero ROS/ML imports.
- `perception/moondream_client.py` — Moondream2 wrapper. Heavy imports (`torch`, `transformers`); hardware/GPU-only.
- `perception/vision_tool.py` — `capture_and_analyze` orchestration + the real `rclpy` capture functions.
- `tests/test_scene_parsing.py` — pure unit tests.
- `tests/test_depth_math.py` — pure unit tests (Phase 5).
- `tests/test_vision_tool_integration.py` — orchestration tests with fakes.
- `scripts/test_vision.py` — manual verification entry point.
- `requirements.txt` — `torch`, `transformers`, `pillow`, `einops`.

**Modified by this plan:**

- `README.md` — mark project Phase 2 done; correct the VLM backbone to Moondream2.

**Deliberate refinement vs. the spec:** the spec's file list mentioned a `perception/package.xml`. On reflection `perception` provides no ROS *nodes* (no `console_scripts`, nothing run via `ros2 run`) — it is a library plus a manual script. It does not need to be a colcon `ament_python` package. `rclpy` / `cv_bridge` / `sensor_msgs` resolve from the sourced ROS2 environment on the rover, exactly as `safety_controller_layer`'s modules already import `rclpy`. So `perception` is a plain Python package and no `package.xml` is created.

**Import path note:** `tests/conftest.py` already adds the repo root to `sys.path`, so `import perception` works under `pytest`. `scripts/test_vision.py` inserts the repo root onto `sys.path` itself (shown in Task 9) so it can be run as `python scripts/test_vision.py` from the repo root.

---

## Phase 1 — Parsed answers → `SceneObservation`

Pure logic. Fully laptop-testable, literal TDD red → green.

### Task 1: `perception` package + `SceneObservation` dataclass

**Files:**
- Create: `perception/__init__.py`
- Create: `perception/scene_parsing.py`
- Test: `tests/test_scene_parsing.py`

- [ ] **Step 1: Create the empty package marker**

Create `perception/__init__.py` with this single line:

```python
"""RoverMind perception package — the agent's vision tool."""
```

- [ ] **Step 2: Write the failing test**

Create `tests/test_scene_parsing.py`:

```python
import dataclasses

import pytest

from perception.scene_parsing import SceneObservation


def test_scene_observation_holds_all_fields():
    obs = SceneObservation(
        target="water bottle",
        found=True,
        direction="left",
        distance="close",
        should_stop=True,
        raw_answers={"visible": "Yes."},
    )
    assert obs.target == "water bottle"
    assert obs.found is True
    assert obs.direction == "left"
    assert obs.distance == "close"
    assert obs.should_stop is True
    assert obs.raw_answers == {"visible": "Yes."}


def test_scene_observation_is_frozen():
    obs = SceneObservation(
        target="bottle", found=False, direction=None, distance=None,
        should_stop=False, raw_answers={},
    )
    with pytest.raises(dataclasses.FrozenInstanceError):
        obs.found = True
```

- [ ] **Step 3: Run test to verify it fails**

Run: `pytest tests/test_scene_parsing.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'perception.scene_parsing'`

- [ ] **Step 4: Write minimal implementation**

Create `perception/scene_parsing.py`:

```python
"""Pure logic for turning Moondream2's free-form answers into structured data.

Zero ROS and zero ML imports — fully unit-testable on the dev laptop.
"""

from dataclasses import dataclass
from typing import Literal, Optional

Direction = Literal["left", "center", "right"]
Distance = Literal["close", "medium", "far"]


@dataclass(frozen=True)
class SceneObservation:
    """Structured result of looking for one target object in one frame."""

    target: str
    found: bool
    direction: Optional[Direction]
    distance: Optional[Distance]
    should_stop: bool
    raw_answers: dict
```

- [ ] **Step 5: Run test to verify it passes**

Run: `pytest tests/test_scene_parsing.py -v`
Expected: PASS (2 passed)

- [ ] **Step 6: Commit**

```bash
git add perception/__init__.py perception/scene_parsing.py tests/test_scene_parsing.py
git commit -m "feat: add perception package and SceneObservation dataclass"
```

### Task 2: `parse_yes_no`

**Files:**
- Modify: `perception/scene_parsing.py`
- Test: `tests/test_scene_parsing.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_scene_parsing.py` (and add `parse_yes_no` to the import from `perception.scene_parsing`):

```python
from perception.scene_parsing import parse_yes_no  # add to existing imports


@pytest.mark.parametrize("answer,expected", [
    ("Yes.", True),
    ("Yes, there is a water bottle on the table.", True),
    ("No.", False),
    ("No, I don't see one.", False),
    ("There is no bottle visible.", False),
    ("", False),
    ("I cannot tell.", False),
    ("Yes, but it is no longer clearly visible.", False),  # both -> safe default
])
def test_parse_yes_no(answer, expected):
    assert parse_yes_no(answer) is expected
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_scene_parsing.py::test_parse_yes_no -v`
Expected: FAIL — `ImportError: cannot import name 'parse_yes_no'`

- [ ] **Step 3: Write minimal implementation**

Add to the top of `perception/scene_parsing.py` (with the other imports):

```python
import re
```

Add this function to `perception/scene_parsing.py`:

```python
def parse_yes_no(answer: str) -> bool:
    """Map a visibility answer to a boolean.

    Finds a 'yes'/'no' token even inside a sentence. Ambiguous input (both
    tokens, neither, or empty) returns False — failing toward 'not found' is
    safe, because the agent then keeps searching instead of acting on a
    phantom target.
    """
    text = answer.lower()
    has_yes = re.search(r"\byes\b", text) is not None
    has_no = re.search(r"\bno\b", text) is not None
    return has_yes and not has_no
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_scene_parsing.py::test_parse_yes_no -v`
Expected: PASS (8 passed)

- [ ] **Step 5: Commit**

```bash
git add perception/scene_parsing.py tests/test_scene_parsing.py
git commit -m "feat: add parse_yes_no answer parser"
```

### Task 3: `parse_direction`

**Files:**
- Modify: `perception/scene_parsing.py`
- Test: `tests/test_scene_parsing.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_scene_parsing.py` (add `parse_direction` to the import):

```python
from perception.scene_parsing import parse_direction  # add to existing imports


@pytest.mark.parametrize("answer,expected", [
    ("It is on the left.", "left"),
    ("The bottle is in the center of the image.", "center"),
    ("It's in the middle.", "center"),
    ("Located in the centre.", "center"),
    ("To the right.", "right"),
    ("It could be on the left or the right.", None),  # ambiguous -> None
    ("", None),
    ("I am not sure where it is.", None),
])
def test_parse_direction(answer, expected):
    assert parse_direction(answer) == expected
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_scene_parsing.py::test_parse_direction -v`
Expected: FAIL — `ImportError: cannot import name 'parse_direction'`

- [ ] **Step 3: Write minimal implementation**

Add to `perception/scene_parsing.py`:

```python
def parse_direction(answer: str) -> Optional[Direction]:
    """Map a direction answer to left/center/right.

    Accepts 'middle' and 'centre' as synonyms for center. If the answer names
    more than one direction, or none, it is ambiguous and returns None.
    """
    text = answer.lower()
    present = {
        "left": "left" in text,
        "center": ("center" in text or "centre" in text or "middle" in text),
        "right": "right" in text,
    }
    matches = [name for name, found in present.items() if found]
    if len(matches) == 1:
        return matches[0]
    return None
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_scene_parsing.py::test_parse_direction -v`
Expected: PASS (8 passed)

- [ ] **Step 5: Commit**

```bash
git add perception/scene_parsing.py tests/test_scene_parsing.py
git commit -m "feat: add parse_direction answer parser"
```

### Task 4: `parse_distance`

**Files:**
- Modify: `perception/scene_parsing.py`
- Test: `tests/test_scene_parsing.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_scene_parsing.py` (add `parse_distance` to the import):

```python
from perception.scene_parsing import parse_distance  # add to existing imports


@pytest.mark.parametrize("answer,expected", [
    ("It is close.", "close"),
    ("The bottle is near the rover.", "close"),
    ("At a medium distance.", "medium"),
    ("It is a moderate distance away.", "medium"),
    ("Far away.", "far"),
    ("It looks distant.", "far"),
    ("It is hard to tell if it is close or far.", None),  # ambiguous -> None
    ("", None),
])
def test_parse_distance(answer, expected):
    assert parse_distance(answer) == expected
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_scene_parsing.py::test_parse_distance -v`
Expected: FAIL — `ImportError: cannot import name 'parse_distance'`

- [ ] **Step 3: Write minimal implementation**

Add to `perception/scene_parsing.py`:

```python
def parse_distance(answer: str) -> Optional[Distance]:
    """Map a distance answer to close/medium/far.

    Accepts synonyms: 'near' -> close; 'moderate' -> medium; 'distant' -> far.
    If the answer names more than one bucket, or none, it returns None.
    """
    text = answer.lower()
    present = {
        "close": ("close" in text or "near" in text),
        "medium": ("medium" in text or "moderate" in text),
        "far": ("far" in text or "distant" in text),
    }
    matches = [name for name, found in present.items() if found]
    if len(matches) == 1:
        return matches[0]
    return None
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_scene_parsing.py::test_parse_distance -v`
Expected: PASS (8 passed)

- [ ] **Step 5: Commit**

```bash
git add perception/scene_parsing.py tests/test_scene_parsing.py
git commit -m "feat: add parse_distance answer parser"
```

### Task 5: `build_observation`

**Files:**
- Modify: `perception/scene_parsing.py`
- Test: `tests/test_scene_parsing.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_scene_parsing.py` (add `build_observation` to the import):

```python
from perception.scene_parsing import build_observation  # add to existing imports


def test_build_observation_found_and_close_sets_should_stop():
    obs = build_observation("water bottle", "Yes.", "On the left.", "It is close.")
    assert obs.found is True
    assert obs.direction == "left"
    assert obs.distance == "close"
    assert obs.should_stop is True
    assert obs.raw_answers["visible"] == "Yes."


def test_build_observation_found_but_far_does_not_stop():
    obs = build_observation("water bottle", "Yes.", "On the right.", "Far away.")
    assert obs.found is True
    assert obs.distance == "far"
    assert obs.should_stop is False


def test_build_observation_not_found_zeroes_fields():
    obs = build_observation("water bottle", "No.", "", "")
    assert obs.found is False
    assert obs.direction is None
    assert obs.distance is None
    assert obs.should_stop is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_scene_parsing.py -k build_observation -v`
Expected: FAIL — `ImportError: cannot import name 'build_observation'`

- [ ] **Step 3: Write minimal implementation**

Add to `perception/scene_parsing.py`:

```python
def build_observation(
    target: str,
    visible_answer: str,
    direction_answer: str,
    distance_answer: str,
) -> SceneObservation:
    """Assemble a SceneObservation from three raw Moondream2 answers.

    `should_stop` is derived here (found and distance == "close"), never asked
    of the VLM. When the target is not found, direction/distance are None and
    should_stop is False.
    """
    raw_answers = {
        "visible": visible_answer,
        "direction": direction_answer,
        "distance": distance_answer,
    }
    found = parse_yes_no(visible_answer)
    if not found:
        return SceneObservation(
            target=target, found=False, direction=None, distance=None,
            should_stop=False, raw_answers=raw_answers,
        )
    direction = parse_direction(direction_answer)
    distance = parse_distance(distance_answer)
    should_stop = distance == "close"
    return SceneObservation(
        target=target, found=True, direction=direction, distance=distance,
        should_stop=should_stop, raw_answers=raw_answers,
    )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_scene_parsing.py -v`
Expected: PASS (all tests in the file pass)

- [ ] **Step 5: Commit**

```bash
git add perception/scene_parsing.py tests/test_scene_parsing.py
git commit -m "feat: add build_observation assembling SceneObservation"
```

---

## Phase 2 — Full tool orchestration (with fakes)

`capture_and_analyze` and the not-visible early return. Laptop-testable with fakes, literal TDD red → green.

### Task 6: `capture_and_analyze` happy path + `FrameCaptureError`

**Files:**
- Create: `perception/vision_tool.py`
- Test: `tests/test_vision_tool_integration.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_vision_tool_integration.py`:

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_vision_tool_integration.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'perception.vision_tool'`

- [ ] **Step 3: Write minimal implementation**

Create `perception/vision_tool.py`:

```python
"""capture_and_analyze: grab a frame, ask Moondream2, return a SceneObservation.

The capture function and the Moondream2 client are injected so the whole
orchestration is testable with fakes. The real rclpy capture lives here too
(ros_capture_fn) but is hardware-only.
"""

from perception.scene_parsing import build_observation, parse_yes_no

_VISIBLE_Q = "Is there a {target} in this image? Answer yes or no."
_DIRECTION_Q = (
    "Is the {target} on the left, in the center, or on the right of the image?"
)
_DISTANCE_Q = "Is the {target} close, at a medium distance, or far away?"


class FrameCaptureError(RuntimeError):
    """Raised when no camera frame can be obtained within the timeout."""


def capture_and_analyze(target, *, capture_fn, moondream):
    """Capture one frame, ask Moondream2 about `target`, return a SceneObservation.

    `capture_fn()` returns an image (or raises FrameCaptureError). `moondream`
    is an object with `ask(image, question) -> str`.
    """
    image = capture_fn()
    visible_answer = moondream.ask(image, _VISIBLE_Q.format(target=target))
    direction_answer = moondream.ask(image, _DIRECTION_Q.format(target=target))
    distance_answer = moondream.ask(image, _DISTANCE_Q.format(target=target))
    return build_observation(
        target, visible_answer, direction_answer, distance_answer,
    )
```

Note: `parse_yes_no` is imported now because Task 7 uses it; it is unused until then.

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_vision_tool_integration.py -v`
Expected: PASS (2 passed)

- [ ] **Step 5: Commit**

```bash
git add perception/vision_tool.py tests/test_vision_tool_integration.py
git commit -m "feat: add capture_and_analyze orchestration"
```

### Task 7: Not-visible early return

**Files:**
- Modify: `perception/vision_tool.py`
- Test: `tests/test_vision_tool_integration.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_vision_tool_integration.py`:

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_vision_tool_integration.py::test_capture_and_analyze_not_visible_skips_followup_questions -v`
Expected: FAIL — `assert 3 == 1` (current code always asks all three questions)

- [ ] **Step 3: Write minimal implementation**

Replace the body of `capture_and_analyze` in `perception/vision_tool.py` with:

```python
def capture_and_analyze(target, *, capture_fn, moondream):
    """Capture one frame, ask Moondream2 about `target`, return a SceneObservation.

    `capture_fn()` returns an image (or raises FrameCaptureError). `moondream`
    is an object with `ask(image, question) -> str`. When the target is not
    visible, the direction/distance questions are skipped.
    """
    image = capture_fn()
    visible_answer = moondream.ask(image, _VISIBLE_Q.format(target=target))
    if not parse_yes_no(visible_answer):
        return build_observation(target, visible_answer, "", "")
    direction_answer = moondream.ask(image, _DIRECTION_Q.format(target=target))
    distance_answer = moondream.ask(image, _DISTANCE_Q.format(target=target))
    return build_observation(
        target, visible_answer, direction_answer, distance_answer,
    )
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_vision_tool_integration.py -v`
Expected: PASS (3 passed)

- [ ] **Step 5: Commit**

```bash
git add perception/vision_tool.py tests/test_vision_tool_integration.py
git commit -m "feat: skip follow-up VLM questions when target not visible"
```

---

## Phase 3 — Real Moondream2 on a still image

Hardware/GPU. No laptop `pytest` for the model (heavy import, needs CUDA) — verification criteria are written first, then made to pass on a GPU box, consistent with this repo's convention for `rclpy`/model code.

### Task 8: `requirements.txt` + `MoondreamClient`

**Files:**
- Create: `requirements.txt`
- Create: `perception/moondream_client.py`

- [ ] **Step 1: Create `requirements.txt`**

Create `requirements.txt`:

```
# RoverMind Python dependencies for VLM perception.
# rclpy, cv_bridge and sensor_msgs are NOT here — they come from the sourced
# ROS2 environment on the rover, not from pip.
torch
transformers
pillow
einops
```

- [ ] **Step 2: Write down the verification criteria**

The check for this task (run in Step 5): `python -c "from perception.moondream_client import MoondreamClient"` must succeed on the dev laptop *without* importing `torch`/`transformers` at module load — those are imported lazily inside `__init__`, so the import itself is laptop-safe. Constructing a `MoondreamClient` is GPU-only and is exercised in Task 9.

- [ ] **Step 3: Write the implementation**

Create `perception/moondream_client.py`:

```python
"""Moondream2 wrapper. Heavy imports (torch, transformers) are deferred to
construction so this module can be imported on the dev laptop. Constructing a
MoondreamClient downloads/loads the model and requires a CUDA GPU.
"""

MODEL_ID = "vikhyatk/moondream2"
# Pin the revision: Moondream2's transformers API has changed across releases.
# This release exposes model.query(image, question) and model.point(image, obj).
MODEL_REVISION = "2025-06-21"


class MoondreamClient:
    """Loads Moondream2 once and answers questions about images."""

    def __init__(self, device: str = "cuda"):
        from transformers import AutoModelForCausalLM

        self._model = AutoModelForCausalLM.from_pretrained(
            MODEL_ID,
            revision=MODEL_REVISION,
            trust_remote_code=True,
            device_map={"": device},
        )

    def ask(self, image, question: str) -> str:
        """Ask one free-form question about `image`; return the answer string."""
        result = self._model.query(image, question)
        return result["answer"]
```

- [ ] **Step 4: Run the laptop import check**

Run: `python -c "from perception.moondream_client import MoondreamClient; print('import ok')"`
Expected: prints `import ok` with no error (no GPU needed — `torch`/`transformers` are not imported until a client is constructed).

- [ ] **Step 5: Commit**

```bash
git add requirements.txt perception/moondream_client.py
git commit -m "feat: add MoondreamClient wrapper and requirements.txt"
```

### Task 9: `scripts/test_vision.py` — static-image mode

**Files:**
- Create: `scripts/test_vision.py`

- [ ] **Step 1: Write down the verification criteria**

On a CUDA box (a GPU laptop or the Orin itself) with the deps installed and a saved photo of a known object, running the script must print a `SceneObservation` whose `found` is `True` and whose `direction`/`distance` plausibly match the photo. Example: a photo with a bottle on the left, fairly near, should yield `found=True`, `direction='left'`, `distance` of `close` or `medium`.

- [ ] **Step 2: Write the implementation**

Create `scripts/test_vision.py`:

```python
#!/usr/bin/env python3
"""Manual verification for the capture_and_analyze vision tool (static image).

Usage:
  python scripts/test_vision.py --target "water bottle" --image photo.jpg

Requires a CUDA GPU and the dependencies in requirements.txt.
"""

import argparse
import sys
from pathlib import Path

# Make `perception` importable when run as `python scripts/test_vision.py`.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from PIL import Image  # noqa: E402

from perception.moondream_client import MoondreamClient  # noqa: E402
from perception.vision_tool import capture_and_analyze  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", required=True, help="object to look for")
    parser.add_argument("--image", required=True, help="path to an image file")
    args = parser.parse_args()

    print("Loading Moondream2 (first run downloads the model)...")
    moondream = MoondreamClient()

    image = Image.open(args.image).convert("RGB")
    observation = capture_and_analyze(
        args.target, capture_fn=lambda: image, moondream=moondream,
    )
    print(observation)
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 3: Run the verification on a GPU box**

Run (on a CUDA box, from the repo root, with a real photo named `photo.jpg`):
`python scripts/test_vision.py --target "water bottle" --image photo.jpg`
Expected: after the model loads, a printed `SceneObservation(...)` line.

- [ ] **Step 4: Confirm the output matches the criteria**

Confirm `found`, `direction` and `distance` plausibly match what is in the photo (per Step 1). If they do not, the prompts in `perception/vision_tool.py` need tuning — record the observed vs. expected and revisit, but do not block the commit on perfect accuracy (prompt tuning is a later roadmap phase).

- [ ] **Step 5: Commit**

```bash
git add scripts/test_vision.py
git commit -m "feat: add test_vision script for static-image verification"
```

---

## Phase 4 — Live camera capture on the rover

Hardware-only (`rclpy`). Verified on the rover.

### Task 10: `ros_capture_fn`

**Files:**
- Modify: `perception/vision_tool.py`

- [ ] **Step 1: Write down the verification criteria**

`ros_capture_fn()` must, on the rover with the LIMO camera driver running, return a `PIL.Image` (RGB) grabbed from `/camera/color/image_raw`, and must raise `FrameCaptureError` if no frame arrives within the timeout. It is exercised end-to-end through `scripts/test_vision.py` in Task 11; this task adds the function and checks it imports cleanly.

- [ ] **Step 2: Write the implementation**

Add this function to `perception/vision_tool.py` (`rclpy`, `cv_bridge`, `sensor_msgs`, `cv2` are imported *inside* the function so the module stays importable on the dev laptop and on a non-ROS GPU box):

```python
def ros_capture_fn(topic: str = "/camera/color/image_raw",
                   timeout_s: float = 5.0):
    """Grab one frame off the camera topic as a PIL RGB image.

    Hardware-only: needs a sourced ROS2 environment. Raises FrameCaptureError
    if no frame arrives within `timeout_s`.
    """
    import time

    import cv2
    import rclpy
    from cv_bridge import CvBridge
    from PIL import Image
    from rclpy.node import Node
    from sensor_msgs.msg import Image as RosImage

    rclpy.init()
    node = Node("vision_tool_capture")
    bridge = CvBridge()
    received = {}

    def _on_image(msg):
        received["msg"] = msg

    node.create_subscription(RosImage, topic, _on_image, 10)
    try:
        deadline = time.monotonic() + timeout_s
        while "msg" not in received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if "msg" not in received:
            raise FrameCaptureError(
                f"no frame on {topic} within {timeout_s:.1f}s"
            )
        bgr = bridge.imgmsg_to_cv2(received["msg"], desired_encoding="bgr8")
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        return Image.fromarray(rgb)
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 3: Run the laptop import check**

Run: `python -c "from perception.vision_tool import ros_capture_fn; print('import ok')"`
Expected: prints `import ok` (the ROS imports are deferred, so importing the symbol works without `rclpy` installed).

- [ ] **Step 4: Commit**

```bash
git add perception/vision_tool.py
git commit -m "feat: add ros_capture_fn for live camera frame capture"
```

### Task 11: `scripts/test_vision.py` — live mode + rover verification

**Files:**
- Modify: `scripts/test_vision.py`

- [ ] **Step 1: Write down the verification criteria**

On the rover, with the LIMO camera driver running, `python scripts/test_vision.py --target "water bottle"` (no `--image`) must grab a live frame and print a `SceneObservation`. With a bottle placed on the rover's left at close range, expect `found=True`, `direction='left'`, `distance` of `close`/`medium`. With no bottle in view, expect `found=False`.

- [ ] **Step 2: Write the implementation**

Replace the whole contents of `scripts/test_vision.py` with:

```python
#!/usr/bin/env python3
"""Manual verification for the capture_and_analyze vision tool.

Static image (any CUDA box):
  python scripts/test_vision.py --target "water bottle" --image photo.jpg

Live camera (on the rover, camera driver running):
  python scripts/test_vision.py --target "water bottle"

Requires a CUDA GPU and the dependencies in requirements.txt.
"""

import argparse
import sys
from pathlib import Path

# Make `perception` importable when run as `python scripts/test_vision.py`.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from perception.moondream_client import MoondreamClient  # noqa: E402
from perception.vision_tool import capture_and_analyze, ros_capture_fn  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", required=True, help="object to look for")
    parser.add_argument(
        "--image", default=None,
        help="path to an image file; omit to capture live from the camera",
    )
    args = parser.parse_args()

    print("Loading Moondream2 (first run downloads the model)...")
    moondream = MoondreamClient()

    if args.image is not None:
        from PIL import Image
        image = Image.open(args.image).convert("RGB")
        capture_fn = lambda: image  # noqa: E731
    else:
        capture_fn = ros_capture_fn

    observation = capture_and_analyze(
        args.target, capture_fn=capture_fn, moondream=moondream,
    )
    print(observation)
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 3: Run the verification on the rover**

On the rover, with the LIMO camera driver running, from the repo root:
`python scripts/test_vision.py --target "water bottle"`
Expected: after the model loads, a printed `SceneObservation(...)` line from a live frame.

- [ ] **Step 4: Confirm the output matches the criteria**

Place a bottle to the rover's left at close range and re-run; confirm `found=True` and `direction='left'`. Remove the bottle and re-run; confirm `found=False`. Record observed vs. expected; prompt tuning is a later roadmap phase, so do not block the commit on perfect accuracy.

- [ ] **Step 5: Commit**

```bash
git add scripts/test_vision.py
git commit -m "feat: add live-camera mode to test_vision script"
```

---

## Phase 5 — Metric distance from the depth camera

Upgrades the distance field from a VLM guess to a metric reading. Pure depth math is literal `pytest` TDD; the camera wiring is hardware-verified.

### Task 12: Extend `SceneObservation` with depth fields

**Files:**
- Modify: `perception/scene_parsing.py`
- Test: `tests/test_scene_parsing.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_scene_parsing.py`:

```python
def test_scene_observation_depth_fields_default_to_vlm():
    obs = SceneObservation(
        target="bottle", found=True, direction="left", distance="close",
        should_stop=True, raw_answers={},
    )
    assert obs.distance_m is None
    assert obs.distance_source == "vlm"


def test_scene_observation_accepts_depth_fields():
    obs = SceneObservation(
        target="bottle", found=True, direction="left", distance="close",
        should_stop=True, raw_answers={}, distance_m=0.42,
        distance_source="depth",
    )
    assert obs.distance_m == 0.42
    assert obs.distance_source == "depth"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_scene_parsing.py -k depth_fields -v`
Expected: FAIL — `TypeError: __init__() got an unexpected keyword argument 'distance_m'`

- [ ] **Step 3: Write minimal implementation**

In `perception/scene_parsing.py`, replace the `SceneObservation` class with:

```python
@dataclass(frozen=True)
class SceneObservation:
    """Structured result of looking for one target object in one frame."""

    target: str
    found: bool
    direction: Optional[Direction]
    distance: Optional[Distance]
    should_stop: bool
    raw_answers: dict
    # Populated only when distance came from the depth camera (Phase 5);
    # the VLM-only path leaves the defaults.
    distance_m: Optional[float] = None
    distance_source: Literal["depth", "vlm"] = "vlm"
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_scene_parsing.py -v`
Expected: PASS (all tests in the file pass — the existing `build_observation` tests still pass because the new fields have defaults)

- [ ] **Step 5: Commit**

```bash
git add perception/scene_parsing.py tests/test_scene_parsing.py
git commit -m "feat: add distance_m and distance_source to SceneObservation"
```

### Task 13: `depth_to_distance_bucket`

**Files:**
- Create: `perception/depth_math.py`
- Test: `tests/test_depth_math.py`

- [ ] **Step 1: Write the failing test**

Create `tests/test_depth_math.py`:

```python
from perception.depth_math import depth_to_distance_bucket


def test_close_reading():
    bucket, metres = depth_to_distance_bucket([0.4, 0.42, 0.41])
    assert bucket == "close"
    assert metres == 0.41


def test_medium_reading():
    bucket, metres = depth_to_distance_bucket([1.0, 1.1, 0.9])
    assert bucket == "medium"
    assert metres == 1.0


def test_far_reading():
    bucket, metres = depth_to_distance_bucket([3.0, 3.0, 3.0])
    assert bucket == "far"
    assert metres == 3.0


def test_all_invalid_returns_none():
    bucket, metres = depth_to_distance_bucket([0.0, 0.0, 0.0])
    assert bucket is None
    assert metres is None


def test_median_ignores_invalid_zeros_and_outliers():
    # 0.0 values are invalid and dropped; median of [0.5, 0.5, 0.5] is 0.5.
    bucket, metres = depth_to_distance_bucket([0.0, 0.5, 0.0, 0.5, 0.5])
    assert bucket == "close"
    assert metres == 0.5
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_depth_math.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'perception.depth_math'`

- [ ] **Step 3: Write minimal implementation**

Create `perception/depth_math.py`:

```python
"""Pure logic: depth-camera samples -> metric distance buckets.

Zero ROS and zero ML imports — fully unit-testable on the dev laptop.
"""

import math
from typing import Optional, Tuple

from perception.scene_parsing import Distance

# Bucket thresholds in metres. `close` extends a little past the controller's
# stop_distance (0.4 m) so "close" trips before the rover reaches the target.
CLOSE_MAX_M = 0.6
MEDIUM_MAX_M = 1.5


def depth_to_distance_bucket(
    depth_values_m,
    close_max_m: float = CLOSE_MAX_M,
    medium_max_m: float = MEDIUM_MAX_M,
) -> Tuple[Optional[Distance], Optional[float]]:
    """Reduce depth samples (in metres) to a (bucket, median_distance) pair.

    A sample is valid when finite and > 0 (a 0 reading is a depth-camera hole).
    Returns (None, None) when no sample is valid. Otherwise returns the bucket
    for the median valid sample and that median distance.
    """
    valid = sorted(
        d for d in depth_values_m
        if d is not None and math.isfinite(d) and d > 0.0
    )
    if not valid:
        return (None, None)
    n = len(valid)
    if n % 2 == 1:
        median = valid[n // 2]
    else:
        median = (valid[n // 2 - 1] + valid[n // 2]) / 2.0
    if median <= close_max_m:
        return ("close", median)
    if median <= medium_max_m:
        return ("medium", median)
    return ("far", median)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_depth_math.py -v`
Expected: PASS (5 passed)

- [ ] **Step 5: Commit**

```bash
git add perception/depth_math.py tests/test_depth_math.py
git commit -m "feat: add depth_to_distance_bucket metric distance logic"
```

### Task 14: `sample_depth_patch`

**Files:**
- Modify: `perception/depth_math.py`
- Test: `tests/test_depth_math.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_depth_math.py` (add `sample_depth_patch` to the import):

```python
from perception.depth_math import sample_depth_patch  # add to existing imports


def _grid(value_mm, width=10, height=10):
    """A height x width depth image where every pixel is value_mm."""
    return [[value_mm for _ in range(width)] for _ in range(height)]


def test_sample_depth_patch_converts_mm_to_metres():
    depth_mm = _grid(500)  # every pixel 500 mm
    samples = sample_depth_patch(depth_mm, 0.5, 0.5, patch_radius=1)
    assert samples == [0.5] * 9  # 3x3 patch, all 0.5 m


def test_sample_depth_patch_clamps_to_image_bounds():
    depth_mm = _grid(800)
    # Point at the top-left corner; the patch is clipped to in-bounds pixels.
    samples = sample_depth_patch(depth_mm, 0.0, 0.0, patch_radius=2)
    assert all(s == 0.8 for s in samples)
    assert len(samples) == 9  # 3x3 of the 5x5 patch lies inside the image
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_depth_math.py -k sample_depth_patch -v`
Expected: FAIL — `ImportError: cannot import name 'sample_depth_patch'`

- [ ] **Step 3: Write minimal implementation**

Add to `perception/depth_math.py`:

```python
def sample_depth_patch(depth_mm, x_norm: float, y_norm: float,
                       patch_radius: int = 2):
    """Sample a square patch of a depth image around a normalised point.

    `depth_mm` is a height x width grid (list-of-lists or ndarray) of raw depth
    in millimetres. `x_norm`/`y_norm` are in [0, 1]. Returns a flat list of the
    in-bounds patch values converted to metres (0 mm holes are kept as 0.0 and
    filtered later by depth_to_distance_bucket).
    """
    height = len(depth_mm)
    width = len(depth_mm[0])
    cx = min(width - 1, max(0, round(x_norm * (width - 1))))
    cy = min(height - 1, max(0, round(y_norm * (height - 1))))
    samples = []
    for yy in range(cy - patch_radius, cy + patch_radius + 1):
        for xx in range(cx - patch_radius, cx + patch_radius + 1):
            if 0 <= yy < height and 0 <= xx < width:
                samples.append(depth_mm[yy][xx] / 1000.0)
    return samples
```

- [ ] **Step 4: Run test to verify it passes**

Run: `pytest tests/test_depth_math.py -v`
Expected: PASS (7 passed)

- [ ] **Step 5: Commit**

```bash
git add perception/depth_math.py tests/test_depth_math.py
git commit -m "feat: add sample_depth_patch depth-image sampler"
```

### Task 15: Depth-aware `capture_and_analyze` + `build_observation` override

**Files:**
- Modify: `perception/scene_parsing.py`
- Modify: `perception/vision_tool.py`
- Test: `tests/test_vision_tool_integration.py`

- [ ] **Step 1: Write the failing test**

Append to `tests/test_vision_tool_integration.py`:

```python
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
```

- [ ] **Step 2: Run test to verify it fails**

Run: `pytest tests/test_vision_tool_integration.py -k depth -v`
Expected: FAIL — `TypeError: capture_and_analyze() got an unexpected keyword argument 'depth_fn'`

- [ ] **Step 3: Extend `build_observation`**

In `perception/scene_parsing.py`, replace the `build_observation` function with:

```python
def build_observation(
    target: str,
    visible_answer: str,
    direction_answer: str,
    distance_answer: str,
    *,
    distance_override: Optional[Distance] = None,
    distance_m: Optional[float] = None,
    distance_source: Literal["depth", "vlm"] = "vlm",
) -> SceneObservation:
    """Assemble a SceneObservation from raw Moondream2 answers.

    `should_stop` is derived here (found and distance == "close"), never asked
    of the VLM. When the target is not found, direction/distance are None and
    should_stop is False. When `distance_override` is given (depth path), it is
    used instead of parsing `distance_answer`.
    """
    raw_answers = {
        "visible": visible_answer,
        "direction": direction_answer,
        "distance": distance_answer,
    }
    found = parse_yes_no(visible_answer)
    if not found:
        return SceneObservation(
            target=target, found=False, direction=None, distance=None,
            should_stop=False, raw_answers=raw_answers,
        )
    direction = parse_direction(direction_answer)
    if distance_override is not None:
        distance = distance_override
    else:
        distance = parse_distance(distance_answer)
    should_stop = distance == "close"
    return SceneObservation(
        target=target, found=True, direction=direction, distance=distance,
        should_stop=should_stop, raw_answers=raw_answers,
        distance_m=distance_m, distance_source=distance_source,
    )
```

- [ ] **Step 4: Extend `capture_and_analyze`**

In `perception/vision_tool.py`, update the imports at the top of the file to:

```python
from perception.depth_math import depth_to_distance_bucket, sample_depth_patch
from perception.scene_parsing import build_observation, parse_yes_no
```

Then replace the `capture_and_analyze` function with:

```python
def capture_and_analyze(target, *, capture_fn, moondream, depth_fn=None):
    """Capture one frame, ask Moondream2 about `target`, return a SceneObservation.

    `capture_fn()` returns an image (or raises FrameCaptureError). `moondream`
    has `ask(image, question) -> str` and `point(image, target) -> (x, y)|None`.

    When `depth_fn` is given, distance is read from the depth camera: Moondream2
    points at the target, the depth patch at that point is sampled, and a metric
    bucket is computed. If the target cannot be pointed at, or the depth patch
    is all-invalid, the tool falls back to asking Moondream2 for the distance.
    """
    image = capture_fn()
    visible_answer = moondream.ask(image, _VISIBLE_Q.format(target=target))
    if not parse_yes_no(visible_answer):
        return build_observation(target, visible_answer, "", "")
    direction_answer = moondream.ask(image, _DIRECTION_Q.format(target=target))

    if depth_fn is not None:
        point = moondream.point(image, target)
        if point is not None:
            x_norm, y_norm = point
            samples = sample_depth_patch(depth_fn(), x_norm, y_norm)
            bucket, distance_m = depth_to_distance_bucket(samples)
            if bucket is not None:
                return build_observation(
                    target, visible_answer, direction_answer, "",
                    distance_override=bucket, distance_m=distance_m,
                    distance_source="depth",
                )

    distance_answer = moondream.ask(image, _DISTANCE_Q.format(target=target))
    return build_observation(
        target, visible_answer, direction_answer, distance_answer,
    )
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `pytest tests/test_vision_tool_integration.py tests/test_scene_parsing.py -v`
Expected: PASS (all tests pass — the Phase 2 tests still pass because `depth_fn` defaults to `None`)

- [ ] **Step 6: Commit**

```bash
git add perception/scene_parsing.py perception/vision_tool.py tests/test_vision_tool_integration.py
git commit -m "feat: add depth-camera distance path to capture_and_analyze"
```

### Task 16: `MoondreamClient.point` + `ros_depth_capture_fn` + rover verification

**Files:**
- Modify: `perception/moondream_client.py`
- Modify: `perception/vision_tool.py`
- Modify: `scripts/test_vision.py`

- [ ] **Step 1: Write down the verification criteria**

On the rover, with the LIMO camera + depth driver running, `python scripts/test_vision.py --target "water bottle" --depth` must print a `SceneObservation` with `distance_source='depth'` and a `distance_m` close to the tape-measured distance from the camera to the bottle (within ~15%). With the bottle ~0.4 m away, expect `distance='close'`.

- [ ] **Step 2: Add `MoondreamClient.point`**

Add this method to the `MoondreamClient` class in `perception/moondream_client.py`:

```python
    def point(self, image, target: str):
        """Locate `target`; return its normalised (x, y) in [0, 1], or None."""
        result = self._model.point(image, target)
        points = result.get("points", [])
        if not points:
            return None
        first = points[0]
        return (first["x"], first["y"])
```

- [ ] **Step 3: Add `ros_depth_capture_fn`**

Add this function to `perception/vision_tool.py` (ROS imports inside, like `ros_capture_fn`):

```python
def ros_depth_capture_fn(topic: str = "/camera/depth/image_raw",
                         timeout_s: float = 5.0):
    """Grab one depth frame as a height x width grid of millimetres.

    Hardware-only: needs a sourced ROS2 environment. Raises FrameCaptureError
    if no frame arrives within `timeout_s`. The depth topic must be registered
    (aligned) to the colour image so a colour-image point indexes the same
    pixel in the depth image.
    """
    import time

    import rclpy
    from cv_bridge import CvBridge
    from rclpy.node import Node
    from sensor_msgs.msg import Image as RosImage

    rclpy.init()
    node = Node("vision_tool_depth_capture")
    bridge = CvBridge()
    received = {}

    def _on_depth(msg):
        received["msg"] = msg

    node.create_subscription(RosImage, topic, _on_depth, 10)
    try:
        deadline = time.monotonic() + timeout_s
        while "msg" not in received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if "msg" not in received:
            raise FrameCaptureError(
                f"no depth frame on {topic} within {timeout_s:.1f}s"
            )
        # passthrough keeps the raw 16-bit millimetre values.
        depth = bridge.imgmsg_to_cv2(received["msg"], desired_encoding="passthrough")
        return depth
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

- [ ] **Step 4: Add a `--depth` flag to `scripts/test_vision.py`**

In `scripts/test_vision.py`, update the `vision_tool` import to:

```python
from perception.vision_tool import (  # noqa: E402
    capture_and_analyze, ros_capture_fn, ros_depth_capture_fn,
)
```

Add this argument inside `main()`, after the `--image` argument:

```python
    parser.add_argument(
        "--depth", action="store_true",
        help="read distance from the depth camera instead of the VLM",
    )
```

And replace the `capture_and_analyze(...)` call in `main()` with:

```python
    depth_fn = ros_depth_capture_fn if args.depth else None
    observation = capture_and_analyze(
        args.target, capture_fn=capture_fn, moondream=moondream,
        depth_fn=depth_fn,
    )
```

- [ ] **Step 5: Run the verification on the rover**

On the rover, with the camera + depth driver running, from the repo root:
`python scripts/test_vision.py --target "water bottle" --depth`
Expected: a printed `SceneObservation(...)` with `distance_source='depth'`.

- [ ] **Step 6: Confirm the output matches the criteria**

Tape-measure the camera-to-bottle distance and confirm `distance_m` is within ~15% of it and `distance` is the matching bucket (per Step 1). If `--depth` produces no depth reading, confirm the topic name — the LIMO's depth topic may differ from the `/camera/depth/image_raw` default; pass the correct name (the default is the only thing to adjust). Record observed vs. expected.

- [ ] **Step 7: Commit**

```bash
git add perception/moondream_client.py perception/vision_tool.py scripts/test_vision.py
git commit -m "feat: add depth-camera capture to the vision tool"
```

---

## Phase 6 — Documentation

### Task 17: Update the README

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Correct the VLM backbone line**

In `README.md`, in the "Tech Stack" section, replace:

```
- **VLM Backbone:** Claude Vision API / GPT-4o (cloud, swappable to local PaliGemma 3B for edge)
```

with:

```
- **VLM Backbone:** Moondream2 (~1.8B, runs locally on the Jetson Orin Nano)
```

- [ ] **Step 2: Mark the roadmap item done**

In `README.md`, in the "Roadmap" section, replace:

```
- [ ] Phase 2: Vision tool with cloud VLM integration
```

with:

```
- [x] Phase 2: Vision tool — `capture_and_analyze` with local Moondream2
```

- [ ] **Step 3: Mark the Build Phases entry done**

In `README.md`, in the "Build Phases" section, change the "Phase 2 — Vision Tool" heading to note completion by replacing:

```
### Phase 2 — Vision Tool
Write the `look()` tool. Capture camera frame, send to cloud VLM, parse spatial response (direction + rough distance). Test by manually pointing the camera at objects.
```

with:

```
### Phase 2 — Vision Tool ✅
`capture_and_analyze(target)` captures a camera frame, asks a local Moondream2 VLM where the target is and how far away it is, and returns a structured `SceneObservation`. Distance uses the depth camera with a VLM fallback. Verified via `scripts/test_vision.py`.
```

- [ ] **Step 4: Verify the README still renders**

Run: `python -c "import pathlib; t = pathlib.Path('README.md').read_text(); assert 'Moondream2' in t and 'capture_and_analyze' in t; print('readme ok')"`
Expected: prints `readme ok`

- [ ] **Step 5: Commit**

```bash
git add README.md
git commit -m "docs: mark Phase 2 vision tool complete with Moondream2"
```

---

## Final verification

- [ ] **Run the full pure-logic test suite**

Run: `pytest -v`
Expected: PASS — all tests in `tests/test_scene_parsing.py`, `tests/test_depth_math.py`, `tests/test_vision_tool_integration.py`, plus the pre-existing safety-controller tests.

- [ ] **Confirm hardware verification is recorded**

The model/ROS code (Tasks 9, 11, 16) has no laptop test by design. Confirm the static-image run (Task 9), the live-camera run (Task 11), and the depth run (Task 16) were each performed and their observed-vs-expected results recorded.
