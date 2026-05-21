# Vision Tool — `capture_and_analyze` — Design Spec

> **Status:** approved design, ready for implementation planning.
> **Date:** 2026-05-21

## Goal

Build the RoverMind agent's vision tool: `capture_and_analyze(target)`. It grabs
one frame from the LIMO Pro's camera, asks a local Vision-Language Model where a
named target object is and how far away it is, and returns that as structured
data the LangGraph agent can act on.

This is **Phase 2 (Vision Tool)** of the project roadmap. It builds *only* the
tool — not the LangGraph agent (Phase 3) and not the `move` / `stop_and_report`
tools. The tool must be independently runnable so it can be verified by pointing
the camera at known objects.

## Context

The repo currently contains the safety/control stack only:

- `safety_controller_layer/` — `ExecuteCommand` action server + autonomous
  emergency braking velocity gate.
- `safety_controller_layer_interfaces/` — ROS2 action interface package.

No perception or agent code exists yet. The README's `limo_vlm_agent/` layout is
aspirational; the actual repo is the safety stack plus this new tool.

The codebase establishes a pattern this feature follows: **pure decision logic
with zero ROS imports** (`control_math.py`, `aeb_math.py` — fully unit-testable
on the dev laptop, where `rclpy` is not installable) plus **thin wrapper code**
for the ROS / hardware side, which is verified manually on the rover.

## VLM choice — Moondream2 (research outcome)

The original idea named **Florence-2-base**. Research found Florence-2 is a
**detection/grounding** model driven by fixed task tokens (`<OD>`,
`<OPEN_VOCABULARY_DETECTION>`, `<CAPTION_TO_PHRASE_GROUNDING>`, …); the released
weights **do not do free-form VQA**. It cannot answer a natural-language prompt
like "where is the bottle and should the rover stop". It would only fit a
detector-style design (bounding box → spatial logic in code).

The chosen approach instead uses **Moondream2** (`vikhyatk/moondream2`, ~1.8B), a
small conversational VLM that *does* answer free-form questions:

- Runs **fully local** on the LIMO Pro's Jetson Orin Nano 8GB — no network, no
  API key, comfortable memory headroom (~2–4 GB).
- Genuinely conversational, so it can reason about the scene rather than only
  locate objects.
- Inference is ~1.5–3 s/frame on the Orin. The hybrid architecture runs
  perception at ~1 Hz with the controller driving smoothly in between, so this
  latency is well within budget.

Other Orin-viable options were weighed and rejected for this phase: Florence-2 /
YOLO-World (detectors, no reasoning); Qwen2.5-VL-3B / PaliGemma-3B (better
reasoning but ~3B sits at the Orin Nano 8GB ceiling, ~4–8 s/frame); cloud
Claude / GPT-4o (best reasoning but needs Wi-Fi + an API key).

## Locked-in design decisions (from clarifying Q&A 2026-05-21)

- **VLM:** Moondream2, local on the Orin Nano, loaded at a **pinned model
  revision** (its `transformers` API has changed across releases; the current
  release is `2025-06-21`, which exposes `model.query(image, question)`).
- **Frame source:** a short-lived `rclpy` subscription to the camera topic
  `/camera/color/image_raw` (`sensor_msgs/Image`), converted with `cv_bridge`.
  Consistent with the rest of the ROS2 stack; works while the LIMO camera driver
  is running. The capture path needs `rclpy`, so it is hardware-only —
  untestable on the dev laptop, like the other ROS code in this repo.
- **Extraction strategy:** **one narrow question per field**. Moondream2 is
  small; a single broad prompt or a JSON-emitting prompt parses unreliably.
  Three constrained single-choice questions give a robust keyword parse. The
  not-visible answer short-circuits the remaining two questions.
- **`should_stop`:** **derived in code** (`found and distance == "close"`), not
  asked of the VLM. The VLM does perception only; the lidar AEB gate remains the
  authoritative safety mechanism.
- **Depth camera:** included, but as the **final phase** — it upgrades the
  distance field from a VLM guess to a metric reading without blocking a working
  tool. See Phase 5.
- **Package:** a **new `perception/` package** at the repo root (the user
  confirmed a new package over folding into `safety_controller_layer` — vision
  is a distinct domain from safety/control).
- **Test strategy:** pure logic gets `pytest` TDD on the laptop; model and
  `rclpy` code get manual hardware verification, consistent with
  `safety_controller_node.py` and `aeb_node.py`.

## Architecture

```
agent (Phase 3, future)
   │  capture_and_analyze("water bottle")
   ▼
┌──────────────────────────────────────────────┐
│  perception/vision_tool.py                    │
│  capture_and_analyze(target, capture_fn,      │
│                      moondream)               │
│   1. image = capture_fn()                     │
│   2. ask "visible?" ─ if no, return early     │
│   3. ask "left / center / right?"             │
│   4. ask "close / medium / far?"              │
│   5. build_observation(...) ─→ SceneObservation│
└────────┬─────────────────────┬────────────────┘
         │ capture_fn          │ moondream
         ▼                     ▼
  rclpy subscription     perception/moondream_client.py
  /camera/color/image_raw   Moondream2 on the Orin GPU
         │                     │
         └──── raw answers ────┴──→ perception/scene_parsing.py
                                    (pure: text → SceneObservation)
```

`capture_fn` and `moondream` are **injected** into `capture_and_analyze` (the
same dependency-injection style as `SafetyController`, which takes `get_yaw` /
`publish_twist` callbacks). This makes the whole orchestration laptop-testable
with fakes, even though the real capture and real model are not.

## File structure

**Create:**

- `perception/__init__.py`
- `perception/scene_parsing.py` — pure logic, zero ROS / ML imports. The
  `SceneObservation` dataclass, the answer parsers, `build_observation`.
- `perception/moondream_client.py` — Moondream2 wrapper. Heavy imports
  (`torch`, `transformers`); hardware/GPU-only.
- `perception/vision_tool.py` — `capture_and_analyze` orchestration plus the
  real `rclpy` `capture_fn`.
- `tests/test_scene_parsing.py` — pure unit tests.
- `tests/test_vision_tool_integration.py` — orchestration tests with fakes.
- `scripts/test_vision.py` — manual verification entry point (README Phase 2
  references this script).
- `requirements.txt` — `torch`, `transformers`, `pillow` (the README references
  a `requirements.txt` that does not exist yet).

**Modify:**

- `perception/package.xml` *(new)* — declare the `cv_bridge` and `sensor_msgs`
  ROS dependencies.
- `README.md` — mark Phase 2 progress; correct the VLM backbone from
  "Claude Vision / GPT-4o" to "Moondream2 (local)".

## Data model — `SceneObservation`

A frozen dataclass, consistent with `AebParams` / `ControllerParams`:

| Field | Type | Meaning |
|---|---|---|
| `target` | `str` | What we looked for, e.g. `"water bottle"`. |
| `found` | `bool` | Whether the target is visible in the frame. |
| `direction` | `Optional[Literal["left","center","right"]]` | Side of the frame; `None` if not found or unparseable. |
| `distance` | `Optional[Literal["close","medium","far"]]` | Rough distance bucket; `None` if not found or unparseable. |
| `should_stop` | `bool` | Derived: `found and distance == "close"`. |
| `raw_answers` | `dict[str, str]` | The raw Moondream2 answers, keyed by field, for logs / debugging. |

Phase 5 adds two optional fields with defaults (non-breaking on a frozen
dataclass): `distance_m: Optional[float] = None` and
`distance_source: Literal["depth","vlm"] = "vlm"`.

## Pure-logic core (`scene_parsing.py`)

All functions are pure and unit-testable without `rclpy` or `torch`.

### `parse_yes_no(answer: str) -> bool`

Maps the visibility answer to a boolean. Lenient: finds a `yes` / `no` token
even when embedded in a sentence ("Yes, there is a bottle."). An ambiguous or
empty answer returns `False` — failing toward "not found" is safe, because the
agent then keeps searching rather than acting on a phantom target.

### `parse_direction(answer: str) -> Optional[str]`

Maps to `"left"` / `"center"` / `"right"`. Accepts synonyms (`"middle"`,
`"centre"` → center). An ambiguous answer returns `None`.

### `parse_distance(answer: str) -> Optional[str]`

Maps to `"close"` / `"medium"` / `"far"`. Accepts synonyms (`"near"` → close;
`"moderate"` → medium; `"distant"` → far). An ambiguous answer returns `None`.

### `build_observation(target, visible_answer, direction_answer, distance_answer) -> SceneObservation`

Runs the three parsers, derives `should_stop = found and distance == "close"`,
and assembles the `SceneObservation`. When `found` is `False`, `direction` and
`distance` are forced to `None` and `should_stop` to `False`.

## Moondream2 client (`moondream_client.py`)

`MoondreamClient`:

- Loads `vikhyatk/moondream2` once at construction via
  `AutoModelForCausalLM.from_pretrained(..., revision=<pinned>,
  trust_remote_code=True)`, onto the Orin GPU. Loading is slow (several
  seconds), so the agent constructs the client once at startup and reuses it.
- `ask(image, question: str) -> str` — wraps `model.query(image, question)` and
  returns the answer string.
- `point(image, target: str) -> Optional[tuple[float, float]]` — *(Phase 5)*
  wraps Moondream2's `point` capability, returning the target's normalised
  `(x, y)` pixel location, or `None` if the target is not located.

Heavy imports; no laptop unit test, consistent with the repo's stance on
hardware-only code.

## Orchestration + capture (`vision_tool.py`)

### `capture_and_analyze(target, *, capture_fn, moondream) -> SceneObservation`

1. `image = capture_fn()` — may raise `FrameCaptureError`.
2. Ask *"Is there a {target} in this image? Answer yes or no."* Parse with
   `parse_yes_no`. If not visible, return `build_observation` immediately with
   only the visibility answer — **skipping the remaining two VLM calls**
   (~4 s saved).
3. Ask *"Is the {target} on the left, in the center, or on the right of the
   image?"*
4. Ask *"Is the {target} close, at a medium distance, or far away?"*
5. Return `build_observation(target, visible, direction, distance)`.

### `ros_capture_fn(topic, timeout_s) -> PIL.Image`

The real `capture_fn`: `rclpy.init()`, create a node, subscribe to
`/camera/color/image_raw` (`sensor_msgs/Image`), spin until one message arrives
or `timeout_s` elapses, convert with `cv_bridge` (BGR ndarray → PIL RGB), then
tear the node down. Raises `FrameCaptureError` on timeout. Hardware-only.

## Error handling

- **Frame capture timeout** → raise `FrameCaptureError`. This is distinct from
  "target not found" so the agent can tell a broken camera apart from an absent
  object.
- **Moondream2 inference error** → propagates to the caller.
- **Target not visible** → a normal `found=False` result, not an error.
- **Unparseable answers** → graceful degradation: visibility ambiguous →
  `False`; direction / distance ambiguous → `None`. The observation is still
  returned; the agent decides what to do with partial information.

## Build phases

Each phase is a vertical slice — one capability end-to-end — and leaves the tool
in a working, demonstrable state. TDD: red → green → commit per unit.

### Phase 1 — Parsed answers → `SceneObservation`

`SceneObservation` + `scene_parsing.py`. Capability: given three raw VLM answer
strings, produce correct structured data. Fully laptop-testable.
**Verification:** `tests/test_scene_parsing.py` (`pytest`), literal red → green.

### Phase 2 — Full tool orchestration (with fakes)

`vision_tool.py` `capture_and_analyze` plus the injection seams. Capability: the
whole tool flow — including the not-visible early return — minus real hardware.
**Verification:** `tests/test_vision_tool_integration.py` with a fake
`capture_fn` (returns a dummy image) and a fake Moondream (scripted answers);
`pytest`, literal red → green. Asserts the not-visible path skips the
direction / distance calls, and a found+close path yields `should_stop=True`.

### Phase 3 — Real Moondream2 on a still image

`moondream_client.py`. Capability: real VLM perception on a saved image.
**Verification:** `scripts/test_vision.py --image <file>` on a GPU box prints a
real `SceneObservation`. Expected-output criteria are written before the code;
no laptop `pytest` (heavy model), consistent with the repo convention.

### Phase 4 — Live camera capture on the rover

`ros_capture_fn` — the real `rclpy` subscription. Capability: the complete tool,
live, end-to-end.
**Verification:** `scripts/test_vision.py` (no `--image`) on the rover grabs a
live frame off `/camera/color/image_raw` and prints the `SceneObservation` for
objects placed at known positions. Manual hardware verification.

### Phase 5 — Metric distance from the depth camera

Adds `depth_to_distance_bucket` (pure) and depth wiring. Capability: distance
upgraded from a VLM guess to a metric reading. Flow: `MoondreamClient.point`
locates the target → sample the aligned depth image at that pixel (median over a
small patch, rejecting zero/invalid depth holes) → `depth_to_distance_bucket`
converts metres to `close` / `medium` / `far` using thresholds tied to the
controller's `stop_distance` (0.4 m). If the depth patch is all-invalid, fall
back to the VLM distance answer (`distance_source` records which was used).
**Verification:** `depth_to_distance_bucket` — `pytest`, literal red → green;
the `point` + depth-sample wiring — manual hardware verification on the rover.

## Testing summary

| Test | Type | Runs on |
|---|---|---|
| `tests/test_scene_parsing.py` | pure unit (`pytest`) | dev laptop |
| `tests/test_vision_tool_integration.py` | orchestration w/ fakes (`pytest`) | dev laptop |
| `depth_to_distance_bucket` tests (Phase 5) | pure unit (`pytest`) | dev laptop |
| `scripts/test_vision.py --image` | manual, real model | GPU box / Orin |
| `scripts/test_vision.py` (live) | manual, real model + camera | rover |

`scene_parsing.py` parsers: each tested with clear cases, synonyms,
answer-embedded-in-a-sentence, and ambiguous input hitting the default.
`build_observation`: `should_stop` true only when found+close; a not-found
result zeroes `direction` / `distance`.

## Out of scope (YAGNI)

- **The LangGraph agent** (Phase 3) and the `move` / `stop_and_report` tools.
- **4-bit quantization** of Moondream2 — a later latency-tuning step, not core.
- **Prompt tuning** — Phase 4 of the project roadmap; the three starting prompts
  are specified above.
- **Multiple instances of the target** — assume one; Moondream2 describes the
  most prominent. Disambiguating several is a later concern.
- **A combined launch file** bringing up the camera driver, controller, AEB, and
  agent together — a deployment concern, not part of this tool.
