# perception/ — vision → structured observation (runs on the rover)

Turn a camera frame (+ optional depth) into a `SceneObservation` via the local Moondream2 VLM.
Runs **on the rover** behind the bridge (Moondream needs the Jetson GPU).

## Key files

- `vision_tool.py` — orchestration. `capture_and_analyze(target, capture_fn, moondream, depth_fn=None)`:
  capture frame → ask visibility → if found ask direction → optionally point + sample depth for a
  metric bucket → `build_observation`. Also the **hardware-only** ROS capture fns
  (`ros_capture_fn` `/camera/color/image_raw`, `ros_depth_capture_fn` `/camera/depth/image_raw`)
  and `FrameCaptureError`.
- `scene_parsing.py` — **pure.** `SceneObservation` (frozen dataclass) + parsers `parse_yes_no`,
  `parse_direction` (**"center" wins** — middle/center, even "slightly right of center", → center →
  drive forward; only a clear side → turn), `parse_distance`, and `build_observation`. Real parsing
  logic lives here; laptop-testable.
- `depth_math.py` — **pure.** `depth_to_distance_bucket` (close/medium/far from metres) and
  `sample_depth_patch` (mm grid → metres around a normalized point).
- `moondream_client.py` — `MoondreamClient(device)` wrapping Moondream2 (`vikhyatk/moondream2`,
  pinned `2025-06-21`). `.ask(image, q)`, `.point(image, target) → (x,y)|None`. Applies an
  SDPA/GQA shim for torch < 2.5; `resolve_model_source()` honors `MOONDREAM_MODEL_PATH` for a local
  snapshot. Lazy-imports transformers (hardware-only).

## How it connects

`bridge_server.py`'s `capture_and_analyze` calls `vision_tool.capture_and_analyze`; the result
crosses the bridge to the agent's `look` tool. Direction/distance buckets are the shared
vocabulary in `context/GLOSSARY.md`.

## Gotchas

- **Depth currently never populates** → `distance_m=None`, `distance_source="vlm"`, so distance
  relies on Moondream's verbal estimate (over-reports "close"). Known follow-up; see `context/ERRORS.md`.
- Camera is a **separate launch** from `limo_start` (Orbbec Dabai DCW2).
- First Moondream call loads the model (~25–40 s), then ~16 s/call (three sequential questions) —
  steady-state, not a leak.

## Tests

`tests/test_scene_parsing.py`, `test_depth_math.py`, `test_observation_formatter.py`,
`test_moondream_source.py`, `test_sdpa_gqa_shim.py`, `test_vision_tool_integration.py`.
