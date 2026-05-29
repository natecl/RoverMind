"""Moondream2 wrapper. Heavy imports (torch, transformers) are deferred to
construction so this module can be imported on the dev laptop. Constructing a
MoondreamClient downloads/loads the model and requires a CUDA GPU.
"""

import os

MODEL_ID = "vikhyatk/moondream2"
# Pin the revision: Moondream2's transformers API has changed across releases.
# This release exposes model.query(image, question) and model.point(image, obj).
MODEL_REVISION = "2025-06-21"

# Moondream2's stock remote code uses py3.9+ builtin-generic annotations
# (e.g. `tuple[int, int]`) evaluated at import, which fails on the rover's
# Python 3.8. There, point this env var at a locally-vendored snapshot whose
# modeling files have been patched with `from __future__ import annotations`.
MODEL_PATH_ENV = "MOONDREAM_MODEL_PATH"


def resolve_model_source(env=None):
    """Return (model_ref, extra_kwargs) for AutoModelForCausalLM.from_pretrained.

    If MOONDREAM_MODEL_PATH names a local snapshot, load from it (no revision --
    a local dir is already pinned). Otherwise pull MODEL_ID at MODEL_REVISION.
    """
    env = os.environ if env is None else env
    local = env.get(MODEL_PATH_ENV)
    if local:
        return local, {}
    return MODEL_ID, {"revision": MODEL_REVISION}


class MoondreamClient:
    """Loads Moondream2 once and answers questions about images."""

    def __init__(self, device: str = "cuda"):
        from transformers import AutoModelForCausalLM

        model_ref, extra = resolve_model_source()
        self._model = AutoModelForCausalLM.from_pretrained(
            model_ref,
            trust_remote_code=True,
            device_map={"": device},
            **extra,
        )

    def ask(self, image, question: str) -> str:
        """Ask one free-form question about `image`; return the answer string."""
        result = self._model.query(image, question)
        return result["answer"]

    def point(self, image, target: str):
        """Locate `target`; return its normalised (x, y) in [0, 1], or None."""
        result = self._model.point(image, target)
        points = result.get("points", [])
        if not points:
            return None
        first = points[0]
        return (first["x"], first["y"])
