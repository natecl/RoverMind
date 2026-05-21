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

    def point(self, image, target: str):
        """Locate `target`; return its normalised (x, y) in [0, 1], or None."""
        result = self._model.point(image, target)
        points = result.get("points", [])
        if not points:
            return None
        first = points[0]
        return (first["x"], first["y"])
