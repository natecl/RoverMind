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


def _sdpa_gqa_compat(orig, query, key, value, *args, enable_gqa=False, **kwargs):
    """Emulate ``scaled_dot_product_attention(enable_gqa=...)`` for old torch.

    The ``enable_gqa`` flag (added in torch 2.5) lets SDPA broadcast fewer key/value
    heads across more query heads (grouped-query attention). The Jetson's CUDA torch
    is 2.1, which lacks it, so we expand the K/V head dim (-3) to match the query
    heads -- matching torch's semantics, where query head ``h`` attends to kv head
    ``h // (Hq // Hkv)`` -- and drop the kwarg before calling the real ``orig``.
    """
    if enable_gqa:
        hq = query.shape[-3]
        hk = key.shape[-3]
        if hk and hq != hk:
            n_rep = hq // hk
            key = key.repeat_interleave(n_rep, dim=-3)
            value = value.repeat_interleave(n_rep, dim=-3)
    return orig(query, key, value, *args, **kwargs)


def _install_sdpa_gqa_shim():
    """Patch ``F.scaled_dot_product_attention`` for ``enable_gqa`` on torch < 2.5.

    Moondream2's remote code passes ``enable_gqa`` (in text.py); the Jetson's torch
    2.1 rejects it with ``TypeError``. Patching the functional in place covers every
    call site. No-op on torch >= 2.5 (native support) and idempotent.
    """
    import functools
    import torch
    import torch.nn.functional as F

    orig = getattr(F, "scaled_dot_product_attention", None)
    if orig is None or getattr(orig, "_gqa_shimmed", False):
        return
    version = tuple(int(p) for p in torch.__version__.split("+")[0].split(".")[:2])
    if version >= (2, 5):
        return

    @functools.wraps(orig)
    def shimmed(query, key, value, *args, enable_gqa=False, **kwargs):
        return _sdpa_gqa_compat(orig, query, key, value, *args, enable_gqa=enable_gqa, **kwargs)

    shimmed._gqa_shimmed = True
    F.scaled_dot_product_attention = shimmed


class MoondreamClient:
    """Loads Moondream2 once and answers questions about images."""

    def __init__(self, device: str = "cuda"):
        from transformers import AutoModelForCausalLM

        _install_sdpa_gqa_shim()
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
