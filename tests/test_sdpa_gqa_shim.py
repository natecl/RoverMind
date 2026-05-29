"""Tests for the grouped-query-attention SDPA compat shim used to run Moondream2
on the rover's torch 2.1 (which lacks ``scaled_dot_product_attention(enable_gqa=)``)."""
import pytest

torch = pytest.importorskip("torch")
import torch.nn.functional as F  # noqa: E402

from perception.moondream_client import _sdpa_gqa_compat  # noqa: E402


def test_enable_gqa_stripped_and_kv_expanded():
    """The shim must not forward enable_gqa to the wrapped fn, and must expand
    K/V from Hk heads up to Hq heads (so old torch's SDPA sees matching shapes)."""
    captured = {}

    def fake_orig(q, k, v, *args, **kwargs):
        captured["kwargs"] = kwargs
        captured["k_heads"] = k.shape[-3]
        captured["v_heads"] = v.shape[-3]
        return q

    q = torch.randn(1, 8, 4, 16)  # (batch, Hq=8, seq, head_dim)
    k = torch.randn(1, 2, 4, 16)  # Hk=2
    v = torch.randn(1, 2, 4, 16)

    _sdpa_gqa_compat(fake_orig, q, k, v, attn_mask=None, enable_gqa=True)

    assert "enable_gqa" not in captured["kwargs"]  # stripped for old torch
    assert captured["k_heads"] == 8  # 2 -> 8
    assert captured["v_heads"] == 8


def test_emulation_matches_native_enable_gqa():
    """Emulated expansion must be numerically identical to torch's native
    enable_gqa (this test only runs where native support exists, i.e. torch>=2.5)."""
    version = tuple(int(p) for p in torch.__version__.split("+")[0].split(".")[:2])
    if version < (2, 5):
        pytest.skip("native enable_gqa requires torch >= 2.5")

    torch.manual_seed(0)
    q = torch.randn(2, 8, 5, 16)
    k = torch.randn(2, 2, 5, 16)
    v = torch.randn(2, 2, 5, 16)

    native = F.scaled_dot_product_attention(q, k, v, enable_gqa=True)
    emulated = _sdpa_gqa_compat(F.scaled_dot_product_attention, q, k, v, enable_gqa=True)

    assert torch.allclose(native, emulated, atol=1e-5)


def test_no_expansion_when_heads_equal():
    """Plain multi-head attention (Hq == Hk) must pass through unchanged."""
    captured = {}

    def fake_orig(q, k, v, *args, **kwargs):
        captured["k_heads"] = k.shape[-3]
        return q

    q = torch.randn(1, 4, 3, 8)
    k = torch.randn(1, 4, 3, 8)
    v = torch.randn(1, 4, 3, 8)

    _sdpa_gqa_compat(fake_orig, q, k, v, enable_gqa=False)
    assert captured["k_heads"] == 4  # untouched
