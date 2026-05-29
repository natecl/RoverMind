"""Unit tests for Moondream model-source resolution.

Pure logic -- no torch/transformers import -- so it runs on the dev laptop.
The rover's Python 3.8 can't import Moondream2's stock remote code (it uses
py3.9+ builtin-generic annotations), so on the rover we load a locally-vendored,
py3.8-patched snapshot pointed to by MOONDREAM_MODEL_PATH.
"""

from perception.moondream_client import (
    MODEL_ID,
    MODEL_REVISION,
    MODEL_PATH_ENV,
    resolve_model_source,
)


def test_defaults_to_hub_id_and_pinned_revision_when_env_unset():
    ref, extra = resolve_model_source(env={})
    assert ref == MODEL_ID
    assert extra == {"revision": MODEL_REVISION}


def test_uses_local_path_and_no_revision_when_env_set():
    ref, extra = resolve_model_source(env={MODEL_PATH_ENV: "/home/agilex/moondream2_local"})
    assert ref == "/home/agilex/moondream2_local"
    assert extra == {}


def test_blank_env_value_falls_back_to_hub():
    ref, extra = resolve_model_source(env={MODEL_PATH_ENV: ""})
    assert ref == MODEL_ID
    assert extra == {"revision": MODEL_REVISION}
