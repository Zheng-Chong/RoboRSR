"""Final V1 safety contracts and preservation of the native camera front/lens envelope."""

from hashlib import sha256
import json
import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from build import build
from geometry import geometry_parts, mount_profile, split_wrist_camera


def test_only_final_geometry_profile_is_accepted(tmp_path):
    profile = mount_profile()
    for changes in ({"focal_length": 477}, {"solid_camera": None}, {"extra_rpy_deg": [5, 0, 0]}):
        p = tmp_path / "profile.json"
        p.write_text(json.dumps(profile | changes))
        with pytest.raises(ValueError):
            mount_profile(p)


def test_invalid_collision_band_is_rejected(tmp_path):
    profile = mount_profile()
    for band in (-0.001, 0.03, float("nan")):
        p = tmp_path / "profile.json"
        p.write_text(json.dumps(profile | {"support_collision_band_height_m": band}))
        with pytest.raises(ValueError, match="support_collision_band_height"):
            mount_profile(p)


def test_existing_outputs_are_preserved(tmp_path):
    sentinel = tmp_path / "keep.txt"
    sentinel.write_text("existing work")
    with pytest.raises(ValueError, match="new independent directory"):
        build(tmp_path / "missing.urdf", tmp_path / "missing.STL", tmp_path / "missing.json", tmp_path)
    assert sentinel.read_text() == "existing work"


def test_final_solids_keep_native_lens_envelope_and_source():
    default = Path(__file__).resolve().parents[4] / "real2sim/robot/g1d_v4/meshes/dex1_1/base_link.STL"
    source = Path(os.environ.get("DEX1_NATIVE_MESH", default))
    if not source.is_file():
        pytest.skip("Set DEX1_NATIVE_MESH to the original official base_link.STL")
    digest = sha256(source.read_bytes()).hexdigest()
    profile = mount_profile()
    _, _, native, _ = split_wrist_camera(source)
    native.apply_translation([0, profile["forward_shift_m"], profile["seat_height_m"] - native.bounds[0, 2]])
    parts, _ = geometry_parts(source, profile)
    housing, upright = parts["housing"][0], parts["support"][1]
    assert housing.is_volume and upright.is_volume
    assert housing.volume > native.volume * 1.1
    assert upright.extents[2] > 0.015
    normal = np.array([0, np.cos(np.deg2rad(35)), -np.sin(np.deg2rad(35))])
    # Added rear case must not move the native front/lens or widen the native housing.
    assert np.isclose((housing.vertices @ normal).max(), (native.vertices @ normal).max(), atol=1e-7)
    assert np.allclose(housing.bounds[:, 0], native.bounds[:, 0], atol=1e-7)
    assert sha256(source.read_bytes()).hexdigest() == digest
