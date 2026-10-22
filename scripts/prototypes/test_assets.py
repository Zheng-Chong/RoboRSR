"""CPU regression tests for optional prototypes; no Isaac/robot/CAD download."""

import hashlib
import json
from pathlib import Path
import struct
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
from import_aurora_lite import export_visual, verify_source
from textile_fixture import ClothConfig, build
from usd_geometry import write_mesh


def test_stl_units_and_visual_only(tmp_path):
    stl = tmp_path / "triangle.stl"
    stl.write_bytes(b"\0" * 80 + struct.pack("<I", 1) + struct.pack("<12fH", 0, 0, 1, 0, 0, 0, 100, 0, 0, 0, 50, 0, 0))
    report = export_visual(stl, tmp_path)
    assert report["bounds_m"] == [(0.0, 0.0, 0.0), (0.1, 0.05, 0.0)]
    assert not report["articulated"] and not report["collision_enabled"] and not report["tactile_simulated"]
    assert 'contype="0" conaffinity="0"' in (tmp_path / "reference.xml").read_text()
    # Actual USD parse is optional for contributors without OpenUSD; CI-independent.
    pxr = pytest.importorskip("pxr.Usd")
    stage = pxr.Stage.Open(str(tmp_path / "geometry.usda"))
    assert stage.GetDefaultPrim().IsValid()
    assert not stage.GetDefaultPrim().GetAppliedSchemas()


def test_pin_rejects_changed_cad(tmp_path):
    source = tmp_path / "input.step"
    data = b"SI_UNIT ( .MILLI., .METRE. );\nEND-ISO-10303-21;"
    source.write_bytes(data)
    spec = dict(source_bytes=len(data), source_sha256=hashlib.sha256(data).hexdigest())
    verify_source(source, spec)
    source.write_bytes(data.replace(b"MILLI", b"CENTI"))
    with pytest.raises(ValueError, match="pinned"):
        verify_source(source, spec)


def test_cloth_mass_topology_and_no_false_material_claims():
    xml, meta = build(ClothConfig(width_m=0.2, length_m=0.1, areal_density_kg_m2=0.15))
    assert meta["mass_kg"] == pytest.approx(0.003)
    assert not meta["real_material_calibrated"] and not meta["robot_integrated"]
    assert 'dim="2"' in xml and 'equality="true"' in xml
    with pytest.raises(ValueError):
        build(ClothConfig(nx=1))
    with pytest.raises(ValueError):
        build(ClothConfig(thickness_m=0.1))


def test_usd_rejects_bad_topology_and_nonfinite_points(tmp_path):
    for points, faces in [([(float("nan"), 0, 0)], [(0, 0, 0)]), ([(0, 0, 0)], [(0, 1, 2)])]:
        with pytest.raises(ValueError):
            write_mesh(tmp_path / "invalid.usda", points, faces)


def test_cloth_cli_usd_and_no_overwrite(tmp_path):
    import subprocess

    script = Path(__file__).with_name("textile_fixture.py")
    out = tmp_path / "cloth"
    subprocess.run([sys.executable, str(script), "--output", str(out)], check=True)
    meta = json.loads((out / "asset.json").read_text())
    assert meta["model_sha256"] == hashlib.sha256((out / "model.xml").read_bytes()).hexdigest()
    instance = json.loads((out / "isaac_instance.json").read_text())
    assert instance["physics"]["garment_config"]["particle_mass"] * 81 == pytest.approx(meta["mass_kg"])
    assert not instance["calibration"]["mujoco_material_equivalence_verified"]
    original = (out / "model.xml").read_bytes()
    assert subprocess.run([sys.executable, str(script), "--output", str(out)], capture_output=True).returncode != 0
    assert (out / "model.xml").read_bytes() == original
    pxr = pytest.importorskip("pxr.Usd")
    stage = pxr.Stage.Open(str(out / "geometry.usda"))
    mesh = stage.GetPrimAtPath("/Prototype/geometry")
    assert len(mesh.GetAttribute("points").Get()) == 81
    assert len(mesh.GetAttribute("faceVertexCounts").Get()) == 128
