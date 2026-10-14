"""Build USD variants required by the 50-episode G1-D checklist."""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path

import numpy as np

SCRIPTS = Path(__file__).resolve().parent
PACK = SCRIPTS.parent
PROJECT = PACK.parent
sys.path.insert(0, str(SCRIPTS))

import build_assets
from build_assets import cup, table
from geometry_utils import export_asset

CHECKLIST_001_050 = PROJECT / "scean3_50episode"
CHECKLIST_051_100 = PROJECT / "scean3_50-100episode"


def _config_dir(eid):
    return CHECKLIST_051_100 if int(eid[1:]) >= 51 else CHECKLIST_001_050


def _load_config(eid):
    path = _config_dir(eid) / "checklist_episodes.py"
    spec = importlib.util.spec_from_file_location(f"checklist_episodes_{eid.lower()}", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Cannot load checklist config: {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def frustum_volume(r0, r1, h):
    return math.pi * h / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1)


def ceramic_volume(c):
    inner_h = c["height"] - c["bottom"]
    return frustum_volume(c["radius_bottom"], c["radius_top"], c["height"]) - frustum_volume(
        c["radius_bottom"] - c["wall"], c["radius_top"] - c["wall"], inner_h
    )


def water_capacity(c, spacing, ml):
    count = int(round(ml * 1e-6 / spacing**3))
    points = []
    for z in np.arange(c["bottom"] + spacing, c["height"] - 2 * spacing, spacing):
        r = c["radius_bottom"] + (c["radius_top"] - c["radius_bottom"]) * z / c["height"] - c["wall"] - spacing
        layer = []
        for x in np.arange(-c["radius_top"], c["radius_top"] + spacing, spacing):
            for y in np.arange(-c["radius_top"], c["radius_top"] + spacing, spacing):
                if x * x + y * y < r * r:
                    layer.append([x, y, z])
        layer.sort(key=lambda v: v[0] * v[0] + v[1] * v[1])
        points.extend(layer)
        if len(points) >= count:
            break
    if len(points) < count:
        raise ValueError(f"Water volume {ml} mL exceeds conservative capacity for {c['label']}")
    return np.asarray(points[:count])


def _material_tuple(name, fallback, materials):
    if name == "baseline":
        return build_assets.MATERIALS[fallback]
    material = materials[name]
    return (material["color"], material["metallic"], material["roughness"], material["opacity"])


def _with_materials(overrides):
    old = {key: build_assets.MATERIALS.get(key) for key in overrides}
    build_assets.MATERIALS.update(overrides)
    return old


def _restore_materials(old):
    for key, value in old.items():
        if value is None:
            build_assets.MATERIALS.pop(key, None)
        else:
            build_assets.MATERIALS[key] = value


def build_cup_variant(kind, spec, material_name, friction, out, materials):
    overrides = {}
    body = _material_tuple(material_name, "enamel", materials)
    overrides["enamel"] = body
    overrides["metal_inner"] = body
    overrides["metal_edge"] = body
    old = _with_materials(overrides)
    try:
        asset = cup(kind, spec)
        if friction is not None:
            asset.friction = friction
        out.parent.mkdir(parents=True, exist_ok=True)
        export_asset(asset, out)
    finally:
        _restore_materials(old)
    water = water_capacity(spec, spec["particle_spacing_m"], spec["water_ml"])
    grasp_z = spec["grasp_height_m"]
    grasp_r = spec["radius_bottom"] + (spec["radius_top"] - spec["radius_bottom"]) * grasp_z / spec["height"]
    provenance = {
        "generator": str(Path(__file__).resolve()),
        "kind": kind,
        "usd": str(out),
        "usd_sha256": hashlib.sha256(out.read_bytes()).hexdigest(),
        "material": material_name,
        "friction": asset.friction,
        "geometry": {k: spec[k] for k in ["height", "radius_bottom", "radius_top", "wall", "bottom"]},
        "mass_kg": spec["mass"],
        "ceramic_volume_m3": ceramic_volume(spec),
        "water_ml_checked": spec["water_ml"],
        "water_particle_count": int(len(water)),
        "water_fill_max_z_m": float(water[:, 2].max()) if len(water) else 0.0,
        "grasp_height_m": grasp_z,
        "grasp_diameter_m": float(2 * grasp_r),
        "open_cavity": True,
    }
    out.with_suffix(".provenance.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8")
    return provenance


def build_table_variant(material_name, out, materials):
    material = _material_tuple(material_name, "table_white", materials)
    old = _with_materials({"table_white": material, "carpet": material})
    try:
        spec = json.loads((PACK / "scene_spec.json").read_text(encoding="utf-8"))
        asset = table(spec)
        out.parent.mkdir(parents=True, exist_ok=True)
        export_asset(asset, out)
    finally:
        _restore_materials(old)
    provenance = {
        "generator": str(Path(__file__).resolve()),
        "kind": "table",
        "usd": str(out),
        "usd_sha256": hashlib.sha256(out.read_bytes()).hexdigest(),
        "material": material_name,
    }
    out.with_suffix(".provenance.json").write_text(json.dumps(provenance, indent=2), encoding="utf-8")
    return provenance


def build_for_episode(eid):
    config = _load_config(eid)
    item = config.effective_spec(eid)
    assets = config.variant_asset_names(item)
    manifest = {"episode": eid, "assets": {}}
    if assets.get("source_cup"):
        spec = dict(item["source_cup"])
        spec["water_ml"] = item["water_ml"]
        spec["particle_spacing_m"] = 0.003
        spec["grasp_height_m"] = item["selected_grasp"]["grasp_height_m"]
        out = PACK / "assets/objects" / assets["source_cup"]
        manifest["assets"]["source_cup"] = build_cup_variant(
            "source_cup", spec, item["source_material"], item["source_friction"], out, config.MATERIALS
        )
    if assets.get("receiver_cup"):
        spec = dict(item["receiver_cup"])
        spec["water_ml"] = item["water_ml"]
        spec["particle_spacing_m"] = 0.003
        spec["grasp_height_m"] = spec["height"] * 0.5
        out = PACK / "assets/objects" / assets["receiver_cup"]
        manifest["assets"]["receiver_cup"] = build_cup_variant(
            "receiver_cup", spec, item["receiver_material"], None, out, config.MATERIALS
        )
    if assets.get("table"):
        out = PACK / "assets" / assets["table"]
        manifest["assets"]["table"] = build_table_variant(item["table"], out, config.MATERIALS)
    return manifest


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--eid", action="append")
    parser.add_argument("--all", action="store_true")
    args = parser.parse_args()
    if args.all:
        eids = [f"E{i:03d}" for i in range(1, 101)]
    else:
        eids = args.eid
    if not eids:
        parser.error("provide --eid or --all")
    manifests = [build_for_episode(eid) for eid in eids]
    out = PACK / "assets/checklist_asset_manifest.json"
    out.write_text(json.dumps(manifests, indent=2, ensure_ascii=False), encoding="utf-8")
    print(json.dumps({"built": eids, "manifest": str(out)}, indent=2))


if __name__ == "__main__":
    main()
