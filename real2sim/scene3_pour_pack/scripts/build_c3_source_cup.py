"""Generate only the C3 low-wide source cup. Does not overwrite C1 assets."""
from __future__ import annotations

import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

SCRIPTS = Path(__file__).resolve().parent
PACK = SCRIPTS.parent
sys.path.insert(0, str(SCRIPTS))
from build_assets import cup
from geometry_utils import export_asset


def frustum_volume(r0, r1, h):
    return math.pi * h / 3.0 * (r0 * r0 + r0 * r1 + r1 * r1)


def ceramic_volume(c):
    inner_h = c["height"] - c["bottom"]
    return frustum_volume(c["radius_bottom"], c["radius_top"], c["height"]) - frustum_volume(
        c["radius_bottom"] - c["wall"], c["radius_top"] - c["wall"], inner_h
    )


def sample_water_points(c, ml, spacing):
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
        raise ValueError("Water volume exceeds conservative cup capacity.")
    return np.array(points[:count])


def main():
    proto = json.loads((PACK / "prototypes/c3l1_low_wide.json").read_text(encoding="utf-8"))
    cup_spec = proto["source_cup"]
    out = PACK / "assets/objects" / cup_spec["usd"]
    if (PACK / "assets/objects/source_cup.usdc").resolve() == out.resolve():
        raise RuntimeError("Refusing to overwrite the C1 source cup.")
    asset = cup("source_cup", cup_spec)
    out.parent.mkdir(parents=True, exist_ok=True)
    export_asset(asset, out)
    water = sample_water_points(cup_spec, proto["pour"]["water_ml"], proto["pour"]["particle_spacing_m"])
    grasp_z = proto["pour"]["grasp_height_m"]
    grasp_r = cup_spec["radius_bottom"] + (cup_spec["radius_top"] - cup_spec["radius_bottom"]) * grasp_z / cup_spec["height"]
    provenance = {
        "generator": str(Path(__file__).resolve()),
        "prototype": str(PACK / "prototypes/c3l1_low_wide.json"),
        "usd": str(out),
        "usd_sha256": hashlib.sha256(out.read_bytes()).hexdigest(),
        "visuals": len(asset.visuals),
        "colliders": len(asset.colliders),
        "ceramic_volume_m3": ceramic_volume(cup_spec),
        "mass_kg": cup_spec["mass"],
        "water_particle_count": int(len(water)),
        "water_fill_max_z_m": float(water[:, 2].max()),
        "grasp_diameter_m": float(2 * grasp_r),
        "open_cavity": True,
        "note": "Compound wall+bottom colliders; interior is not a solid convex hull.",
    }
    provenance_path = out.with_suffix(".provenance.json")
    provenance_path.write_text(json.dumps(provenance, indent=2), encoding="utf-8")
    print(json.dumps(provenance, indent=2))


if __name__ == "__main__":
    main()
