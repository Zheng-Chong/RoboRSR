"""Write manifest.json: every delivered file with sha256, plus per-asset physical/collision metadata."""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
USD = HERE / "usd"

CONFIDENCE = {
    "table": "high: outline from multi-view back-projection, height from cup-base layer (scale-dependent)",
    "blue_cup_a": "position high (+-2 cm), height high (+-1 cm), diameters estimated",
    "blue_cup_b": "position high (+-2 cm), height high (+-1 cm), diameters estimated",
    "paper_cup": "position high (+-2 cm), height high (+-1 cm), diameters estimated",
    "tissue_pack": "position/yaw medium (front edge measured in 3 views), depth estimated, soft pack modeled as box",
    "plush_toy": "position high, height high, shape approximated by ellipsoids, yaw estimated",
}
COLLISION = {
    "cup": "12 convex wall segments + bottom disk (open top)",
    "tissue": "single box (tissue puff not collidable)",
    "toy": "convex hull of body+head",
}


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


params = json.loads((USD / "build_params.json").read_text(encoding="utf-8"))
files = sorted(p for p in USD.rglob("*") if p.is_file())
assets = {
    "table": dict(file="table.usda", prim="/Table", type="static collider", dims_m=params["table"],
                  collision="top box (rounded corners ignored), 2 columns, 2 feet", confidence=CONFIDENCE["table"]),
}
for n, s in params["objects"].items():
    assets[n] = dict(file=f"objects/{n}.usda", prim=f"/{n}", type="rigid body", pose_on_table=dict(xy=s["pos"], yaw_deg=s["yaw"]),
                     mass_kg=s["mass"], friction_static_dynamic=s["friction"], collision=COLLISION[s["kind"]],
                     mass_and_friction="estimated (not measured)", confidence=CONFIDENCE[n],
                     dims_m={k: s[k] for k in ("height", "r_top", "r_bot", "wall", "size") if k in s})
manifest = dict(
    package="real_scene1 office scene", frame=params["frame"], scale=dict(
        meters_per_sfm_unit=params["meters_per_sfm_unit"], correction=params["scale_correction"],
        basis="table length 1.20 m, desk height 0.75 m, 0.5 m carpet tiles (all agree; no direct measurement yet)"),
    entry_points=dict(full_scene="usd/scene.usda", isaaclab_cfg="isaac/real_scene1_scene_cfg.py"),
    assets=assets,
    background=dict(file="background.usda", prim="/Background", fidelity="coarse",
                    colliders="/Background/Colliders/* (guide purpose, box/convex approximations)",
                    layout_sources={k: v.get("src", "") for k, v in params["layout_unscaled"].items() if isinstance(v, dict)}),
    lights=dict(file="lights.usda", sun_direction_to_sun=params["sun_direction_to_sun"],
                note="intensities are uncalibrated starting values; ceiling panels unobserved and set to 0"),
    files={str(p.relative_to(HERE)).replace("\\", "/"): dict(bytes=p.stat().st_size, sha256=sha(p)) for p in files},
)
(HERE / "manifest.json").write_text(json.dumps(manifest, indent=1, ensure_ascii=False), encoding="utf-8")
print("manifest:", len(files), "files")
