#!/usr/bin/env python3
"""Paper cups and paper plates of the school lab table (real_scene4) as RoboDojo `Rigid` assets.

- paper_cup/00000 white, 00001 black: each is two nested paper cups, one rigid body (as in the source twin). Profiles
  are the image fits of the colleague's MuJoCo twin (real2sim/scenes/real_scene4_school/bundle/cup_shapes.json): outer
  and inner wall, the lower cup's rolled rim and the upper cup's rim.
- paper_plate/00000 citrus, 00001 red, 00002 flower (named after what was on them): flat bottom and a conical rim,
  sizes from the twin's plate meshes.

Not registered on the Real2Sim platform and not tape-measured: sizes are fitted from the real camera frames by the twin,
masses come from its density hypothesis (288 kg/m^3 on the collision volume), colours are its flat sRGB RGBA.

Usage:
    python scripts/objects/make_school_tableware.py [RIGID_DIR]    # default Assets/Object/RoboDojo/Rigid
    python scripts/objects/check_object.py Assets/Object/RoboDojo/Rigid/{paper_cup/0000{0,1},paper_plate/0000{0,1,2}}
Writes RIGID_DIR/<category>/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py needs.

Frame: Z up, metres, origin at bbox centre.
USD:   /root (RigidBodyAPI) -> visual/model (whole object, one material)
                            -> collision/hull_NN (bottom slab + wall staves, hollow inside)
"""

import json
import os
from pathlib import Path
import sys
import traceback

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_pitcher as mp  # noqa: E402  (shared USD helpers; starts Kit when pxr needs it)
import make_plastic_cups as pc  # noqa: E402  (loft, wall staves, cavity check, save)

ROOT = Path(__file__).resolve().parents[2]
RIGID_DIR = ROOT / "Assets/Object/RoboDojo/Rigid"
CUP_SHAPES = json.loads((ROOT / "real2sim/scenes/real_scene4_school/bundle/cup_shapes.json").read_text())["shapes"]
SEC = pc.unit_section(2.0)
CUP_TB = 0.003  # bottom thickness: the fitted profiles start their tapered wall at 3 mm
# profile points that carry the collision wall: base, taper, lower rim, upper cup body, top; the rolled beads (~1.5 mm)
# are visual only
CUP_WALL_POINTS = (1, 2, 3, 4, 11, 12, 18)
# (category index, twin name, outside RGB, mass kg)
CUPS = [(0, "white_cup", (0.93, 0.92, 0.85), 0.0193), (1, "black_cup", (0.12, 0.10, 0.10), 0.0207)]
PLATE_T = 0.0015  # paper thickness; the twin's meshes are 1 mm apart, 1.5 mm keeps the PhysX slab sound
# (category index, twin name, rim radius, bottom radius, height, RGB, mass kg); radii and heights of the outer mesh
PLATES = [
    (0, "citrus_plate", 0.0802, 0.0544, 0.0160, (0.59, 0.54, 0.70), 0.0043),
    (1, "red_plate", 0.0687, 0.0466, 0.0137, (0.55, 0.55, 0.67), 0.0031),
    (2, "flower_plate", 0.0755, 0.0512, 0.0151, (0.77, 0.73, 0.60), 0.0038),
]


def build_cup(name):
    s = CUP_SHAPES[name]
    outer, inner = np.array(s["outer_profile_m"]), np.array(s["inner_profile_m"])
    keep = inner[:, 1] >= CUP_TB
    profile = [(0.0, 0.0)] + [tuple(p) for p in outer] + [tuple(p) for p in inner[keep][::-1]] + [(0.0, CUP_TB)]
    pts, faces = pc.loft(profile, SEC)
    wall = [(outer[i, 1], outer[i, 0] * SEC, inner[i, 0] * SEC) for i in CUP_WALL_POINTS]
    bottom = outer[0, 0] * SEC[:: pc.N_THETA // pc.BOTTOM_SIDES]
    hulls = [np.array([(*p, z) for p in bottom for z in (0.0, CUP_TB)])] + pc.wall_staves(wall)
    zs, rs = inner[list(CUP_WALL_POINTS), 1], inner[list(CUP_WALL_POINTS), 0]
    top = outer[-1, 1]
    c = dict(pts=pts, faces=faces, normals=mp.vertex_normals(pts, faces), hulls=hulls, contact_r=float(outer[0, 0]))
    c["probe"] = pc.cavity_probe(lambda z: np.interp(z, zs, rs) * SEC, CUP_TB, top)
    c["ml"] = pc.volume_ml(lambda z: np.interp(z, zs, rs) * SEC, CUP_TB, top)
    return c


def build_plate(R, rb, h):
    def inner_r(z):  # inner face of the conical rim, PLATE_T in from the outer one (horizontally)
        return rb + (R - rb) * z / h - PLATE_T

    profile = [(0.0, 0.0), (rb, 0.0), (R, h), (R - PLATE_T, h), (inner_r(PLATE_T), PLATE_T), (0.0, PLATE_T)]
    pts, faces = pc.loft(profile, SEC)
    bottom = rb * SEC[:: pc.N_THETA // pc.BOTTOM_SIDES]
    hulls = [np.array([(*p, z) for p in bottom for z in (0.0, PLATE_T)])]
    hulls += pc.wall_staves([(0.0, rb * SEC, inner_r(0.0) * SEC), (h, R * SEC, inner_r(h) * SEC)])
    c = dict(pts=pts, faces=faces, normals=mp.vertex_normals(pts, faces), hulls=hulls, contact_r=rb)
    c["probe"] = pc.cavity_probe(lambda z: inner_r(z) * SEC, PLATE_T, h)
    return c


def paper(srgb):
    rgb = tuple(c**2.2 for c in srgb)  # the twin's RGBA is display sRGB; OmniPBR takes linear colour

    def material(st):
        return mp.add_material(
            st,
            "/root/_materials/Paper",
            "OmniPBR",
            {"diffuse_color_constant": rgb, "reflection_roughness_constant": 0.7, "metallic_constant": 0.0},
            rgb,
            1.0,
            0.7,
        )

    return material


def cup_description(uid, colour, ml, H):
    caption = {
        "color": [colour, f"{colour} paper", "plain"],
        "material": ["paper", "coated paperboard", "cardboard"],
        "size": [f"holds about {ml:.0f} ml", f"about {H * 100:.1f} cm tall", "fits in one hand"],
        "name": ["paper cup", "disposable cup", "nested paper cups"],
        "shape": ["tapered cylinder", "rolled rim", "two cups stacked"],
        "function": ["holding drinks", "holding food", "being sorted into"],
    }
    desc = [
        f"two nested {colour} paper cups",
        f"{colour} disposable paper cup with a rolled rim",
        f"tapered paper cup that holds about {ml:.0f} ml",
        f"light {colour} paperboard cup, wider at the rim",
        "paper cup grasped around its body",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def plate_description(uid, R):
    caption = {
        "color": ["pale lilac", "off-white", "plain"],
        "material": ["paper", "paperboard", "disposable paper"],
        "size": [f"about {R * 200:.0f} cm across", "shallow", "light"],
        "name": ["paper plate", "disposable plate", "small plate"],
        "shape": ["round", "flat bottom", "sloped rim"],
        "function": ["holding food", "serving snacks", "sorting food into"],
    }
    desc = [
        "small round paper plate",
        f"shallow disposable plate about {R * 200:.0f} cm across",
        "light paper plate with a flat bottom and a sloped rim",
        "plain paperboard plate for snacks",
        "round paper dish for sorting food",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def dims(meta, c, keys):
    """measurements_mm from the saved (centred) mesh."""
    p = c["pts"] - (c["pts"].min(0) + c["pts"].max(0)) / 2
    r = np.hypot(p[:, 0], p[:, 1])
    z = p[:, 2] - p[:, 2].min()
    out = {"口径": r[z > z.max() - 1e-4].max() * 2, "底径": r[z < 1e-4].max() * 2, "高": z.max()}
    meta["measurements_mm"] = {k: round(float(out[k]) * 1000, 1) for k in keys}
    return meta


def main(rigid_dir):
    for idx, name, rgb, mass in CUPS:
        c = build_cup(name)
        pc.self_check(c)
        H = CUP_SHAPES[name]["height_m"]
        colour = "white" if name == "white_cup" else "black"
        d, meta = pc.save(
            rigid_dir, "paper_cup", idx, c, paper(rgb), mass, lambda u: cup_description(u, colour, c["ml"], H)
        )
        (d / "metadata.json").write_text(json.dumps(dims(meta, c, ("口径", "底径", "高")), indent=4))
        print(f"{d}: {name}, {c['ml']:.0f} ml, size {meta['physics']['size']}, {len(c['hulls'])} collision hulls")
    for idx, name, R, rb, h, rgb, mass in PLATES:
        c = build_plate(R, rb, h)
        pc.self_check(c)
        d, meta = pc.save(rigid_dir, "paper_plate", idx, c, paper(rgb), mass, lambda u: plate_description(u, R))
        (d / "metadata.json").write_text(json.dumps(dims(meta, c, ("口径", "底径", "高")), indent=4))
        print(f"{d}: {name}, size {meta['physics']['size']}, {len(c['hulls'])} collision hulls")


if __name__ == "__main__":
    try:
        main(sys.argv[1] if len(sys.argv) > 1 else RIGID_DIR)
    except Exception:
        if mp.APP is None:
            raise
        traceback.print_exc()
        sys.stderr.flush()
        os._exit(1)
    if mp.APP is not None:
        mp.APP.close()
