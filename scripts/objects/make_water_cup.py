#!/usr/bin/env python3
"""Procedural enamel water cup (the scene3_pour_pack receiver cup) as a RoboDojo `Rigid` asset, or with --plastic
the clear disposable cup of the real cup-to-cup bean pours (G1D-PourBeansPlus clips).

Plain tapered cup without a handle: white enamel outside, steel inside. Dimensions from
Assets/scene3_pour_pack/scene_spec.json (receiver_cup). The plastic cup is clear PP (OmniGlass), sized by eye against
the Dex1-1 fingers in the clips.

Usage:
    python scripts/objects/make_water_cup.py [OUT_DIR]    # default Assets/Object/RoboDojo/Rigid/water_cup
    python scripts/objects/make_water_cup.py --plastic [OUT_DIR]    # default .../Rigid/plastic_cup
    python scripts/objects/check_object.py Assets/Object/RoboDojo/Rigid/water_cup/00000
Writes OUT_DIR/00000/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py needs.

Frame: Z up, metres, origin at bbox centre.
USD:   /root (RigidBodyAPI) -> visual/model (enamel outside and rim), visual/inside (steel)
                            -> collision/hull_NN (bottom disc + wall staves, hollow inside)
"""

import json
import os
from pathlib import Path
import sys
import tempfile
import traceback
import uuid

import numpy as np
from scipy.spatial import ConvexHull

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_pitcher as mp  # noqa: E402  (shared USD helpers; starts Kit when pxr needs it)
from make_pitcher import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402

OUT_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/RoboDojo/Rigid/water_cup"

H, R0, R1 = 0.118, 0.0275, 0.0365  # height, outer radius at the bottom and at the rim
T_WALL, T_BOTTOM = 0.003, 0.006
MASS, FRICTION = 0.165, 0.5
PLASTIC_CUP = False
N_THETA, N_STAVE = 192, 32  # visual segments; collision wall staves (inner chord sag 0.15 mm)
ENAMEL, STEEL = (0.84, 0.85, 0.81), (0.56, 0.58, 0.60)
PLASTIC = dict(H=0.095, R0=0.026, R1=0.036, T_WALL=0.002, T_BOTTOM=0.003, MASS=0.03)
CLEAR = (0.97, 0.98, 0.98)


def lathe(profile):
    """Surface of revolution through [(r, z)] about Z; r == 0 is a pole. Repeat a point for a sharp edge."""
    t = np.linspace(0, 2 * np.pi, N_THETA, endpoint=False)
    pts, rings = [], []
    for r, z in profile:
        if r == 0:
            rings.append([len(pts)] * N_THETA)
            pts.append((0.0, 0.0, z))
        else:
            rings.append(list(range(len(pts), len(pts) + N_THETA)))
            pts += [(r * np.cos(a), r * np.sin(a), z) for a in t]
    faces = []
    for (p, q), a, b in zip(zip(profile, profile[1:]), rings, rings[1:]):
        if p == q:  # sharp edge: no strip between the two copies
            continue
        for j in range(N_THETA):
            k = (j + 1) % N_THETA
            quad = [a[j], a[k], b[k], b[j]]
            faces.append([v for n, v in enumerate(quad) if v not in quad[:n]])
    return np.array(pts), faces


def wall(z):
    """Outer and inner radius of the tapered wall at height z."""
    r = R0 + (R1 - R0) * z / H
    return r, r - T_WALL


def build():
    """Visual parts [(pts, faces, material)] and collision hull point sets, origin at the bottom centre."""
    r0i, r1i = wall(T_BOTTOM)[1], wall(H)[1]
    outside = [(0.0, 0.0), (R0 - 0.002, 0.0), (R0, 0.002), (R0, 0.002), (R1, H), (R1, H), (r1i, H), (r1i, H)]
    inside = [(r1i, H), (r0i, T_BOTTOM), (r0i, T_BOTTOM), (0.0, T_BOTTOM)]
    visual = [(*lathe(outside), "enamel"), (*lathe(inside), "steel")]

    a = np.linspace(0, 2 * np.pi, N_STAVE, endpoint=False)
    disc = [(R0 * np.cos(x), R0 * np.sin(x)) for x in a]
    hulls = [np.array([(x, y, z) for x, y in disc for z in (0.0, T_BOTTOM)])]
    for a0, a1 in zip(a, np.r_[a[1:], 2 * np.pi]):
        ring = []
        for z in (0.0, H):
            r = wall(z)[0]
            ring += [(r * np.cos(x), r * np.sin(x), z) for x in (a0, a1)]
        for z in (T_BOTTOM, H):
            r = wall(z)[1]
            ring += [(r * np.cos(x), r * np.sin(x), z) for x in (a0, a1)]
        hulls.append(np.array(ring))
    return visual, hulls


def self_check(hulls):
    ml = np.pi / 3 * (H - T_BOTTOM) * (wall(T_BOTTOM)[1] ** 2 + wall(T_BOTTOM)[1] * wall(H)[1] + wall(H)[1] ** 2) * 1e6
    probe = np.array(
        [
            (f * wall(z)[1] * np.cos(x), f * wall(z)[1] * np.sin(x), z)
            for z in np.linspace(T_BOTTOM + 1e-3, H - 1e-3, 12)
            for f in np.linspace(0, 0.97, 6)
            for x in np.linspace(0, 2 * np.pi, 48, endpoint=False)
        ]
    )
    for h in hulls:
        eq = ConvexHull(h).equations
        assert not np.any(np.all(probe @ eq[:, :3].T + eq[:, 3] < -1e-6, axis=1)), "collision hull intrudes into cavity"
    return ml


def write_usdz(out, visual, hulls):
    tmp = Path(tempfile.mkdtemp()) / "object.usdc"
    st = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = mp.add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    if PLASTIC_CUP:  # one refractive material over the outer and inner wall surfaces (a solid 2 mm shell)
        clear = mp.add_material(
            st,
            "/root/_materials/Plastic",
            "OmniGlass",
            {"glass_color": CLEAR, "glass_ior": 1.49, "frosting_roughness": 0.0, "thin_walled": False},
            CLEAR,
            0.25,
            0.05,
        )
        mats = {"enamel": clear, "steel": clear}
    else:
        mats = {
            name: mp.add_material(
                st,
                f"/root/_materials/{name.capitalize()}",
                "OmniPBR",
                {"diffuse_color_constant": rgb, "reflection_roughness_constant": rough, "metallic_constant": metal},
                rgb,
                1.0,
                rough,
            )
            for name, rgb, rough, metal in (("enamel", ENAMEL, 0.32, 0.0), ("steel", STEEL, 0.27, 0.94))
        }
    # the enamel stays /root/visual/model, the first mesh RigidObject.sample_mesh_vertices() finds
    mp.add_xform(st, "/root/visual")
    for (pts, faces, name), path in zip(visual, ("/root/visual/model", "/root/visual/inside")):
        mesh = mp.add_mesh(st, path, pts, faces, mp.vertex_normals(pts, faces))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mats[name])

    mp.add_xform(st, "/root/collision")
    for i, h in enumerate(hulls):
        hull = ConvexHull(h)
        tri = hull.simplices.copy()
        n = np.cross(h[tri[:, 1]] - h[tri[:, 0]], h[tri[:, 2]] - h[tri[:, 0]])
        flip = np.einsum("ij,ij->i", n, hull.equations[:, :3]) < 0
        tri[flip] = tri[flip][:, ::-1]
        used, tri = np.unique(tri, return_inverse=True)
        m = mp.add_mesh(st, f"/root/collision/hull_{i:02d}", h[used], tri.reshape(-1, 3).tolist())
        prim = m.GetPrim()
        m.CreatePurposeAttr(UsdGeom.Tokens.guide)
        vis = UsdGeom.VisibilityAPI.Apply(prim)
        vis.CreateRenderVisibilityAttr(UsdGeom.Tokens.invisible)
        vis.CreateGuideVisibilityAttr(UsdGeom.Tokens.invisible)
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
    st.Save()
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    zw.Save()


def metadata(uid, pts, n_faces):
    lo, hi = pts.min(0), pts.max(0)
    ext = hi - lo
    corners = [mp.r4((x, y, z)) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    bbox = {"vertices": corners, "extents": mp.r4(ext)}
    return {
        "uuid": uid,
        "physics": {"mass": MASS, "friction": FRICTION, "size": mp.r4(ext)},
        "geometry": {
            "faces": n_faces,
            "vertices": len(pts),
            "aligned_bbox": bbox,
            "oriented_bbox": bbox,
            "radius": round(float(np.linalg.norm(pts, axis=1).max()), 4),
            "bbox": mp.r4(ext),
            "center": mp.r4((lo + hi) / 2),
        },
        "active": {
            "place": {
                "up": {
                    "orientation": [1.0, 0.0, 0.0, 0.0],
                    "projection_circle": {"center": mp.r4((0, 0, lo[2], 1, 0, 0, 0)), "radius": R1},
                    "contact_circle": {"center": mp.r4((0, 0, lo[2], 1, 0, 0, 0)), "radius": R0},
                }
            },
            "functional": {},
        },
        "passive": {"functional": {}, "support": {}},
    }


def description(uid, ml):
    if PLASTIC_CUP:
        caption = {
            "color": ["clear", "transparent", "colourless"],
            "material": ["plastic", "polypropylene", "PP"],
            "size": [f"holds about {ml:.0f} ml", "about 9.5 cm tall", "fits in one hand"],
            "name": ["plastic cup", "disposable cup", "clear cup"],
            "shape": ["tapered cylinder", "wider at the rim", "no handle"],
            "function": ["drinking water", "holding beans", "pouring into another cup"],
        }
        desc = [
            "clear plastic cup without a handle",
            "transparent disposable cup, wider at the rim",
            f"thin-walled plastic cup that holds about {ml:.0f} ml",
            "colourless tapered drinking cup",
            "light plastic cup grasped around its body",
        ]
        return {"uuid": uid, "caption": caption, "description": desc}
    caption = {
        "color": ["white enamel", "steel inside", "silver rim"],
        "material": ["enamel", "enamelled steel", "metal"],
        "size": [f"holds about {ml:.0f} ml", "about 12 cm tall", "fits in one hand"],
        "name": ["water cup", "enamel cup", "drinking cup"],
        "shape": ["tapered cylinder", "wider at the rim", "no handle"],
        "function": ["drinking water", "holding drinks", "receiving a pour"],
    }
    desc = [
        "white enamel water cup without a handle",
        "tapered white drinking cup, wider at the rim",
        f"enamel steel cup that holds about {ml:.0f} ml",
        "plain white cup with a steel inside and a silver rim",
        "handleless enamel cup grasped around its body",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def main(out_dir):
    visual, hulls = build()
    ml = self_check(hulls)
    shift = np.array([0.0, 0.0, -H / 2])
    visual = [(p + shift, f, m) for p, f, m in visual]
    hulls = [h + shift for h in hulls]
    all_pts = np.concatenate([p for p, _, _ in visual])
    d = Path(out_dir) / "00000"
    d.mkdir(parents=True, exist_ok=True)
    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/{'plastic_cup' if PLASTIC_CUP else 'water_cup'}/0"))
    write_usdz(d / "object.usdz", visual, hulls)
    meta = metadata(uid, all_pts, sum(len(f) for _, f, _ in visual))
    (d / "metadata.json").write_text(json.dumps(meta, indent=4))
    (d / "description.json").write_text(json.dumps(description(uid, ml), indent=4))
    print(f"{d}: {ml:.0f} ml, size {meta['physics']['size']}, {len(hulls)} collision hulls")


if __name__ == "__main__":
    args = sys.argv[1:]
    PLASTIC_CUP = "--plastic" in args
    args = [a for a in args if a != "--plastic"]
    if PLASTIC_CUP:
        globals().update(PLASTIC)
    try:
        main(args[0] if args else OUT_DIR.with_name("plastic_cup") if PLASTIC_CUP else OUT_DIR)
    except Exception:
        if mp.APP is None:
            raise
        traceback.print_exc()
        sys.stderr.flush()
        os._exit(1)
    if mp.APP is not None:
        mp.APP.close()
