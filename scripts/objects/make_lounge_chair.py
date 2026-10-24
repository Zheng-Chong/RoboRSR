#!/usr/bin/env python3
"""Upholstered tub armchair of the counter-side lounge (real_scene2, reference/lounge{1,2,3}.jpg) as RoboDojo `Rigid`
assets the G1-D chassis can push aside.

- lounge_chair/00000: blue faux-leather shell and seat, light oak legs (the four chairs at the round table).
- lounge_chair/00001: the same chair in off-white (the chairs of the next table, only partly in the photos).
Not registered on the Real2Sim platform and nothing measured: every size below is estimated from the photos against
the lounge fit (fit_lounge.py, table top assumed 0.75 m); the mass is a typical value for such a chair. The white
chairs are a different, rounder model in the photos; only the colour differs here.

Usage:
    python scripts/objects/make_lounge_chair.py [RIGID_DIR]    # default Assets/Object/RoboDojo/Rigid
    python scripts/objects/check_object.py Assets/Object/RoboDojo/Rigid/lounge_chair/00000
Writes RIGID_DIR/lounge_chair/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py
needs.

Frame: Z up, metres, origin at bbox centre; the chair faces +X (seat front), its back toward -X.
Shape (lounge2/3 photos): a U in plan, straight front, half-round back. An upholstered tub base runs all round under
the inset seat cushion; one shell wraps back and sides, its top sloping down toward the arms' front; at the sides'
front it reaches down to the base, behind that a slot with rounded ends opens between base and shell. Four tapered,
splayed oak legs.
USD:   /root (RigidBodyAPI) -> visual/model (shell + base), visual/seat (cushion), visual/leg_N
                            -> collision/hull_NN (base, cushion, legs, shell pieces; each <= 64 vertices)
The runtime caps rigid masses at 0.5 kg unless the layout entry's physics sets `mass_cap`
(env/scene_manager/objects/rigid.py); layouts with these chairs set it to MASS.
"""

import json
import os
from pathlib import Path
import sys
import tempfile
import traceback
import uuid

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import make_pitcher as mp  # noqa: E402  (shared USD helpers; starts Kit when pxr needs it)
from make_pitcher import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402
import make_plastic_cups as pc  # noqa: E402  (add_collision, metadata)

RIGID_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/RoboDojo/Rigid"
CHAIR = dict(
    # plan: a U, straight front at FRONT_X, half-round back about (BACK_CX, 0) of radius R (width 2R), estimated
    FRONT_X=0.26, BACK_CX=-0.01, R=0.29,
    BASE_Z=(0.30, 0.455),  # upholstered tub base all round under the seat, estimated
    CUSHION_Z=0.48, CUSHION_IN=0.055,  # seat cushion top, inset from the outline (inside the shell wall)
    SHELL_T=0.05,  # shell wall thickness
    TOP_Z=(0.80, 0.68),  # shell top edge at the back and at the arms' front: one line sloping forward
    SLOT_Z=0.58, SLOT_X=0.02,  # the opening between base and shell: behind SLOT_X, up to SLOT_Z; rounded ends
    SLOT_ROUND=0.05,  # m of outline over which the slot's ends round off
    ARM_FRONT=0.02,  # the arms' front edge this far behind the seat front
    LEG_R=(0.016, 0.024), LEG_SPLAY=0.03,  # tapered legs (bottom, top radius) and their outward splay at the floor
    LEGS=((0.20, 0.22), (-0.20, 0.20)),  # (x, |y|) of the front and back legs' tops, under the base
    MASS=7.0,  # estimated, a typical upholstered armchair
    FRICTION=(0.5, 0.4),  # static, dynamic: wood feet on terrazzo, estimated; low enough that a push slides it
)  # fmt: skip
COLOURS = {0: ("blue", (0.11, 0.25, 0.55)), 1: ("off-white", (0.72, 0.70, 0.66))}  # linear RGB
OAK = (0.55, 0.33, 0.15)
HULL_MAX = 64


def frustum(c0, c1, r0, r1, n=10):
    """Closed n-gon frustum from centre c0 (radius r0) to c1 (radius r1)."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.c_[np.cos(a), np.sin(a), np.zeros(n)]
    bot, top = np.asarray(c0) + r0 * ring, np.asarray(c1) + r1 * ring
    return orient([band(bot, top, closed=True), fan(bot), fan(top)])


def band(lo, hi, closed=False):
    """Quads between two matching rows of points (each N x 3), own vertices so normals smooth along it only."""
    n = len(lo)
    p = np.r_[lo, hi]
    idx = [(i, (i + 1) % n) for i in range(n if closed else n - 1)]
    return p, [t for i, j in idx for t in ((i, j, n + j), (i, n + j, n + i))]


def fan(poly):
    """Triangle fan over a convex polygon (N x 3)."""
    return poly, [(0, i, i + 1) for i in range(1, len(poly) - 1)]


def prism(poly, z0, z1):
    """Closed prism over the convex polygon `poly` (N x 2): smooth sides, flat caps."""
    n = len(poly)
    bot, top = np.c_[poly, np.full(n, z0)], np.c_[poly, np.full(n, z1)]
    return orient([band(bot, top, closed=True), fan(bot), fan(top)])


def orient(pieces):
    """Turn each piece's faces outward relative to the solid's centroid, then merge."""
    centre = np.concatenate([p for p, _ in pieces]).mean(0)
    out = []
    for p, f in pieces:
        tri = p[np.array(f)]
        normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        outward = np.einsum("ij,ij->i", normals, tri.mean(1) - centre).sum() > 0
        out.append((p, f if outward else [t[::-1] for t in f]))
    return merge(out)


def u_outline(inset=0.0, step=0.02):
    """The chair's U in plan, `inset` in from the outer edge: points from the front left round the back to the front
    right (counter-clockwise seen from above), with their outward normals."""
    g = CHAIR
    r, cx, xf = g["R"] - inset, g["BACK_CX"], g["FRONT_X"] - g["ARM_FRONT"]
    side = np.linspace(xf, cx, max(int((xf - cx) / step), 2), endpoint=False)
    arc = np.linspace(np.pi / 2, 3 * np.pi / 2, max(int(np.pi * r / step), 8))
    pts = np.r_[np.c_[side, np.full(len(side), r)], np.c_[cx + r * np.cos(arc), r * np.sin(arc)]]
    pts = np.r_[pts, np.c_[side[::-1], np.full(len(side), -r)]]
    nrm = np.r_[np.tile([0, 1.0], (len(side), 1)), np.c_[np.cos(arc), np.sin(arc)], np.tile([0, -1.0], (len(side), 1))]
    return pts, nrm


def shell():
    """The wall wrapping back and sides: its top slopes from TOP_Z[0] at the back to TOP_Z[1] at the arms' front; its
    bottom sits on the base at the sides' front and rises to SLOT_Z behind SLOT_X (the slot), with rounded ends.
    Returns the solid and its per-sample rings (for convex collision pieces)."""
    g = CHAIR
    out, nrm = u_outline()
    inner = out - g["SHELL_T"] * nrm
    back_x, xf = g["BACK_CX"] - g["R"], g["FRONT_X"] - g["ARM_FRONT"]
    top = g["TOP_Z"][1] + (g["TOP_Z"][0] - g["TOP_Z"][1]) * (xf - out[:, 0]) / (xf - back_x)
    s = np.clip((g["SLOT_X"] - out[:, 0]) / g["SLOT_ROUND"], 0.0, 1.0)
    s = np.sin(np.pi / 2 * s)  # quarter-round slot ends
    bottom = g["BASE_Z"][1] + (g["SLOT_Z"] - g["BASE_Z"][1]) * s
    rings = np.stack([np.c_[out, bottom], np.c_[out, top], np.c_[inner, top], np.c_[inner, bottom]])  # 4 x n x 3
    sides = [band(rings[k], rings[(k + 1) % 4]) for k in range(4)]  # outer wall, top, inner wall, bottom
    caps = [fan(rings[:, i]) for i in (0, -1)]  # the arms' front ends
    return orient_shell(sides, caps, rings), rings


def orient_shell(sides, caps, rings):
    """Like orient, but outward relative to the wall's own mid-surface (the U wraps round the centroid)."""
    mid = rings.mean(0)  # n x 3: middle of each cross-section
    out = []
    for p, f in sides + caps:
        tri = p[np.array(f)]
        normals = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        c = tri.mean(1)
        near = mid[np.argmin(np.linalg.norm(c[:, None, :2] - mid[None, :, :2], axis=2), axis=1)]
        d = c - near
        if p.shape[0] == 4:  # an end cap: outward along the outline, away from the wall
            k = np.argmin(np.linalg.norm(mid - c.mean(0), axis=1))
            d = np.repeat([mid[k] - mid[1 if k == 0 else -2]], len(c), axis=0)
        outward = np.einsum("ij,ij->i", normals, d).sum() > 0
        out.append((p, f if outward else [t[::-1] for t in f]))
    return merge(out)


def merge(parts):
    pts, faces, off = [], [], 0
    for p, f in parts:
        pts.append(p)
        faces += [tuple(i + off for i in t) for t in f]
        off += len(p)
    return np.concatenate(pts), faces


def build():
    g = CHAIR
    wall, rings = shell()
    full, _ = u_outline(step=0.06)
    full[0, 0] = full[-1, 0] = g["FRONT_X"]  # the base runs to the seat front
    base = prism(full, *g["BASE_Z"])
    cushion_poly, _ = u_outline(g["CUSHION_IN"], step=0.06)
    cushion = prism(cushion_poly, g["BASE_Z"][1] - 0.01, g["CUSHION_Z"])
    legs = []
    for (x, y), sign in ((leg, sign) for leg in g["LEGS"] for sign in (1, -1)):
        top = np.array([x, sign * y, g["BASE_Z"][0]])
        out = np.r_[top[:2] / np.hypot(*top[:2]) * g["LEG_SPLAY"], 0.0]
        legs.append(frustum(top + out - [0, 0, top[2]], top, *g["LEG_R"]))
    parts = {"model": merge([wall, base]), "seat": cushion}
    parts.update({f"leg_{i}": leg for i, leg in enumerate(legs)})
    for name, (p, f) in parts.items():
        assert mp.signed_volume(p, f) > 0, f"{name} has inward-facing winding"
    hulls = [np.unique(part[0], axis=0) for part in (base, cushion, *legs)]  # faces split their vertices
    n = rings.shape[1]
    for i in range(0, n - 1, 6):  # the wall in pieces of <= 7 samples (28 vertices), each nearly convex
        hulls.append(rings[:, i : min(i + 7, n)].reshape(-1, 3))
    assert all(len(h) <= HULL_MAX for h in hulls)
    return dict(parts=parts, hulls=hulls)


def oak(st):
    return mp.add_material(
        st,
        "/root/_materials/Oak",
        "OmniPBR",
        {"diffuse_color_constant": OAK, "reflection_roughness_constant": 0.5, "metallic_constant": 0.0},
        OAK,
        1.0,
        0.5,
    )


def leather(rgb):
    def material(st):
        return mp.add_material(
            st,
            "/root/_materials/Leather",
            "OmniPBR",
            {"diffuse_color_constant": rgb, "reflection_roughness_constant": 0.45, "metallic_constant": 0.0},
            rgb,
            1.0,
            0.45,
        )

    return material


def write_usdz(out, c, rgb):
    tmp = Path(tempfile.mkdtemp()) / "object.usdc"
    st = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = mp.add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mp.add_xform(st, "/root/visual")
    skin, wood = leather(rgb)(st), oak(st)
    for name, (pts, faces) in c["parts"].items():  # model first: loaders read the first Mesh
        faces = [list(t) for t in faces]
        mesh = mp.add_mesh(st, f"/root/visual/{name}", pts, faces, mp.vertex_normals(pts, faces))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(wood if name.startswith("leg") else skin)
    pc.add_collision(st, c["hulls"])
    st.Save()
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    zw.Save()


def description(uid, colour):
    caption = {
        "color": [colour, f"{colour} upholstery", "oak legs"],
        "material": ["faux leather", "upholstered foam", "wood legs"],
        "size": ["about 60 cm wide", "about 80 cm tall", "seat 47 cm high"],
        "name": ["armchair", "lounge chair", "tub chair"],
        "shape": ["curved tub back", "open slot behind the seat", "four tapered legs"],
        "function": ["sitting at a cafe table", "furnishing a break area", "being pushed aside"],
    }
    desc = [
        f"{colour} upholstered tub armchair on four light oak legs",
        "curved shell wraps the back and sides with a slot above the seat",
        "cafe chair standing around a round wooden table",
        "heavy piece of furniture that slides when pushed",
        "low armrests at seat height curve into the back",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def save(rigid_dir, idx, c):
    every = np.concatenate([p for p, _ in c["parts"].values()])
    shift = -(every.min(0) + every.max(0)) / 2
    parts = {k: (p + shift, f) for k, (p, f) in c["parts"].items()}
    c = dict(c, parts=parts, hulls=[h + shift for h in c["hulls"]])
    d = Path(rigid_dir) / "lounge_chair" / f"{idx:05d}"
    d.mkdir(parents=True, exist_ok=True)
    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/lounge_chair/{idx}"))
    colour, rgb = COLOURS[idx]
    write_usdz(d / "object.usdz", c, rgb)
    faces = [f for _, fs in parts.values() for f in fs]
    contact_r = float(np.hypot(*CHAIR["LEGS"][0]))  # front leg feet
    meta = pc.metadata(uid, dict(pts=every + shift, faces=faces, contact_r=contact_r), CHAIR["MASS"], shift)
    del meta["physics"]["friction"]
    meta["physics"]["static_friction"], meta["physics"]["dynamic_friction"] = CHAIR["FRICTION"]
    (d / "metadata.json").write_text(json.dumps(meta, indent=4, ensure_ascii=False))
    (d / "description.json").write_text(json.dumps(description(uid, colour), indent=4))
    return d, meta


def main(rigid_dir):
    c = build()
    for idx in COLOURS:
        d, meta = save(rigid_dir, idx, c)
        print(f"{d}: size {meta['physics']['size']}, mass {meta['physics']['mass']} kg, {len(c['hulls'])} hulls")


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
