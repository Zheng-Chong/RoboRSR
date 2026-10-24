#!/usr/bin/env python3
"""Upholstered tub armchair of the counter-side lounge (real_scene2, reference/lounge{1,2,3}.jpg) as RoboDojo `Rigid`
assets the G1-D chassis can push aside.

- lounge_chair/00000: blue faux-leather shell and seat, light oak legs (the four chairs at the round table).
- lounge_chair/00001: the same chair in off-white (the chairs of the next table, only partly in the photos).
Not registered on the Real2Sim platform and nothing measured: every size below is estimated from the photos against
the lounge fit (fit_lounge.py, table top assumed 0.75 m); the mass is a typical value for such a chair. The white
chair's real shell is rounder; only its colour differs here.

Usage:
    python scripts/objects/make_lounge_chair.py [RIGID_DIR]    # default Assets/Object/RoboDojo/Rigid
    python scripts/objects/check_object.py Assets/Object/RoboDojo/Rigid/lounge_chair/00000
Writes RIGID_DIR/lounge_chair/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py
needs.

Frame: Z up, metres, origin at bbox centre; the chair faces +X (seat front), its back toward -X.
USD:   /root (RigidBodyAPI) -> visual/model (shell), visual/seat, visual/leg_N
                            -> collision/hull_NN (seat, shell segments, legs; each <= 64 vertices)
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
    SEAT_W=0.50, SEAT_D=0.48, SEAT_Z=(0.40, 0.47),  # cushion, estimated
    SHELL_R=0.30, SHELL_T=0.035,  # tub shell outer radius in plan and wall thickness, estimated
    SHELL_CX=-0.02,  # shell centre in plan, a little behind the seat centre
    BACK_Z=(0.55, 0.80),  # back band: the slot between it and the seat is the opening seen from behind
    ARM_Z=(0.47, 0.68), ARM_DEG=(55, 125),  # armrests (from the seat front side round to the back), deg from -X
    BACK_DEG=125,  # the back band spans +-BACK_DEG about -X
    LEG_R=(0.016, 0.024), LEG_SPLAY=0.03,  # tapered legs (bottom, top radius) and their outward splay at the floor
    MASS=7.0,  # estimated, a typical upholstered armchair
    FRICTION=(0.5, 0.4),  # static, dynamic: wood feet on terrazzo, estimated; low enough that a push slides it
)  # fmt: skip
COLOURS = {0: ("blue", (0.11, 0.25, 0.55)), 1: ("off-white", (0.72, 0.70, 0.66))}  # linear RGB
OAK = (0.55, 0.33, 0.15)
HULL_MAX = 64


def box(x0, x1, y0, y1, z0, z1):
    p = np.array([(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)], float)
    f = [(0, 2, 3), (0, 3, 1), (4, 5, 7), (4, 7, 6), (0, 1, 5), (0, 5, 4)]
    f += [(2, 6, 7), (2, 7, 3), (0, 4, 6), (0, 6, 2), (1, 3, 7), (1, 7, 5)]
    return p, f


def frustum(c0, c1, r0, r1, n=10):
    """Closed n-gon frustum from centre c0 (radius r0) to c1 (radius r1), outward winding for c1 above c0."""
    a = np.linspace(0, 2 * np.pi, n, endpoint=False)
    ring = np.c_[np.cos(a), np.sin(a), np.zeros(n)]
    p = np.r_[np.asarray(c0) + r0 * ring, np.asarray(c1) + r1 * ring, [c0], [c1]]
    f = []
    for i in range(n):
        j = (i + 1) % n
        f += [(i, j, n + j), (i, n + j, n + i), (2 * n, j, i), (2 * n + 1, n + i, n + j)]
    return p, f


def arc_band(cx, r_out, r_in, a0, a1, z0, z1, n):
    """Closed annular-sector solid about (cx, 0) between angles a0 < a1 (rad, from +X), heights z0..z1."""
    a = np.linspace(a0, a1, n)
    rings = []
    for r, z in ((r_out, z0), (r_out, z1), (r_in, z1), (r_in, z0)):
        rings.append(np.c_[cx + r * np.cos(a), r * np.sin(a), np.full(n, z)])
    p = np.concatenate(rings)
    f = []
    for k in range(4):  # outer wall, top, inner wall, bottom: ring k to ring k+1
        s, t = k * n, ((k + 1) % 4) * n
        for i in range(n - 1):
            f += [(s + i, s + i + 1, t + i + 1), (s + i, t + i + 1, t + i)]
    for i in (0, n - 1):  # end caps
        q = [i, n + i, 2 * n + i, 3 * n + i]
        f += [(q[0], q[1], q[2]), (q[0], q[2], q[3])] if i else [(q[0], q[2], q[1]), (q[0], q[3], q[2])]
    return p, f


def merge(parts):
    pts, faces, off = [], [], 0
    for p, f in parts:
        pts.append(p)
        faces += [tuple(i + off for i in t) for t in f]
        off += len(p)
    return np.concatenate(pts), faces


def build():
    g = CHAIR
    deg = np.deg2rad
    ro, ri, cx = g["SHELL_R"], g["SHELL_R"] - g["SHELL_T"], g["SHELL_CX"]
    back = (np.pi - deg(g["BACK_DEG"]), np.pi + deg(g["BACK_DEG"]))
    arms = [(np.pi - deg(g["ARM_DEG"][1]), np.pi - deg(g["ARM_DEG"][0]))]
    arms.append((np.pi + deg(g["ARM_DEG"][0]), np.pi + deg(g["ARM_DEG"][1])))
    shell = [arc_band(cx, ro, ri, *back, *g["BACK_Z"], 40)]
    shell += [arc_band(cx, ro, ri, a0, a1, *g["ARM_Z"], 12) for a0, a1 in arms]
    sx, sy = g["SEAT_D"] / 2, g["SEAT_W"] / 2
    seat = box(-sx, sx, -sy, sy, *g["SEAT_Z"])
    z_leg = g["SEAT_Z"][0]
    legs = []
    for x, y in ((sx - 0.06, sy - 0.06), (sx - 0.06, -sy + 0.06), (-sx + 0.06, sy - 0.06), (-sx + 0.06, -sy + 0.06)):
        out = np.array([x, y]) / np.hypot(x, y) * g["LEG_SPLAY"]
        legs.append(frustum((x + out[0], y + out[1], 0.0), (x, y, z_leg), *g["LEG_R"]))
    parts = {"model": merge(shell), "seat": seat}
    parts.update({f"leg_{i}": leg for i, leg in enumerate(legs)})
    for name, (p, f) in parts.items():
        assert mp.signed_volume(p, f) > 0, f"{name} has inward-facing winding"
    hulls = [seat[0]] + [leg[0] for leg in legs]
    for band, n in zip(shell, (40, 12, 12)):  # split each band into convex pieces of <= 8 arc steps
        p = band[0].reshape(4, n, 3)
        for i in range(0, n - 1, 4):
            hulls.append(p[:, i : min(i + 5, n)].reshape(-1, 3))
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
    contact_r = float(np.hypot(CHAIR["SEAT_D"], CHAIR["SEAT_W"]) / 2)
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
