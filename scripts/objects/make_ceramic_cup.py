#!/usr/bin/env python3
"""Glazed ceramic cups (no handle) with a stepped outer wall, as RoboDojo `Rigid` assets.

- ceramic_cup/00000: Real2Sim platform registration real2sim/objects/Rigid/ceramic_cup/00000 (spec.json, by 徐鹤鸣,
  2026-10-23 17:21:00): black glossy ceramic cup, 112 g, 杯口外径 73.1 mm, 杯底 58.4 mm, 杯高 92 mm (measured).
  The outer wall is a stack of rings that step outward toward the rim (photos: six steps, seven rings); the inner wall
  is a smooth cone. Wall and rim thickness, foot ring and inner bottom radius are estimated from the four photos.
- ceramic_cup/00001-00006: generalization variants of 00000 (not measured; colour, glaze, size and shape differ):
  plain white cup, celadon cup, matte terracotta cup, cobalt fine-ribbed cup, wide low stoneware cup, red barrel cup.

Usage:
    python scripts/objects/make_ceramic_cup.py [RIGID_DIR]    # default Assets/Object/RoboDojo/Rigid
    python scripts/objects/check_object.py Assets/Object/RoboDojo/Rigid/ceramic_cup/0000{0,1,2,3,4,5,6} \
        --image-dir real2sim/objects
Writes RIGID_DIR/ceramic_cup/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py needs.

Frame: Z up, metres, origin at bbox centre.
USD:   /root (RigidBodyAPI) -> visual/model (outer wall and bottom), visual/inside (rim, inner wall and bottom)
                            -> collision/hull_NN (solid base up to the inner bottom + wall staves, hollow and open)
The shell is one closed loft of sections r * ring along Z (ring: circle or rounded square), split at the rim.
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
import make_plastic_cups as pc  # noqa: E402  (loft, unit_section, add_collision, metadata)

RIGID_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/RoboDojo/Rigid"
CATEGORY = "ceramic_cup"
N_THETA = pc.N_THETA
STAVE_TOL = 0.0006  # max inner-chord sag of a collision stave
LEVEL_TOL = 0.0013  # max deviation of the collider from the outer profile between two collision levels: steps up to
# ~1.3 mm are smoothed over (a stave per step tripled the hull count for no contact that matters)
MAX_COLS = 28  # perimeter columns per stave: <= 64 hull points (PhysX GPU convex limit)
BASE_SIDES = 24
STEP_RISE = 0.0015  # height over which the wall steps out to the next ring (rounded step edge), estimated

BLACK_GLAZE = dict(mdl="OmniPBR", rgb=(0.012, 0.012, 0.014), rough=0.08, metal=0.0)
WHITE_GLAZE = dict(mdl="OmniPBR", rgb=(0.88, 0.87, 0.84), rough=0.08, metal=0.0)
BLACK = dict(
    H=0.092,  # 杯高, measured: bottom to the top of the rim
    R_TOP=0.03655,  # 杯口外径 / 2, measured 73.1 mm: outside of the top ring at the rim
    R_BOT=0.0292,  # 杯底 / 2, measured 58.4 mm: outside of the bottom ring at the top of the bottom fillet
    STEPS=6,  # outward steps between equal-height rings (0: smooth taper), counted on the photos
    BULGE=0.0,  # outward belly of the profile at mid-height (m)
    T=0.004, TB=0.007,  # wall and rim thickness, inner bottom height: estimated
    FIL=0.003, RECESS=0.0015, RING=0.005,  # bottom fillet, foot recess and foot ring width: estimated
    R_IN=0.008,  # inner bottom corner radius: estimated
    OUTER=BLACK_GLAZE, INNER=BLACK_GLAZE,
    MASS=0.112,  # measured
    WORDS=dict(
        color=["glossy black", "black glaze", "jet black"],
        material=["ceramic", "glazed porcelain", "stoneware"],
        name=["ceramic cup", "black tea cup", "tumbler"],
        shape=["stepped ring wall", "tapered cylinder", "no handle"],
        function=["drinking tea", "holding water", "being picked up and placed"],
        look="glossy black ceramic cup whose wall steps out in rings toward the rim",
    ),
)  # fmt: skip

VARIANTS = [  # generalization variants 00001-00006: overrides of BLACK, all unmeasured (estimated)
    dict(
        H=0.075, R_TOP=0.0320, R_BOT=0.0280, STEPS=0, OUTER=WHITE_GLAZE, INNER=WHITE_GLAZE,
        WORDS=dict(
            color=["glossy white", "white glaze", "pure white"],
            material=["porcelain", "glazed ceramic", "ceramic"],
            name=["white cup", "porcelain cup", "small tumbler"],
            shape=["smooth tapered cylinder", "rounded rim", "no handle"],
            function=["drinking tea", "holding water", "being picked up and placed"],
            look="small plain white porcelain cup with a smooth wall",
        ),
    ),
    dict(
        H=0.100, R_TOP=0.0380, R_BOT=0.0300, STEPS=4,
        OUTER=dict(mdl="OmniPBR", rgb=(0.42, 0.58, 0.47), rough=0.1, metal=0.0),
        INNER=dict(mdl="OmniPBR", rgb=(0.50, 0.65, 0.55), rough=0.1, metal=0.0),
        WORDS=dict(
            color=["celadon green", "pale jade glaze", "sea green"],
            material=["celadon", "glazed porcelain", "ceramic"],
            name=["celadon cup", "jade tea cup", "tumbler"],
            shape=["four stepped rings", "tapered cylinder", "no handle"],
            function=["drinking tea", "holding water", "being picked up and placed"],
            look="pale green celadon cup with a stepped ring wall",
        ),
    ),
    dict(
        H=0.088, R_TOP=0.0360, R_BOT=0.0310, STEPS=3,
        OUTER=dict(mdl="OmniPBR", rgb=(0.48, 0.20, 0.10), rough=0.85, metal=0.0), INNER=WHITE_GLAZE,
        WORDS=dict(
            color=["terracotta outside", "cream glaze inside", "brick red"],
            material=["terracotta", "unglazed clay", "ceramic"],
            name=["terracotta cup", "clay cup", "tumbler"],
            shape=["three stepped rings", "tapered cylinder", "no handle"],
            function=["drinking water", "holding plants", "being picked up and placed"],
            look="matte terracotta cup with a glossy cream glaze inside",
        ),
    ),
    dict(
        H=0.095, R_TOP=0.0350, R_BOT=0.0290, STEPS=12,
        OUTER=dict(mdl="OmniPBR", rgb=(0.02, 0.06, 0.30), rough=0.08, metal=0.0), INNER=WHITE_GLAZE,
        WORDS=dict(
            color=["cobalt blue outside", "white inside", "deep blue"],
            material=["glazed porcelain", "ceramic", "stoneware"],
            name=["blue ribbed cup", "cobalt cup", "tumbler"],
            shape=["fine stepped ribs", "tapered cylinder", "no handle"],
            function=["drinking coffee", "holding water", "being picked up and placed"],
            look="cobalt blue cup with many fine stepped ribs and a white inside",
        ),
    ),
    dict(
        H=0.065, R_TOP=0.0400, R_BOT=0.0280, STEPS=5,
        OUTER=dict(mdl="OmniPBR", rgb=(0.30, 0.29, 0.27), rough=0.6, metal=0.0), INNER=WHITE_GLAZE,
        WORDS=dict(
            color=["stone grey outside", "white glaze inside", "grey"],
            material=["stoneware", "matte ceramic", "ceramic"],
            name=["tea bowl cup", "wide ceramic cup", "low cup"],
            shape=["wide low body", "five stepped rings", "no handle"],
            function=["drinking tea", "holding snacks", "being picked up and placed"],
            look="wide low grey stoneware cup with a white glazed inside",
        ),
    ),
    dict(
        H=0.095, R_TOP=0.0330, R_BOT=0.0290, STEPS=5, BULGE=0.005,
        OUTER=dict(mdl="OmniPBR", rgb=(0.40, 0.02, 0.02), rough=0.08, metal=0.0), INNER=BLACK_GLAZE,
        WORDS=dict(
            color=["glossy red", "oxblood glaze", "black inside"],
            material=["glazed ceramic", "porcelain", "ceramic"],
            name=["barrel cup", "red ceramic cup", "tumbler"],
            shape=["barrel-shaped stepped body", "narrower rim", "no handle"],
            function=["drinking tea", "holding water", "being picked up and placed"],
            look="glossy red barrel-shaped ceramic cup with stepped rings and a black inside",
        ),
    ),
]  # fmt: skip


class Cup:
    """Outer stepped wall r(z), rounded rim, foot ring; inner smooth cone with a rounded bottom corner."""

    def __init__(self, g):
        self.g = g
        self.sec = pc.unit_section(2.0)
        self.z_rim = g["H"] - g["T"] / 2  # rim arc centre height
        n = g["STEPS"] + 1
        self.bounds = g["FIL"] + (self.z_rim - g["FIL"]) * np.arange(1, n) / n  # ring boundaries
        k = np.arange(n) / max(n - 1, 1)
        self.radii = g["R_BOT"] + (g["R_TOP"] - g["R_BOT"]) * k + g["BULGE"] * np.sin(np.pi * k)
        lo = self.inner(g["TB"])
        assert all(self.outer(z) - self.inner(z) > 0.0015 for z in np.linspace(g["TB"], self.z_rim, 200)), (
            "wall too thin"
        )
        assert lo - g["R_IN"] > 0.005, "inner bottom too small for its corner radius"

    def ring(self, r):
        return r * self.sec

    def outer(self, z):
        g = self.g
        if g["STEPS"] == 0:
            t = (z - g["FIL"]) / (self.z_rim - g["FIL"])
            return g["R_BOT"] + (g["R_TOP"] - g["R_BOT"]) * t + g["BULGE"] * np.sin(np.pi * t)
        r = self.radii[0]
        for zb, dr in zip(self.bounds, np.diff(self.radii)):
            u = np.clip((z - zb) / STEP_RISE + 0.5, 0.0, 1.0)
            r += dr * u * u * (3 - 2 * u)
        return float(r)

    def inner(self, z):
        g = self.g
        r0, r1 = g["R_BOT"] - g["T"], g["R_TOP"] - g["T"]
        return r0 + (r1 - r0) * (z - g["TB"]) / (self.z_rim - g["TB"])

    def wall_samples(self):
        g = self.g
        z = list(np.arange(g["FIL"], self.z_rim, 0.002)) + [self.z_rim]
        for zb in self.bounds:
            z += list(np.linspace(zb - STEP_RISE / 2, zb + STEP_RISE / 2, 9))
        return np.unique(np.round([v for v in z if g["FIL"] < v <= self.z_rim], 7))

    def profiles(self):
        """Outside (bottom pole -> rim start) and inside (rim -> inner bottom pole) as [(r, z)]."""
        g = self.g
        T, FIL = g["T"], g["FIL"]
        foot = g["R_BOT"] - FIL - g["RING"]
        outside = [(0.0, g["RECESS"]), (foot, g["RECESS"]), (foot + 0.001, 0.0)]
        outside += pc.fillet(g["R_BOT"], FIL)
        outside += [(self.outer(z), z) for z in self.wall_samples()]
        cx = g["R_TOP"] - T / 2
        rim = [(cx + T / 2 * np.cos(a), self.z_rim + T / 2 * np.sin(a)) for a in np.radians(np.linspace(0, 180, 13))]
        zi = np.linspace(g["TB"] + g["R_IN"], self.z_rim, 50)[::-1]
        inside = rim + [(self.inner(z), z) for z in zi[1:]]
        rc = self.inner(g["TB"] + g["R_IN"]) - g["R_IN"]
        inside += mp.arc(rc, g["TB"] + g["R_IN"], g["R_IN"], 0, -np.pi / 2, 8)[1:] + [(0.0, g["TB"])]
        return outside, inside

    def loft(self, profile):
        return pc.loft([(0.0, z) if r == 0 else (self.ring(r), z) for r, z in profile])

    def hulls(self):
        g = self.g
        step = N_THETA // BASE_SIDES
        base = [(*p, 0.0) for p in self.ring(g["R_BOT"] - g["FIL"])[::step]]
        base += [(*p, g["TB"]) for p in self.ring(self.outer(g["TB"]))[::step]]
        hulls = [np.array(base)]  # solid up to the inner bottom
        zs = np.unique(np.r_[np.arange(g["TB"], self.z_rim, 0.0005), self.z_rim])
        rings = [self.ring(self.outer(z)) for z in zs]
        idx, i = [0], 0
        while i < len(zs) - 1:  # greedy levels: the collider follows the steps within LEVEL_TOL
            j = i + 1
            while j + 1 < len(zs):
                k = np.arange(i + 1, j + 1)
                w = ((zs[k] - zs[i]) / (zs[j + 1] - zs[i]))[:, None, None]
                lerp = (1 - w) * rings[i] + w * rings[j + 1]
                if np.max(np.linalg.norm(np.stack([rings[m] for m in k]) - lerp, axis=2)) > LEVEL_TOL:
                    break
                j += 1
            idx.append(j)
            i = j
        levels = [(z, self.ring(self.outer(z)), self.ring(self.inner(z))) for z in zs[idx]]
        levels.append((g["H"], self.ring(g["R_TOP"] - g["T"] / 2), self.ring(self.inner(self.z_rim))))  # rim top
        hulls += wall_staves(levels)
        return hulls

    def brim_ml(self):
        z = np.linspace(self.g["TB"], self.z_rim, 400)
        a = np.array([pc.section_area(self.ring(self.inner(v))) for v in z])
        return float(np.sum((a[1:] + a[:-1]) / 2 * np.diff(z)) * 1e6)


def wall_staves(levels):
    """pc.wall_staves with STAVE_TOL and at most MAX_COLS perimeter columns per stave."""
    hulls = []
    for (z0, o0, i0), (z1, o1, i1) in zip(levels, levels[1:]):
        cuts = [0]
        while cuts[-1] < N_THETA:
            a, b = cuts[-1], cuts[-1] + 1
            while b < N_THETA and b - a < MAX_COLS and max(pc.chord_sag(r, a, b + 1) for r in (i0, i1)) <= STAVE_TOL:
                b += 1
            cuts.append(b)
        for a, b in zip(cuts, cuts[1:]):
            idx = np.arange(a, b + 1) % N_THETA
            ring = [(*p, z) for r, z in ((o0, z0), (o1, z1)) for p in r[idx]]
            hulls.append(np.array(ring + [(*p, z) for r, z in ((i0, z0), (i1, z1)) for p in r[idx[[0, -1]]]]))
    return hulls


def build(g):
    c = Cup(g)
    outside, inside = c.profiles()
    parts = {"model": c.loft(outside), "inside": c.loft(inside)}
    volume = sum(mp.signed_volume(*p) for p in parts.values())
    assert volume > 0, "shell has inward-facing winding"
    hulls = c.hulls()
    probe = np.array(
        [
            (*(f * p), z)
            for z in np.linspace(g["TB"] + 0.001, g["H"] - 0.001, 40)
            for f in np.linspace(0, 0.94, 6)
            for p in c.ring(c.inner(min(z, c.z_rim)))[:: N_THETA // 48]
        ]
    )
    for h in hulls:
        assert len(h) <= 64, f"collision hull has {len(h)} points"
        eq = ConvexHull(h).equations
        assert not np.any(np.all(probe @ eq[:, :3].T + eq[:, 3] < -1e-6, axis=1)), "hull intrudes into cavity"
    bottom = hulls[0][:, 2]
    assert np.all((bottom == 0) | (bottom >= 0.0015)), "base points within PhysX plane tolerance of the bottom"
    return dict(parts=parts, hulls=hulls, ml=c.brim_ml(), volume=volume, contact_r=g["R_BOT"] - g["FIL"], cup=c)


def surface(name, m):
    def material(st):
        inputs = {"diffuse_color_constant": m["rgb"], "reflection_roughness_constant": m["rough"]}
        inputs["metallic_constant"] = m["metal"]
        return mp.add_material(st, f"/root/_materials/{name}", m["mdl"], inputs, m["rgb"], 1.0, m["rough"])

    return material


def write_usdz(out, c, g):
    materials = {"model": surface("Outside", g["OUTER"]), "inside": surface("Inside", g["INNER"])}
    tmp = Path(tempfile.mkdtemp()) / "object.usdc"
    st = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = mp.add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    mp.add_xform(st, "/root/visual")
    for name, (pts, faces) in c["parts"].items():  # model first: loaders read the first Mesh
        mesh = mp.add_mesh(st, f"/root/visual/{name}", pts, faces, mp.vertex_normals(pts, faces))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(materials[name](st))
    pc.add_collision(st, c["hulls"])
    st.Save()
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    zw.Save()


def description(uid, g, ml):
    w = g["WORDS"]
    caption = {
        "color": w["color"],
        "material": w["material"],
        "size": [f"holds about {ml:.0f} ml", f"about {g['H'] * 100:.1f} cm tall", "fits in one hand"],
        "name": w["name"],
        "shape": w["shape"],
        "function": w["function"],
    }
    desc = [
        w["look"],
        f"{w['name'][0]} without a handle, about {g['H'] * 100:.1f} cm tall",
        f"{w['material'][0]} {w['name'][1]} that holds about {ml:.0f} ml",
        f"{w['name'][0]} with a {w['shape'][0]} and a {w['shape'][1]}",
        f"{w['name'][0]} grasped around its body",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def save(rigid_dir, idx, c, g, mass):
    every = np.concatenate([p for p, _ in c["parts"].values()])
    shift = -(every.min(0) + every.max(0)) / 2
    parts = {k: (p + shift, f) for k, (p, f) in c["parts"].items()}
    c = dict(c, parts=parts, hulls=[h + shift for h in c["hulls"]])
    d = Path(rigid_dir) / CATEGORY / f"{idx:05d}"
    d.mkdir(parents=True, exist_ok=True)
    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/{CATEGORY}/{idx}"))
    write_usdz(d / "object.usdz", c, g)
    faces = [f for _, fs in parts.values() for f in fs]
    meta = pc.metadata(uid, dict(pts=every + shift, faces=faces, contact_r=c["contact_r"]), mass, shift)
    del meta["physics"]["friction"]  # no measured friction: runtime defaults apply
    meta["visual"] = {"color_parts": ["model"]}  # rt_color recolours the outside glaze, the inside glaze stays
    if idx == 0:
        pts = every + shift
        lo = pts[:, 2].min()
        outside = parts["model"][0]
        foot = outside[np.abs(outside[:, 2] - (lo + g["FIL"])) < 1e-6]  # top of the bottom fillet
        meta["measurements_mm"] = {
            "杯口外径": round(float(np.ptp(pts[:, 0])) * 1000, 1),  # widest ring, at the rim
            "杯底": round(float(np.ptp(foot[:, 0])) * 1000, 1),
            "杯高": round(float(np.ptp(pts[:, 2])) * 1000, 1),
        }
    else:
        meta["variant_of"] = f"Rigid/{CATEGORY}/00000"
    (d / "metadata.json").write_text(json.dumps(meta, indent=4, ensure_ascii=False))
    (d / "description.json").write_text(json.dumps(description(uid, g, c["ml"]), indent=4))
    return d, meta


def main(rigid_dir):
    base = None
    for idx, over in enumerate([{}] + VARIANTS):
        g = {**BLACK, **over}
        c = build(g)
        if idx == 0:
            base, mass = c["volume"], g["MASS"]
        else:
            mass = round(BLACK["MASS"] * c["volume"] / base, 4)  # estimated: same ceramic density as 00000
        d, meta = save(rigid_dir, idx, c, g, mass)
        print(
            f"{d}: {c['ml']:.0f} ml, mass {mass} kg, size {meta['physics']['size']}, "
            f"{len(c['hulls'])} collision hulls, measurements_mm {meta.get('measurements_mm', '-')}",
            flush=True,
        )


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
