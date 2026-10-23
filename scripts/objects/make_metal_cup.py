#!/usr/bin/env python3
"""Coated metal tumblers (open, no handle) as RoboDojo `Clutter` assets.

- metal_cup/00000: Real2Sim platform registration real2sim/objects/Clutter/metal_cup/00000 (spec.json, by 徐鹤鸣,
  2026-10-23 16:41:51): aluminium cup, white coating outside and on the bottom, bare reflective metal inside, 65 g,
  开口外径 80.2 mm, 杯底外径 57.3 mm, 杯子高度 105.4 mm (measured). Tapered straight wall with a rolled rim; wall and
  rim thickness, bottom recess and the raised disc on the inner bottom are estimated from the four photos. Not
  modelled: the small red logo near the bottom.
- metal_cup/00001-00006: generalization variants of 00000 (not measured; colour, material, size and shape differ):
  espresso cup, tall tumbler, polished steel straight cup, rounded-square cup, wide low cup, bellied cup.

Usage:
    python scripts/objects/make_metal_cup.py [TYPE_DIR]    # default Assets/Object/RoboDojo/Clutter
Writes TYPE_DIR/metal_cup/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py needs.

Frame: Z up, metres, origin at bbox centre.
USD:   /root (RigidBodyAPI) -> visual/model (coated outside: wall and bottom), visual/inside (rim, inner wall, bottom)
                            -> collision/hull_NN (bottom slab + wall staves, hollow and open)
The shell is one closed loft of sections r * ring along Z (ring: circle or rounded square), split into the coated
outside and the metal inside where the rolled rim starts.
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

TYPE_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/RoboDojo/Clutter"
CATEGORY = "metal_cup"
N_THETA = pc.N_THETA
T_COLL = 0.002  # collider wall, grown inward from the outer surface
STAVE_TOL = 0.0006  # max inner-chord sag of a collision stave
LEVEL_TOL = 0.0005  # max deviation of the collider from the outer profile between two collision levels
MAX_COLS = 28  # perimeter columns per stave: <= 64 hull points (PhysX GPU convex limit)
BOTTOM_SIDES = 24

ALU_COATED = dict(
    H=0.1054,  # 杯子高度, measured: bottom to the top of the rim
    R_TOP=0.0401,  # 开口外径 / 2, measured 80.2 mm: outside of the rolled rim
    R_BOT=0.02865,  # 杯底外径 / 2, measured 57.3 mm: outer wall at the top of the bottom fillet
    SQ=2.0,  # section superellipse exponent: 2 = round
    T_WALL=0.0006, RB=0.0012,  # wall thickness, rolled-rim tube radius: estimated
    FIL=0.0025, RECESS=0.0015, RING=0.004,  # bottom fillet, outer bottom recess, standing ring width: estimated
    TB=0.003, DOME=0.0012, DISC=0.5,  # inner bottom height, raised disc height and radius share: estimated (photo)
    BULGE=0.0,  # outward belly at mid-height (m)
    OUTER=dict(mdl="OmniPBR", rgb=(0.86, 0.86, 0.84), rough=0.55, metal=0.0),  # white coating
    INNER=dict(mdl="OmniPBR", rgb=(0.80, 0.80, 0.80), rough=0.22, metal=1.0),  # bare aluminium
    MASS=0.065,  # measured
    WORDS=dict(
        color=["white outside", "silver metal inside", "matte white"],
        material=["aluminium", "coated metal", "metal"],
        name=["metal cup", "aluminium tumbler", "drinking cup"],
        shape=["tapered cylinder", "rolled rim", "no handle"],
        function=["drinking water", "holding drinks", "receiving a pour"],
        look="white coated aluminium tumbler with a reflective metal inside",
    ),
)  # fmt: skip

STEEL = dict(mdl="OmniPBR", rgb=(0.78, 0.78, 0.80), rough=0.12, metal=1.0)
VARIANTS = [  # generalization variants 00001-00006: overrides of ALU_COATED, all unmeasured (estimated)
    dict(
        H=0.070, R_TOP=0.0320, R_BOT=0.0250, MASS_SCALE=1.0,
        OUTER=dict(mdl="OmniPBR", rgb=(0.03, 0.03, 0.035), rough=0.6, metal=0.0), INNER=STEEL,
        WORDS=dict(
            color=["matte black outside", "steel inside", "black"],
            material=["stainless steel", "coated metal", "metal"],
            name=["espresso cup", "small metal cup", "shot tumbler"],
            shape=["short tapered cylinder", "rolled rim", "no handle"],
            function=["drinking espresso", "holding small drinks", "being picked up and placed"],
            look="small matte black metal espresso cup with a steel inside",
        ),
    ),
    dict(
        H=0.140, R_TOP=0.0400, R_BOT=0.0300, OUTER=dict(mdl="OmniPBR", rgb=(0.30, 0.52, 0.75), rough=0.5, metal=0.0),
        WORDS=dict(
            color=["powder blue outside", "silver inside", "light blue"],
            material=["aluminium", "powder-coated metal", "metal"],
            name=["tall tumbler", "metal tumbler", "tall metal cup"],
            shape=["tall tapered cylinder", "rolled rim", "no handle"],
            function=["drinking juice", "holding cold drinks", "being picked up and placed"],
            look="tall powder blue metal tumbler with a silver inside",
        ),
    ),
    dict(
        H=0.100, R_TOP=0.0375, R_BOT=0.0350, OUTER=STEEL, INNER=STEEL, MASS_SCALE=2.9,
        WORDS=dict(
            color=["polished silver", "mirror steel", "silver"],
            material=["stainless steel", "polished metal", "metal"],
            name=["steel cup", "straight metal cup", "camping cup"],
            shape=["straight cylinder", "rolled rim", "no handle"],
            function=["drinking water", "camping", "being picked up and placed"],
            look="polished stainless steel straight cup, metal inside and out",
        ),
    ),
    dict(
        H=0.105, R_TOP=0.0370, R_BOT=0.0280, SQ=4.0,
        OUTER=dict(mdl="OmniPBR", rgb=(0.45, 0.58, 0.45), rough=0.55, metal=0.0),
        INNER=dict(mdl="OmniPBR", rgb=(0.85, 0.50, 0.32), rough=0.25, metal=1.0),
        WORDS=dict(
            color=["sage green outside", "copper inside", "green"],
            material=["copper-lined metal", "coated metal", "metal"],
            name=["square metal cup", "metal tumbler", "copper cup"],
            shape=["rounded square section", "tapered body", "no handle"],
            function=["drinking cocktails", "holding drinks", "being picked up and placed"],
            look="sage green rounded-square metal cup with a copper inside",
        ),
    ),
    dict(
        H=0.072, R_TOP=0.0400, R_BOT=0.0320, OUTER=dict(mdl="OmniPBR", rgb=(0.62, 0.06, 0.05), rough=0.5, metal=0.0),
        INNER=dict(mdl="OmniPBR", rgb=(0.85, 0.68, 0.35), rough=0.25, metal=1.0),
        WORDS=dict(
            color=["red outside", "gold inside", "red"],
            material=["anodized aluminium", "coated metal", "metal"],
            name=["wide metal cup", "low tumbler", "metal bowl cup"],
            shape=["wide low body", "rolled rim", "no handle"],
            function=["holding snacks", "drinking tea", "being picked up and placed"],
            look="wide low red metal cup with a gold anodized inside",
        ),
    ),
    dict(
        H=0.110, R_TOP=0.0380, R_BOT=0.0290, BULGE=0.004,
        OUTER=dict(mdl="OmniPBR", rgb=(0.92, 0.90, 0.84), rough=0.15, metal=0.0),
        INNER=dict(mdl="OmniPBR", rgb=(0.32, 0.33, 0.35), rough=0.3, metal=1.0),
        WORDS=dict(
            color=["glossy cream outside", "dark gunmetal inside", "cream"],
            material=["enamelled metal", "coated metal", "metal"],
            name=["bellied metal cup", "enamel tumbler", "tulip cup"],
            shape=["bellied curved body", "rolled rim", "no handle"],
            function=["drinking coffee", "holding drinks", "being picked up and placed"],
            look="glossy cream enamel cup with a bellied body and a dark metal inside",
        ),
    ),
]  # fmt: skip


class Cup:
    """Profile of one tumbler: outer wall r(z), rolled rim, recessed bottom, raised disc inside."""

    def __init__(self, g):
        self.g = g
        self.sec = pc.unit_section(g["SQ"])
        self.r_top = g["R_TOP"] - 2 * g["RB"]  # outer wall at the rim, inside the rolled bead
        self.cz = g["H"] - g["RB"]  # bead centre height
        assert self.r_top - g["T_WALL"] > g["R_BOT"] - g["FIL"] - g["RING"], "wall too steep for the bottom"

    def ring(self, r):
        return r * self.sec

    def wall(self, z):
        """Outer wall radius from the fillet top (z = FIL) to the bead centre (z = cz)."""
        g = self.g
        t = (z - g["FIL"]) / (self.cz - g["FIL"])
        return g["R_BOT"] + (self.r_top - g["R_BOT"]) * t + g["BULGE"] * np.sin(np.pi * t)

    def profiles(self):
        """Coated outside (bottom pole -> bead start) and metal inside (bead -> inner bottom pole) as [(r, z)]."""
        g = self.g
        FIL, RB, T = g["FIL"], g["RB"], g["T_WALL"]
        zs = np.linspace(FIL, self.cz - 0.866 * RB, 60)[1:]
        r_in_ring = g["R_BOT"] - FIL - g["RING"]
        outer = [(0.0, g["RECESS"]), (r_in_ring, g["RECESS"]), (r_in_ring + 0.001, 0.0)]
        outer += pc.fillet(g["R_BOT"], FIL)
        outer += [(self.wall(z), z) for z in zs]
        cx = self.r_top + RB
        bead = [(cx + RB * np.cos(a), self.cz + RB * np.sin(a)) for a in np.radians(np.linspace(-120, 180, 16))]
        outer.append(bead[0])
        fi = 0.002
        zi = np.linspace(g["TB"] + fi, self.cz, 60)[::-1]
        rb_in = self.wall(g["TB"] + fi) - T - fi
        inner = bead + [(self.r_top - T, self.cz)]
        inner += [(self.wall(z) - T, z) for z in zi[1:]]
        inner += mp.arc(rb_in, g["TB"] + fi, fi, 0, -np.pi / 2, 6)
        disc = g["DISC"] * rb_in
        inner += [(disc + 0.001, g["TB"]), (disc, g["TB"] + g["DOME"]), (0.0, g["TB"] + g["DOME"])]
        return outer, inner

    def loft(self, profile):
        return pc.loft([(0.0, z) if r == 0 else (self.ring(r), z) for r, z in profile])

    def envelope(self, z):
        return self.wall(min(z, self.cz)) if z < self.g["H"] else self.g["R_TOP"]

    def hulls(self):
        g = self.g
        step = N_THETA // BOTTOM_SIDES
        slab = [(*p, 0.0) for p in self.ring(g["R_BOT"] - g["FIL"])[::step]]
        slab += [(*p, g["FIL"]) for p in self.ring(g["R_BOT"])[::step]]
        hulls = [np.array(slab)]
        zs = np.unique(np.r_[np.arange(g["FIL"], self.cz, 0.001), self.cz])
        rings = [self.ring(self.wall(z)) for z in zs]
        idx, i = [0], 0
        while i < len(zs) - 1:  # greedy levels: the collider follows a curved wall within LEVEL_TOL
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
        levels = [(z, self.ring(self.wall(z)), self.ring(self.wall(z) - T_COLL)) for z in zs[idx]]
        levels.append((g["H"], self.ring(g["R_TOP"]), self.ring(self.r_top - T_COLL)))  # covers the rolled rim
        hulls += wall_staves(levels)
        return hulls

    def cavity(self, z):
        return self.ring(min(self.wall(min(z, self.cz)), self.r_top) - T_COLL)

    def brim_ml(self):
        z = np.linspace(self.g["TB"], self.cz, 600)
        a = np.array([pc.section_area(self.ring(self.wall(v) - self.g["T_WALL"])) for v in z])
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
    outer, inner = c.profiles()
    parts = {"model": c.loft(outer), "inside": c.loft(inner)}
    assert sum(mp.signed_volume(*p) for p in parts.values()) > 0, "shell has inward-facing winding"
    hulls = c.hulls()
    probe = np.array(
        [
            (*(f * p), z)
            for z in np.linspace(g["TB"] + 0.001, g["H"] - 0.001, 40)
            for f in np.linspace(0, 0.94, 6)
            for p in c.cavity(z)[:: N_THETA // 48]
        ]
    )
    for h in hulls:
        assert len(h) <= 64, f"collision hull has {len(h)} points"
        eq = ConvexHull(h).equations
        assert not np.any(np.all(probe @ eq[:, :3].T + eq[:, 3] < -1e-6, axis=1)), "hull intrudes into cavity"
    bottom = hulls[0][:, 2]
    assert np.all((bottom == 0) | (bottom >= 0.0015)), "slab points within PhysX plane tolerance of the bottom"
    return dict(parts=parts, hulls=hulls, ml=c.brim_ml(), contact_r=g["R_BOT"] - g["FIL"], cup=c)


def surface(name, m):
    def material(st):
        return mp.add_material(
            st,
            f"/root/_materials/{name}",
            m["mdl"],
            {
                "diffuse_color_constant": m["rgb"],
                "reflection_roughness_constant": m["rough"],
                "metallic_constant": m["metal"],
            },
            m["rgb"],
            1.0,
            m["rough"],
        )

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
        f"light {w['name'][0]} grasped around its body",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def save(type_dir, idx, c, g, mass):
    every = np.concatenate([p for p, _ in c["parts"].values()])
    shift = -(every.min(0) + every.max(0)) / 2
    parts = {k: (p + shift, f) for k, (p, f) in c["parts"].items()}
    c = dict(c, parts=parts, hulls=[h + shift for h in c["hulls"]])
    d = Path(type_dir) / CATEGORY / f"{idx:05d}"
    d.mkdir(parents=True, exist_ok=True)
    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/clutter/{CATEGORY}/{idx}"))
    write_usdz(d / "object.usdz", c, g)
    faces = [f for _, fs in parts.values() for f in fs]
    meta = pc.metadata(uid, dict(pts=every + shift, faces=faces, contact_r=c["contact_r"]), mass, shift)
    del meta["physics"]["friction"]  # no measured friction: runtime defaults apply
    meta["visual"] = {"color_parts": ["model"]}  # rt_color recolours the coated outside, the metal inside stays
    if idx == 0:
        pts = every + shift
        lo = pts[:, 2].min()
        outside = parts["model"][0]
        foot = outside[np.abs(outside[:, 2] - (lo + g["FIL"])) < 1e-6]  # top of the bottom fillet
        meta["measurements_mm"] = {
            "开口外径": round(float(np.ptp(pts[:, 0])) * 1000, 1),  # outside of the rolled rim
            "杯底外径": round(float(np.ptp(foot[:, 0])) * 1000, 1),
            "杯子高度": round(float(np.ptp(pts[:, 2])) * 1000, 1),
        }
    else:
        meta["variant_of"] = f"Clutter/{CATEGORY}/00000"
    (d / "metadata.json").write_text(json.dumps(meta, indent=4, ensure_ascii=False))
    (d / "description.json").write_text(json.dumps(description(uid, g, c["ml"]), indent=4))
    return d, meta


def shell_area(parts):
    pts, faces = parts["model"]
    return sum(
        np.linalg.norm(np.cross(pts[f[i]] - pts[f[0]], pts[f[i + 1]] - pts[f[0]])) / 2
        for f in faces
        for i in range(1, len(f) - 1)
    )


def main(type_dir):
    base_area = None
    for idx, over in enumerate([{}] + VARIANTS):
        g = {**ALU_COATED, **over}
        c = build(g)
        area = shell_area(c["parts"])
        if idx == 0:
            base_area, mass = area, g["MASS"]
        else:  # estimated: same wall per area as 00000, times the material density ratio (steel / aluminium)
            mass = round(ALU_COATED["MASS"] * area / base_area * g.get("MASS_SCALE", 1.0), 4)
        d, meta = save(type_dir, idx, c, g, mass)
        print(
            f"{d}: {c['ml']:.0f} ml, mass {mass} kg, size {meta['physics']['size']}, "
            f"{len(c['hulls'])} collision hulls, measurements_mm {meta.get('measurements_mm', '-')}",
            flush=True,
        )


if __name__ == "__main__":
    try:
        main(sys.argv[1] if len(sys.argv) > 1 else TYPE_DIR)
    except Exception:
        if mp.APP is None:
            raise
        traceback.print_exc()
        sys.stderr.flush()
        os._exit(1)
    if mp.APP is not None:
        mp.APP.close()
