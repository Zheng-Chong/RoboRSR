#!/usr/bin/env python3
"""Open plastic water bottles (cap removed) as RoboDojo `Clutter` assets.

- plastic_bottle/00000: Real2Sim platform registration real2sim/objects/Clutter/plastic_bottle/00000 (spec.json, by
  徐鹤鸣, 2026-10-23 10:54:15): 农夫山泉 550 ml bottle, transparent, red-and-white label, static friction 0.35,
  开口直径 29.8 mm, 底部直径 64.7 mm (measured). The user added on 2026-10-23: 总高 227 mm without cap, empty mass
  18 g (the registered 328 g included water; modelled empty), 开口直径 is the outer diameter. Shoulder, neck finish,
  label band and grip grooves are estimated from the four photos. Hollow and open at the mouth; no cap.
- plastic_bottle/00001-00005: generalization variants of 00000 (not measured; colour, material, size and shape differ):
  380 ml mini, 1 L tall, frosted rounded-square, green waisted grip bottle, amber wide-mouth juice bottle.

Usage:
    python scripts/objects/make_plastic_bottle.py [TYPE_DIR]    # default Assets/Object/RoboDojo/Clutter
    python scripts/objects/check_object.py Assets/Object/RoboDojo/Clutter/plastic_bottle/0000{0,1,2,3,4,5}
Writes TYPE_DIR/plastic_bottle/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py needs.

Frame: Z up, metres, origin at bbox centre.
USD:   /root (RigidBodyAPI) -> visual/model (bottle shell), visual/label_bottom, visual/label_top (label bands)
                            -> collision/hull_NN (bottom slab + wall staves up to the mouth, hollow and open)
Each bottle is a loft of sections r * ring(b) along Z: b = 1 is the variant's section (circle or rounded square), b = 0
a circle; the shoulder blends from b = 1 into the round neck.
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
CATEGORY = "plastic_bottle"
N_THETA = pc.N_THETA
T_WALL = 0.0004  # visual PET wall, estimated (real 550 ml bottles are ~0.25 mm)
T_COLL = 0.002  # collider wall: thicker than the PET so contacts stay robust; grows inward from the outer surface
STAVE_TOL = 0.0006  # max inner-chord sag of a collision stave (around the perimeter)
LEVEL_TOL = 0.0005  # max deviation of the collider from the outer profile between two collision levels (along Z)
MAX_COLS = 28  # perimeter columns per stave: 2 * 29 outer + 4 inner points stays <= 64 (PhysX GPU convex limit)
BOTTOM_SIDES = 24
STATIC_FRICTION = 0.35  # measured (spec static_friction)
DYNAMIC_FRICTION = 0.35  # estimated: not measured, same as static

# Bottle 00000. Radii are outer unless named otherwise; heights from the outside bottom.
NONGFU = dict(
    H=0.227,  # 总高, measured (2026-10-23, from the user): outside bottom to mouth, without cap
    R=0.03235,  # 底部直径 / 2, measured 64.7 mm: outer body diameter just above the bottom fillet
    SQ=2.0,  # section superellipse exponent: 2 = round
    FIL=0.004, FOOT=0.022, DOME=0.006,  # bottom edge fillet, inner edge of the standing ring, push-up height: estimated
    Z_SH=0.158, SH_P=1.3,  # shoulder start (body full width below), shoulder fullness exponent: estimated
    R_NECK=0.013,  # neck under the support ring, estimated (28 mm-class finish)
    NECK_H=0.005, RING_T=0.0015, R_RING=0.0165,  # neck height, support ring thickness and radius: estimated
    FINISH_H=0.0155,  # support ring top to mouth, estimated
    R_THREAD=0.0149,  # 开口直径 / 2, measured 29.8 mm: outer diameter of the threaded finish (thread crests)
    R_CORE=0.0138, R_BORE=0.011,  # finish root and bore radius: estimated
    THREADS=(0.0085, 0.0045),  # thread crest rings, depth below the mouth: estimated
    GROOVES=(0.008, 0.013, 0.018), GROOVE_D=0.0010, GROOVE_W=0.003,  # grip grooves under the label: estimated
    WAIST=None,  # (depth, z0, z1) of an hourglass waist
    LABEL=(0.026, 0.127, 0.60), RECESS=0.0006,  # label z0, z1, bottom-colour share; label panel recess: estimated
    LABEL_RGB=((0.42, 0.015, 0.02), (0.90, 0.90, 0.88)),  # bottom (red), top (white); linear RGB
    TINT=(0.97, 0.98, 0.98), FROST=0.0,  # FROST: milky share of a frosted wall, 0 = clear
    MASS=0.018,  # measured empty (2026-10-23, from the user; the registered 328 g was the bottle with water)
    WORDS=dict(
        color=["transparent", "red and white label", "colourless clear plastic"],
        material=["plastic", "clear PP", "thin-walled plastic"],
        name=["water bottle", "plastic bottle", "Nongfu Spring bottle"],
        shape=["round cylindrical body", "rounded shoulder", "narrow open mouth, no cap"],
        function=["holding drinking water", "being picked up and placed", "recycling"],
        look="clear round water bottle with a red and white label",
    ),
)  # fmt: skip

VARIANTS = [  # generalization variants 00001-00005: overrides of NONGFU, all unmeasured (estimated)
    dict(
        H=0.168, R=0.0290, Z_SH=0.110, GROOVES=(0.007, 0.012), LABEL=(0.030, 0.092, 0.55),
        LABEL_RGB=((0.02, 0.09, 0.40), (0.90, 0.90, 0.88)),
        WORDS=dict(
            color=["transparent", "blue and white label", "clear"],
            material=["plastic", "clear PET", "thin plastic"],
            name=["small water bottle", "mini bottle", "380 ml bottle"],
            shape=["short round body", "rounded shoulder", "narrow open mouth, no cap"],
            function=["holding drinking water", "carrying in a bag", "being picked up and placed"],
            look="small clear water bottle with a blue and white label",
        ),
    ),
    dict(
        H=0.262, R=0.0375, Z_SH=0.185, SH_P=1.1, GROOVES=(0.010, 0.016, 0.022, 0.028, 0.034),
        LABEL=(0.050, 0.150, 0.50), LABEL_RGB=((0.02, 0.22, 0.06), (0.90, 0.90, 0.88)), TINT=(0.84, 0.92, 0.98),
        WORDS=dict(
            color=["light blue tinted", "green and white label", "transparent"],
            material=["plastic", "blue-tinted PET", "thin plastic"],
            name=["large water bottle", "1 litre bottle", "family-size bottle"],
            shape=["tall round body", "long sloping shoulder", "narrow open mouth, no cap"],
            function=["holding drinking water", "sharing water", "refilling cups"],
            look="tall light blue tinted water bottle with a green and white label",
        ),
    ),
    dict(
        H=0.215, R=0.0310, SQ=4.0, Z_SH=0.138, GROOVES=(), LABEL=(0.040, 0.118, 0.50),
        LABEL_RGB=((0.012, 0.012, 0.014), (0.80, 0.22, 0.02)), TINT=(0.95, 0.95, 0.95), FROST=0.35,
        WORDS=dict(
            color=["frosted white", "black and orange label", "translucent"],
            material=["plastic", "frosted PET", "matte plastic"],
            name=["square bottle", "frosted drink bottle", "plastic bottle"],
            shape=["rounded square body", "round neck", "narrow open mouth, no cap"],
            function=["holding drinks", "standing on a shelf", "being picked up and placed"],
            look="frosted rounded-square plastic bottle with a black and orange label",
        ),
    ),
    dict(
        H=0.230, R=0.0335, Z_SH=0.150, SH_P=1.0, GROOVES=(), WAIST=(0.0045, 0.035, 0.110),
        LABEL=(0.112, 0.146, 0.50), LABEL_RGB=((0.015, 0.03, 0.15), (0.85, 0.60, 0.02)), TINT=(0.66, 0.88, 0.70),
        WORDS=dict(
            color=["green tinted", "blue and yellow label", "transparent green"],
            material=["plastic", "green PET", "thin plastic"],
            name=["soda bottle", "green plastic bottle", "grip bottle"],
            shape=["hourglass waist", "rounded shoulder", "narrow open mouth, no cap"],
            function=["holding soft drinks", "being gripped at the waist", "being picked up and placed"],
            look="green tinted soda bottle with an hourglass grip waist",
        ),
    ),
    dict(
        H=0.150, R=0.0340, Z_SH=0.100, SH_P=1.0, R_NECK=0.0205, R_RING=0.0235, FINISH_H=0.013,
        R_THREAD=0.0210, R_CORE=0.0200, R_BORE=0.0185, THREADS=(0.0065, 0.003), GROOVES=(0.008, 0.013),
        LABEL=(0.028, 0.090, 0.60), LABEL_RGB=((0.75, 0.16, 0.01), (0.90, 0.89, 0.84)), TINT=(0.86, 0.60, 0.28),
        WORDS=dict(
            color=["amber tinted", "orange and white label", "translucent brown"],
            material=["plastic", "amber PET", "thick plastic"],
            name=["juice bottle", "wide-mouth bottle", "amber plastic bottle"],
            shape=["short wide body", "wide open mouth, no cap", "short shoulder"],
            function=["holding juice", "storing small items", "being picked up and placed"],
            look="short amber wide-mouth juice bottle with an orange and white label",
        ),
    ),
]  # fmt: skip


class Bottle:
    """Profile of one bottle: outer radius r(z) and section blend b(z), visual and collider."""

    def __init__(self, g):
        self.g = g
        self.sec = pc.unit_section(g["SQ"])
        self.z_ring0 = g["H"] - g["FINISH_H"] - g["RING_T"]
        self.z_ring1 = g["H"] - g["FINISH_H"]
        self.z_neck = self.z_ring0 - g["NECK_H"]
        assert g["Z_SH"] < self.z_neck, "shoulder must end below the neck"
        assert g["R_BORE"] <= g["R_NECK"] - T_COLL + 1e-9, "neck too thin for the collider wall"

    def ring(self, r, b):
        rho = np.linalg.norm(self.sec, axis=1)
        return (r * ((1 - b) + b * rho) / rho)[:, None] * self.sec

    def body(self, z):
        """Outer body envelope (with the waist, without grooves and label recess)."""
        g = self.g
        r = g["R"]
        if g["WAIST"]:
            d, z0, z1 = g["WAIST"]
            if z0 < z < z1:
                r -= d * np.sin(np.pi * (z - z0) / (z1 - z0)) ** 2
        return r

    def visual_body(self, z):
        g = self.g
        r = self.body(z)
        for zc in g["GROOVES"]:
            if abs(z - zc) < g["GROOVE_W"] / 2:
                r -= g["GROOVE_D"] * np.cos(np.pi * (z - zc) / g["GROOVE_W"]) ** 2
        l0, l1, _ = g["LABEL"]
        ramp = min(z - (l0 - 0.004), (l1 + 0.004) - z) / 0.002
        return r - g["RECESS"] * float(np.clip(ramp, 0, 1))

    def shoulder(self, t):
        g = self.g
        r0 = self.body(g["Z_SH"])
        r = g["R_NECK"] + (r0 - g["R_NECK"]) * (1 + np.cos(np.pi * t ** g["SH_P"])) / 2
        return r, 1 - t * t * (3 - 2 * t)

    def envelope(self, z):
        """Collider outer radius and section blend at z (fillet top up to the shoulder end)."""
        g = self.g
        if z <= g["Z_SH"]:
            return self.body(z), 1.0
        return self.shoulder((z - g["Z_SH"]) / (self.z_neck - g["Z_SH"]))

    def body_samples(self):
        g = self.g
        z = list(np.arange(g["FIL"], g["Z_SH"], 0.002)[1:]) + [g["Z_SH"]]
        windows = [(zc - g["GROOVE_W"] / 2, zc + g["GROOVE_W"] / 2) for zc in g["GROOVES"]]
        l0, l1, _ = g["LABEL"]
        windows += [(l0 - 0.004, l0 - 0.002), (l1 + 0.002, l1 + 0.004)]
        if g["WAIST"]:
            windows.append(g["WAIST"][1:])
        for a, b in windows:
            z += list(np.linspace(a, b, max(int((b - a) / 0.00025), 2) + 1))
        return np.unique(np.round([v for v in z if g["FIL"] < v <= g["Z_SH"]], 7))

    def profiles(self):
        """Outer (bottom pole -> mouth lip) and inner (lip -> bottom pole) profiles of the shell as [(r, z, b)]."""
        g = self.g
        H, R, FIL, FOOT, DOME = g["H"], g["R"], g["FIL"], g["FOOT"], g["DOME"]
        dome = [(s * FOOT, DOME * (1 - s * s) ** 2) for s in (0.25, 0.5, 0.75)]
        t = np.linspace(0, 1, 25)[1:]
        sh = [(*self.shoulder(v), g["Z_SH"] + v * (self.z_neck - g["Z_SH"])) for v in t]
        zs = self.body_samples()

        outer = [(0.0, DOME, 1.0)] + [(r, z, 1.0) for r, z in dome]
        outer += [(r, z, 1.0) for r, z in pc.fillet(R, FIL)]
        outer += [(self.visual_body(z), z, 1.0) for z in zs]
        outer += [(r, z, b) for r, b, z in sh]
        z0, z1, rn, rr, rc, rt = self.z_ring0, self.z_ring1, g["R_NECK"], g["R_RING"], g["R_CORE"], g["R_THREAD"]
        finish = [(rn, z0)] * 2 + [(rr, z0)] * 2 + [(rr, z1)] * 2 + [(rc, z1)] * 2
        finish += [(rc, z1 + 0.002), (rt, z1 + 0.003), (rt, z1 + 0.004), (rc, z1 + 0.005)]  # tamper-band bead
        for d in g["THREADS"]:
            zt = H - d
            assert zt - 0.001 > z1 + 0.005, "thread overlaps the bead"
            finish += [(rc, zt - 0.001), (rt, zt - 0.0003), (rt, zt + 0.0003), (rc, zt + 0.001)]
        rb = g["R_BORE"]
        finish += [(rc, H - 0.001), (rc - 0.0004, H - 0.0001), ((rc + rb) / 2, H), (rb + 0.0004, H - 0.0001)]
        finish += [(rb, H - 0.001)]
        outer += [(r, z, 0.0) for r, z in finish]

        inner = [(rb, self.z_neck, 0.0)] * 2 + [(rn - T_WALL, self.z_neck, 0.0)]
        inner += [(r - T_WALL, z, b) for r, b, z in sh[::-1]]
        inner += [(self.visual_body(z) - T_WALL, z, 1.0) for z in zs[::-1]]
        inner += [(r, z, 1.0) for r, z in mp.arc(R - FIL, FIL, FIL - T_WALL, 0, -np.pi / 2, 6)]
        inner += [(FOOT, T_WALL, 1.0)] + [(r, z + T_WALL, 1.0) for r, z in dome[::-1]] + [(0.0, DOME + T_WALL, 1.0)]
        return outer, inner

    def loft(self, profile):
        return pc.loft([(0.0, z) if r == 0 else (self.ring(r, b), z) for r, z, b in profile])

    def labels(self):
        l0, l1, share = self.g["LABEL"]
        split = l0 + share * (l1 - l0)
        parts = {}
        for name, (a, b) in (("label_bottom", (l0, split)), ("label_top", (split, l1))):
            z = np.linspace(a, b, max(int((b - a) / 0.002), 1) + 1)
            out = [(self.visual_body(v) + 0.00025, v, 1.0) for v in z]
            inn = [(self.visual_body(v) + 0.00005, v, 1.0) for v in z[::-1]]
            parts[name] = self.loft(out + [out[-1]] + inn + [inn[-1], out[0]])
        return parts

    def collision_levels(self):
        """Greedy Z levels from the fillet top to the shoulder end: the collider follows the outer envelope within
        LEVEL_TOL between consecutive levels."""
        g = self.g
        zs = np.unique(np.r_[np.arange(g["FIL"], self.z_neck, 0.001), g["Z_SH"], self.z_neck])
        rings = [self.ring(*self.envelope(z)) for z in zs]
        levels, i = [0], 0
        while i < len(zs) - 1:
            j = i + 1
            while j + 1 < len(zs):
                k = np.arange(i + 1, j + 1)
                w = ((zs[k] - zs[i]) / (zs[j + 1] - zs[i]))[:, None, None]
                lerp = (1 - w) * rings[i] + w * rings[j + 1]
                if np.max(np.linalg.norm(np.stack([rings[m] for m in k]) - lerp, axis=2)) > LEVEL_TOL:
                    break
                j += 1
            levels.append(j)
            i = j
        return zs[levels]

    def hulls(self):
        g = self.g
        step = N_THETA // BOTTOM_SIDES
        slab = [(*p, 0.0) for p in self.ring(g["R"] - g["FIL"], 1.0)[::step]]
        slab += [(*p, g["FIL"]) for p in self.ring(self.body(g["FIL"]), 1.0)[::step]]
        hulls = [np.array(slab)]
        levels = []
        for z in self.collision_levels():
            r, b = self.envelope(z)
            levels.append((z, self.ring(r, b), self.ring(r - T_COLL, b)))
        hulls += wall_staves(levels)
        bore = self.ring(g["R_BORE"], 0.0)
        for z0, z1, r in (
            (self.z_neck, self.z_ring0, g["R_NECK"]),
            (self.z_ring0, self.z_ring1, g["R_RING"]),
            (self.z_ring1, g["H"], g["R_THREAD"]),
        ):
            hulls += wall_staves([(z0, self.ring(r, 0.0), bore), (z1, self.ring(r, 0.0), bore)])
        return hulls

    def cavity(self, z):
        """Collider inner wall at z."""
        if z <= self.z_neck:
            r, b = self.envelope(z)
            return self.ring(r - T_COLL, b)
        return self.ring(self.g["R_BORE"], 0.0)

    def brim_ml(self):
        """Visual inner volume from the bottom to the mouth (push-up ignored)."""
        g = self.g
        z = np.linspace(T_WALL, g["H"], 1200)
        a = []
        for v in z:
            if v > self.z_neck:
                ring = self.ring(g["R_BORE"], 0.0)
            elif v > g["Z_SH"]:
                r, b = self.envelope(v)
                ring = self.ring(r - T_WALL, b)
            else:
                ring = self.ring(self.visual_body(max(v, g["FIL"])) - T_WALL, 1.0)
            a.append(pc.section_area(ring))
        a = np.array(a)
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
    b = Bottle(g)
    outer, inner = b.profiles()
    parts = {"model": b.loft(outer + inner), **b.labels()}
    for name, part in parts.items():
        assert mp.signed_volume(*part) > 0, f"{name} has inward-facing winding"
    hulls = b.hulls()
    probe = np.array(
        [
            (*(f * p), z)
            for z in np.linspace(g["FIL"] + 0.001, g["H"] - 0.001, 60)
            for f in np.linspace(0, 0.94, 6)
            for p in b.cavity(z)[:: N_THETA // 48]
        ]
    )
    for h in hulls:
        assert len(h) <= 64, f"collision hull has {len(h)} points"
        eq = ConvexHull(h).equations
        assert not np.any(np.all(probe @ eq[:, :3].T + eq[:, 3] < -1e-6, axis=1)), "hull intrudes into cavity"
    bottom = hulls[0][:, 2]
    assert np.all((bottom == 0) | (bottom >= 0.0015)), "slab points within PhysX plane tolerance of the bottom"
    return dict(parts=parts, hulls=hulls, ml=b.brim_ml(), contact_r=g["R"] - g["FIL"], bottle=b)


def glass(g):
    def material(st):
        if g["FROST"]:  # milky share, as in pc.clear_pp: OmniGlass frosting only blurs, it never whitens the wall
            return mp.add_material(
                st,
                "/root/_materials/Plastic",
                "OmniSurface",
                {
                    "diffuse_reflection_weight": 1.0,
                    "diffuse_reflection_color": g["TINT"],
                    "enable_specular_transmission": True,
                    "specular_transmission_weight": 1.0 - g["FROST"],
                    "specular_reflection_weight": 0.1,
                    "specular_reflection_roughness": 0.3,
                    "specular_reflection_ior": 1.57,
                    "thin_walled": False,
                },
                g["TINT"],
                0.5,
                0.3,
            )
        return mp.add_material(
            st,
            "/root/_materials/Plastic",
            "OmniGlass",
            {"glass_color": g["TINT"], "glass_ior": 1.57, "frosting_roughness": 0.0, "thin_walled": False},
            g["TINT"],
            0.25,
            0.05,
        )

    return material


def paint(name, rgb):
    def material(st):
        return mp.add_material(
            st,
            f"/root/_materials/{name}",
            "OmniPBR",
            {"diffuse_color_constant": rgb, "reflection_roughness_constant": 0.45, "metallic_constant": 0.0},
            rgb,
            1.0,
            0.45,
        )

    return material


def write_usdz(out, c, g):
    materials = {
        "model": glass(g),
        "label_bottom": paint("LabelBottom", g["LABEL_RGB"][0]),
        "label_top": paint("LabelTop", g["LABEL_RGB"][1]),
    }
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
        "size": [f"holds about {ml:.0f} ml to the brim", f"about {g['H'] * 100:.0f} cm tall", "fits in one hand"],
        "name": w["name"],
        "shape": w["shape"],
        "function": w["function"],
    }
    desc = [
        w["look"],
        f"open {w['name'][1]} without a cap, about {g['H'] * 100:.0f} cm tall",
        f"hollow {w['material'][1]} {w['name'][0]} that holds about {ml:.0f} ml",
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
    del meta["physics"]["friction"]
    meta["physics"].update(static_friction=STATIC_FRICTION, dynamic_friction=DYNAMIC_FRICTION)
    meta["visual"] = {"color_parts": ["label_bottom"]}  # rt_color recolours the label, the bottle stays clear
    if idx == 0:
        model = parts["model"][0]
        mouth = model[model[:, 2] > c["bottle"].z_ring1 + shift[2] + 1e-6]
        foot = model[model[:, 2] < model[:, 2].min() + 0.010]
        meta["measurements_mm"] = {
            "开口直径": round(float(np.ptp(mouth[:, 0])) * 1000, 1),  # outer, thread crests
            "底部直径": round(float(np.ptp(foot[:, 0])) * 1000, 1),  # outer body just above the bottom fillet
        }
    else:
        meta["variant_of"] = f"Clutter/{CATEGORY}/00000"
    (d / "metadata.json").write_text(json.dumps(meta, indent=4, ensure_ascii=False))
    (d / "description.json").write_text(json.dumps(description(uid, g, c["ml"]), indent=4))
    return d, meta


def main(type_dir):
    base = None
    for idx, over in enumerate([{}] + VARIANTS):
        g = {**NONGFU, **over}
        c = build(g)
        if idx == 0:
            base = c["ml"]
            assert 550 < base < 620, f"brim volume {base:.0f} ml does not fit a 550 ml bottle"
            mass = g["MASS"]
        else:
            mass = round(NONGFU["MASS"] * c["ml"] / base, 4)  # estimated: empty 00000 scaled by brim volume
        d, meta = save(type_dir, idx, c, g, mass)
        print(
            f"{d}: {c['ml']:.0f} ml brim, mass {mass} kg, size {meta['physics']['size']}, "
            f"{len(c['hulls'])} collision hulls, measurements_mm {meta.get('measurements_mm', '-')}"
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
