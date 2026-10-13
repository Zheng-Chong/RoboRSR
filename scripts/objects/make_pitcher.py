#!/usr/bin/env python3
"""Procedural ribbed-glass fridge jug (Luminarc Quadro style) as RoboDojo `Rigid` assets.

Rounded-square ribbed glass body, shoulder into a round neck, round plastic lid with spout,
side handle hanging from the lid.

Usage:
    python scripts/objects/make_pitcher.py [OUT_DIR]    # default Assets/Object/RoboDojo/Rigid/pitcher
    python scripts/objects/check_object.py Assets/Object/RoboDojo/Rigid/pitcher/0000{0,1,2}
Writes OUT_DIR/0000{0,1,2}/{object.usdz, metadata.json, description.json}, one index per colour
variant. After adding to Assets, run Assets/Object/RoboDojo/Rigid/format.py to refresh uuid_map.json.
Needs numpy + scipy and pxr (usd-core, or the Isaac Sim env, where Kit is started to provide it).

Frame: Z up, metres, origin at bbox centre, handle on +X, spout on -X (like `mug`).
USD:   /root (RigidBodyAPI) -> visual/model (glass body), visual/lid (plastic lid, spout, handle)
                            -> collision/hull_NN (explicit convex hulls, hollow inside)
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

try:
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

    APP = None
except ImportError:  # Isaac Sim pip env: pxr is importable only once Kit is running
    from isaacsim import SimulationApp

    APP = SimulationApp({"headless": True})
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt

OUT_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/RoboDojo/Rigid/pitcher"

# ---- dimensions (m). Reference: Luminarc Quadro 1.1 L jug, 24.5 cm tall, 8.9 cm wide ----
A = 0.0445  # body half-width at face centre (rib crests)
SQ = 5.0  # cross-section superellipse exponent: 2 = circle, larger = sharper square corners
T_WALL = 0.0045  # glass wall at crests (valleys are RIB_DEPTH thinner)
T_BOTTOM = 0.008
FILLET = 0.006  # outer bottom edge radius
N_RIB, RIB_DEPTH = 40, 0.0015  # ribs around the perimeter (multiple of 4 -> symmetric faces)
SHOULDER = 0.014  # square body -> round neck transition height
R_NECK, T_NECK = 0.0385, 0.0035
Z_TOP = 0.245  # top of lid
T_CAP = 0.008  # lid top plate
LID_BAND = 0.033  # plastic collar height
COLLAR_T = 0.0025
SPOUT_OPEN = 0.013  # pour opening cut into the lid on the spout side; 0 = closed lid
SPOUT_LEN = 0.011
X_GRIP = A + 0.031  # handle grip centreline
HANDLE_Z0 = 0.150  # free end of the handle
HANDLE_W, HANDLE_T, BEND_R = 0.013, 0.009, 0.012
MASS, FRICTION = 0.5, 0.25  # RigidObject clamps mass to <= 0.5 kg
N_THETA = N_RIB * 8
STAVE_TOL = 0.0007  # max collision-wall intrusion into the cavity (smaller = more convex hulls)

A_IN = A - T_WALL
R_NI = R_NECK - T_NECK
H_GLASS = Z_TOP - T_CAP - 0.001
Z_LID = Z_TOP - T_CAP  # underside of lid plate / top of collar
Z_C0 = Z_LID - LID_BAND  # collar bottom = shoulder top
Z_SH0 = Z_C0 - SHOULDER
R_CI = R_NECK + 0.0005
R_C = R_CI + COLLAR_T

VARIANTS = [  # colour word, glass tint, lid colour
    ("smoky gray", (0.78, 0.80, 0.80), (0.22, 0.22, 0.24)),
    ("amber", (0.92, 0.74, 0.38), (0.50, 0.30, 0.08)),
    ("clear", (0.97, 0.99, 0.98), (0.93, 0.93, 0.93)),
]
THETA = np.linspace(0, 2 * np.pi, N_THETA, endpoint=False)


def unit_section():
    """Superellipse of half-width 1, resampled uniformly by arc length from the +X face centre (CCW).
    Returns points and outward unit normals."""
    t = np.linspace(0, 2 * np.pi, 8192, endpoint=False)
    rho = (np.abs(np.cos(t)) ** SQ + np.abs(np.sin(t)) ** SQ) ** (-1 / SQ)
    closed = np.vstack([np.c_[rho * np.cos(t), rho * np.sin(t)], [[1.0, 0.0]]])
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))]
    u = np.linspace(0, s[-1], N_THETA, endpoint=False)
    q = np.c_[np.interp(u, s, closed[:, 0]), np.interp(u, s, closed[:, 1])]
    tan = np.roll(q, -1, 0) - np.roll(q, 1, 0)
    nrm = np.c_[tan[:, 1], -tan[:, 0]]
    return q, nrm / np.linalg.norm(nrm, axis=1, keepdims=True)


SECTION, SECTION_N = unit_section()
RIB = RIB_DEPTH * (1 - np.abs(np.cos(np.pi * N_RIB * np.arange(N_THETA) / N_THETA)))  # crest at face centres


# ---------------------------------------------------------------- geometry
def section(r, b):
    """Cross-section with half-width r, blending circle (b=0) -> rounded square (b=1)."""
    rho = np.linalg.norm(SECTION, axis=1)
    return (r * ((1 - b) + b * rho) / rho)[:, None] * SECTION


def arc(cx, cz, rad, a0, a1, n=7):
    a = np.linspace(a0, a1, n)
    return list(zip(cx + rad * np.cos(a), cz + rad * np.sin(a)))


def loft(profile, ribbed=()):
    """Closed surface through cross-sections [(r, z, b)] stacked along Z; r == 0 is a pole.
    Indices in `ribbed` get vertical ribs."""
    pts, rings = [], []
    for i, (r, z, b) in enumerate(profile):
        if r == 0:
            rings.append([len(pts)] * N_THETA)
            pts.append((0.0, 0.0, z))
            continue
        xy = section(r, b) - (RIB[:, None] * SECTION_N if i in ribbed else 0)
        rings.append(list(range(len(pts), len(pts) + N_THETA)))
        pts += [(x, y, z) for x, y in xy]
    faces = []
    for a, b in zip(rings, rings[1:]):
        for j in range(N_THETA):
            k = (j + 1) % N_THETA
            quad = [a[j], a[k], b[k], b[j]]
            faces.append([v for n, v in enumerate(quad) if v not in quad[:n]])  # pole quad -> tri
    return np.array(pts), faces


def prism(poly, z0, z1):
    """Closed, flat-shaded prism over a convex CCW polygon [(x, y)]."""
    n = len(poly)
    pts = np.array([(x, y, z) for z in (z0, z1) for x, y in poly])
    faces = [list(range(n))[::-1], list(range(n, 2 * n))]
    faces += [[i, (i + 1) % n, n + (i + 1) % n, n + i] for i in range(n)]
    return unweld(pts, faces)


def unweld(pts, faces):
    out = [pts[f] for f in faces]
    starts = np.cumsum([0] + [len(f) for f in faces])
    return np.concatenate(out), [list(range(s, s + len(f))) for s, f in zip(starts, faces)]


def sweep(path, w, t):
    """Rectangular bar (w along Y, t in XZ) along an XZ polyline. Returns mesh and per-ring corners."""
    p = np.array([(x, 0.0, z) for x, z in path])
    tan = np.gradient(p, axis=0)
    tan /= np.linalg.norm(tan, axis=1, keepdims=True)
    nrm = np.c_[-tan[:, 2], np.zeros(len(p)), tan[:, 0]]
    y = np.array([0.0, 1.0, 0.0])
    sec = [(1, 1), (-1, 1), (-1, -1), (1, -1)]
    q = np.stack([p + a * nrm * t / 2 + b * y * w / 2 for a, b in sec], axis=1)  # (n, 4, 3)
    pts, faces = [], []
    for s in range(4):  # each side gets its own vertices -> sharp box edges, smooth along the bend
        base = len(pts)
        for ring in q:
            pts += [ring[s], ring[(s + 1) % 4]]
        faces += [[base + 2 * i, base + 2 * i + 2, base + 2 * i + 3, base + 2 * i + 1] for i in range(len(q) - 1)]
    for ring, order in ((q[0], [0, 1, 2, 3]), (q[-1], [3, 2, 1, 0])):
        faces.append([len(pts) + i for i in order])
        pts += list(ring)
    return np.array(pts), faces, q


def vertex_normals(pts, faces):
    n = np.zeros_like(pts)
    for f in faces:
        p = pts[f]
        n[f] += sum(np.cross(p[i] - p[0], p[i + 1] - p[0]) for i in range(1, len(f) - 1))
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)


def signed_volume(pts, faces):
    return sum(np.dot(pts[f[0]], np.cross(pts[f[i]], pts[f[i + 1]])) for f in faces for i in range(1, len(f) - 1)) / 6


def stave_cuts(inner_rings):
    """Perimeter split points so every stave's inner chord stays within STAVE_TOL of the inner wall:
    long staves on flat faces, short ones around corners."""
    cuts = [0]
    while cuts[-1] < N_THETA:
        a, b = cuts[-1], cuts[-1] + 1
        while b < N_THETA and max(chord_sag(r, a, b + 1) for r in inner_rings) <= STAVE_TOL:
            b += 1
        cuts.append(b)
    return cuts


def chord_sag(ring, a, b):
    p = ring[np.arange(a, b + 1) % N_THETA]
    d, v = p[-1] - p[0], p - p[0]
    return np.max(np.abs(d[0] * v[:, 1] - d[1] * v[:, 0])) / np.linalg.norm(d)


def lid_polygon():
    c = np.c_[R_C * np.cos(THETA), R_C * np.sin(THETA)]
    if SPOUT_OPEN <= 0:
        return [tuple(v) for v in c]
    xc = -R_CI + SPOUT_OPEN
    yc = np.sqrt(R_C**2 - xc**2)
    keep = c[:, 0] > xc
    upper, lower = c[keep & (THETA <= np.pi)], c[keep & (THETA > np.pi)]
    return [tuple(v) for v in upper] + [(xc, yc), (xc, -yc)] + [tuple(v) for v in lower]


def build():
    """Returns visual parts [(pts, faces, material)], collision hull point sets, grip point."""
    fi = 0.004  # inner bottom fillet
    u = np.linspace(0, 1, 7)
    sm = u * u * (3 - 2 * u)
    shoulder_out = [(A + (R_NECK - A) * s, Z_SH0 + SHOULDER * v, 1 - s) for v, s in zip(u, sm)]
    shoulder_in = [(A_IN + (R_NI - A_IN) * s, Z_SH0 + SHOULDER * v, 1 - s) for v, s in zip(u, sm)][::-1]
    glass = (
        [(0.0, 0.0, 1)]
        + [(r, z, 1) for r, z in arc(A - FILLET, FILLET, FILLET, -np.pi / 2, 0)]
        + shoulder_out
        + [(R_NECK, H_GLASS, 0), (R_NECK, H_GLASS, 0), (R_NI, H_GLASS, 0), (R_NI, H_GLASS, 0)]
        + shoulder_in
        + [(r, z, 1) for r, z in arc(A_IN - fi, T_BOTTOM + fi, fi, 0, -np.pi / 2)]
        + [(0.0, T_BOTTOM, 1)]
    )
    collar = [
        (R_CI, Z_LID),
        (R_CI, Z_C0),
        (R_CI, Z_C0),
        (R_C, Z_C0),
        (R_C, Z_C0),
        (R_C, Z_LID),
        (R_C, Z_LID),
        (R_CI, Z_LID),
    ]
    lid = lid_polygon()
    spout = [(-R_C + 0.004, 0.016), (-R_C - SPOUT_LEN, 0.0), (-R_C + 0.004, -0.016)]
    z_h = Z_LID - 0.012
    path = [(R_C - 0.002, z_h)] + arc(X_GRIP - BEND_R, z_h - BEND_R, BEND_R, np.pi / 2, 0) + [(X_GRIP, HANDLE_Z0)]
    h_pts, h_faces, h_rings = sweep(path, HANDLE_W, HANDLE_T)

    visual = [
        (*loft(glass, ribbed=range(1, 9)), "glass"),  # bottom fillet + straight wall up to the shoulder
        (*loft([(r, z, 0) for r, z in collar]), "plastic"),
        (*prism(lid, Z_LID, Z_TOP), "plastic"),
        (*prism(spout, Z_LID - 0.003, Z_LID), "plastic"),
        (h_pts, h_faces, "plastic"),
    ]

    hulls = [np.array([(x, y, z) for x, y in section(A, 1)[::4] for z in (0, T_BOTTOM)])]
    # wall staves per height band (body, shoulder, neck/collar): a stave spanning the shoulder
    # would not be convex and its hull would fill the cavity
    levels = [  # z, outer section, inner section
        (0.0, section(A, 1), section(A_IN, 1)),
        (Z_SH0, section(A, 1), section(A_IN, 1)),
        (Z_C0, section(R_C, 0), section(R_NI, 0)),
        (Z_LID, section(R_C, 0), section(R_NI, 0)),
    ]
    for (z0, o0, i0), (z1, o1, i1) in zip(levels, levels[1:]):
        cuts = stave_cuts([i0, i1])
        for a, b in zip(cuts, cuts[1:]):  # inner face uses only the end columns so it stays a chord
            idx = np.arange(a, b + 1) % N_THETA
            ring = [(*p, z) for o, z in ((o0, z0), (o1, z1)) for p in o[idx]]
            hulls.append(np.array(ring + [(*p, z) for s, z in ((i0, z0), (i1, z1)) for p in s[idx[[0, -1]]]]))
    hulls.append(np.array([(x, y, z) for x, y in lid for z in (Z_LID, Z_TOP)]))
    hulls.append(np.array([(x, y, z) for x, y in spout for z in (Z_LID - 0.003, Z_LID)]))
    hulls += [np.concatenate([h_rings[i], h_rings[i + 1]]) for i in range(len(h_rings) - 1)]

    grip = np.array([X_GRIP, 0.0, (z_h - BEND_R + HANDLE_Z0) / 2])
    return visual, hulls, grip


def self_check(visual, hulls):
    for pts, faces, _ in visual:
        assert signed_volume(pts, faces) > 0, "part has inward-facing winding"
    x, y = SECTION.T
    unit_area = 0.5 * abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))
    litres = (A_IN**2 * unit_area * (Z_SH0 - T_BOTTOM) + np.pi * R_NI**2 * (H_GLASS - Z_SH0)) * 1000
    assert 1.0 < litres < 1.6, litres
    # cavity (body + neck; shoulder skipped) must be free of collision so liquid fits, and so must the pour opening
    probe = [
        (*(f * s), z)
        for s_full, z0, z1 in ((section(A_IN, 1), T_BOTTOM, Z_SH0), (section(R_NI, 0), Z_C0, Z_LID))
        for f in np.linspace(0, 1 - 0.0015 / A_IN, 6)
        for s in s_full[::8]
        for z in np.linspace(z0 + 1e-3, z1 - 1e-3, 10)
    ]
    if SPOUT_OPEN > 0:
        probe.append((-R_CI + SPOUT_OPEN - 0.004, 0.0, Z_TOP - T_CAP / 2))
    probe = np.array(probe)
    for h in hulls:
        eq = ConvexHull(h).equations
        assert not np.any(np.all(probe @ eq[:, :3].T + eq[:, 3] < -1e-6, axis=1)), "collision hull intrudes into cavity"
    return litres


# ---------------------------------------------------------------- USD
def add_mesh(stage, path, pts, faces, normals=None):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    m.CreateFaceVertexCountsAttr(Vt.IntArray([len(f) for f in faces]))
    m.CreateFaceVertexIndicesAttr(Vt.IntArray([int(i) for f in faces for i in f]))
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    m.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*pts.min(0)), Gf.Vec3f(*pts.max(0))]))
    if normals is not None:
        m.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(normals.astype(np.float32)))
        m.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    return m


def add_material(stage, path, mdl, mdl_inputs, rgb, opacity, roughness):
    """MDL surface for Isaac RTX plus a UsdPreviewSurface fallback (Quick Look, usdview)."""
    types = {tuple: Sdf.ValueTypeNames.Color3f, float: Sdf.ValueTypeNames.Float, bool: Sdf.ValueTypeNames.Bool}
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/Shader")
    sh.SetSourceAsset(Sdf.AssetPath(f"{mdl}.mdl"), "mdl")
    sh.SetSourceAssetSubIdentifier(mdl, "mdl")
    for k, v in mdl_inputs.items():
        sh.CreateInput(k, types[type(v)]).Set(v)
    mat.CreateSurfaceOutput("mdl").ConnectToSource(sh.CreateOutput("out", Sdf.ValueTypeNames.Token))
    pv = UsdShade.Shader.Define(stage, f"{path}/Preview")
    pv.CreateIdAttr("UsdPreviewSurface")
    pv.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(rgb)
    pv.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
    pv.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(roughness)
    pv.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.5)
    mat.CreateSurfaceOutput().ConnectToSource(pv.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    return mat


def add_xform(stage, path):
    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0))
    x.AddOrientOp().Set(Gf.Quatf(1, 0, 0, 0))
    return x


def write_usdz(out, visual, hulls, glass_rgb, lid_rgb):
    tmp = Path(tempfile.mkdtemp()) / "object.usdc"
    st = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())

    mats = {
        "glass": add_material(
            st,
            "/root/_materials/Glass",
            "OmniGlass",
            {"glass_color": glass_rgb, "glass_ior": 1.5, "frosting_roughness": 0.0, "thin_walled": False},
            glass_rgb,
            0.25,
            0.05,
        ),
        "plastic": add_material(
            st,
            "/root/_materials/Plastic",
            "OmniPBR",
            {"diffuse_color_constant": lid_rgb, "reflection_roughness_constant": 0.3, "metallic_constant": 0.0},
            lid_rgb,
            1.0,
            0.3,
        ),
    }

    # Keep refractive glass and opaque plastic in separate meshes with direct material bindings.
    # The glass body stays /root/visual/model, the first mesh RigidObject.sample_mesh_vertices() finds.
    add_xform(st, "/root/visual")
    for name, path in (("glass", "/root/visual/model"), ("plastic", "/root/visual/lid")):
        pts, faces, nrm = [], [], []
        for p, f, m in visual:
            if m == name:
                base = sum(len(x) for x in pts)
                pts.append(p)
                nrm.append(vertex_normals(p, f))
                faces += [[i + base for i in face] for face in f]
        mesh = add_mesh(st, path, np.concatenate(pts), faces, np.concatenate(nrm))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mats[name])

    add_xform(st, "/root/collision")
    for i, h in enumerate(hulls):
        hull = ConvexHull(h)
        tri = hull.simplices.copy()
        n = np.cross(h[tri[:, 1]] - h[tri[:, 0]], h[tri[:, 2]] - h[tri[:, 0]])
        flip = np.einsum("ij,ij->i", n, hull.equations[:, :3]) < 0
        tri[flip] = tri[flip][:, ::-1]
        used, tri = np.unique(tri, return_inverse=True)
        m = add_mesh(st, f"/root/collision/hull_{i:02d}", h[used], tri.reshape(-1, 3).tolist())
        prim = m.GetPrim()
        m.CreatePurposeAttr(UsdGeom.Tokens.guide)
        vis = UsdGeom.VisibilityAPI.Apply(prim)
        vis.CreateRenderVisibilityAttr(UsdGeom.Tokens.invisible)
        vis.CreateGuideVisibilityAttr(UsdGeom.Tokens.invisible)
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(UsdPhysics.Tokens.convexHull)
    st.Save()
    # plain zip instead of UsdUtils packaging: OmniPBR/OmniGlass.mdl are Isaac built-ins, not files to bundle
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter  # Usd.* in Isaac Sim's older USD
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    zw.Save()


# ---------------------------------------------------------------- metadata
def r4(v):
    return [round(float(x), 4) for x in v]


def metadata(uid, pts, n_faces, grip, axis_xy):
    lo, hi = pts.min(0), pts.max(0)
    ext = hi - lo
    corners = [r4((x, y, z)) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]
    bbox = {"vertices": corners, "extents": r4(ext)}
    return {
        "uuid": uid,
        "physics": {"mass": MASS, "friction": FRICTION, "size": r4(ext)},
        "geometry": {
            "faces": n_faces,
            "vertices": len(pts),
            "aligned_bbox": bbox,
            "oriented_bbox": bbox,  # faces are axis-aligned and spout -> handle is X
            "radius": round(float(np.linalg.norm(pts, axis=1).max()), 4),
            "bbox": r4(ext),
            "center": r4((lo + hi) / 2),
        },
        "active": {
            "place": {
                "up": {
                    "orientation": [1.0, 0.0, 0.0, 0.0],
                    "projection_circle": {
                        "center": r4((0, 0, lo[2], 1, 0, 0, 0)),
                        "radius": round(float(max(ext[:2]) / 2), 4),
                    },
                    "contact_circle": {"center": r4((*axis_xy, lo[2], 1, 0, 0, 0)), "radius": round(A - FILLET, 4)},
                }
            },
            # same grasp frames as mug/00015, whose handle is also a vertical bar on +X
            "functional": {"handle": {"frame": [r4((*grip, 0.5, 0.5, 0.5, 0.5)), r4((*grip, 0.5, -0.5, 0.5, -0.5))]}},
        },
        "passive": {"functional": {}, "support": {}},
    }


def description(uid, color, litres):
    caption = {
        "color": [f"{color} tinted glass", f"{color} plastic lid", "transparent body"],
        "material": ["glass", "ribbed pressed glass", "plastic lid and handle"],
        "size": [f"holds about {litres:.1f} liters", "tall and slim", "fits in a fridge door"],
        "name": ["water pitcher", "fridge jug", "cold water jug"],
        "shape": ["tall square body with rounded corners", "vertical ribs", "round lid with spout and side handle"],
        "function": ["holding cold water", "pouring drinks", "storing juice in the fridge"],
    }
    desc = [
        f"{color} ribbed square glass water pitcher with a plastic lid and handle",
        f"tall slim fridge jug in {color} glass with vertical ribs",
        "square cold water jug with a spout on the lid for pouring",
        f"{color} glass pitcher that holds about {litres:.1f} liters",
        "rounded square glass jug with a side handle attached to the lid",
        "fridge door water pitcher with a ribbed transparent square body",
        f"glass water jug with a {color} plastic lid used for pouring drinks",
        "tall ribbed square pitcher for storing juice or water in the fridge",
        "transparent cold water jug grasped by its plastic handle",
        f"{color} fridge jug with spout, lid and handle",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def main(out_dir):
    visual, hulls, grip = build()
    litres = self_check(visual, hulls)
    all_pts = np.concatenate([p for p, _, _ in visual])
    shift = -(all_pts.min(0) + all_pts.max(0)) / 2
    visual = [(p + shift, f, m) for p, f, m in visual]
    hulls = [h + shift for h in hulls]
    all_pts += shift
    n_faces = sum(len(f) for _, f, _ in visual)

    for idx, (color, glass_rgb, lid_rgb) in enumerate(VARIANTS):
        d = Path(out_dir) / f"{idx:05d}"
        d.mkdir(parents=True, exist_ok=True)
        uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/pitcher/{idx}"))
        write_usdz(d / "object.usdz", visual, hulls, glass_rgb, lid_rgb)
        meta = metadata(uid, all_pts, n_faces, grip + shift, shift[:2])
        (d / "metadata.json").write_text(json.dumps(meta, indent=4))
        (d / "description.json").write_text(json.dumps(description(uid, color, litres), indent=4))

        st = Usd.Stage.Open(str(d / "object.usdz"))
        first_mesh = next(p for p in Usd.PrimRange(st.GetDefaultPrim()) if p.IsA(UsdGeom.Mesh))
        assert first_mesh.GetPath() == Sdf.Path("/root/visual/model"), first_mesh.GetPath()
        print(f"{d}: {color}, {litres:.2f} L, size {meta['physics']['size']}, {len(hulls)} collision hulls")


if __name__ == "__main__":
    try:
        main(sys.argv[1] if len(sys.argv) > 1 else OUT_DIR)
    except Exception:
        if APP is None:
            raise
        traceback.print_exc()
        sys.stderr.flush()
        os._exit(1)  # Kit swallows exceptions and exits 0
    if APP is not None:
        APP.close()
