#!/usr/bin/env python3
"""Earbud charging case (white glossy plastic, AirPods-style) as a RoboDojo `Rigid` asset, lying on its back.

- airpods/00000: Real2Sim platform registration real2sim/objects/Rigid/airpods/00000 (spec.json, by 种政,
  2026-10-20 19:33:42): PP plastic, white, 38 g, 高 × 宽 × 厚 46.40 × 54.40 × 21.38 mm (measured). Rounded box with
  big plan-view corners and rounded edges (radii estimated from the photos), lid seam groove, USB-C port.
  Not modelled: speaker holes, status light, lanyard loop, the slight dome of the faces.

Usage:
    python scripts/objects/make_airpods.py [RIGID_DIR]    # default Assets/Object/RoboDojo/Rigid
    python scripts/objects/check_object.py Assets/Object/RoboDojo/Rigid/airpods/00000
Writes RIGID_DIR/airpods/<index>/{object.usdz, metadata.json, description.json}. Needs what make_pitcher.py needs.

Frame: Z up, metres, origin at bbox centre. 厚 along Z (lying flat), 宽 along X, 高 along Y; lid (hinge side) on +Y,
USB-C port on the -Y end.
USD:   /root (RigidBodyAPI) -> visual/model (case shell), visual/seam (lid seam), visual/port (USB-C)
                            -> collision/hull_00 (one convex hull of the shell, <= 64 vertices)
The shell is lofted along the 高 axis: each section is a rounded rectangle in the 宽-厚 plane.
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
import make_plastic_cups as pc  # noqa: E402  (loft, add_collision, metadata)

RIGID_DIR = Path(__file__).resolve().parents[2] / "Assets/Object/RoboDojo/Rigid"
N_THETA = pc.N_THETA  # multiple of 4: every ring passes through its four face centres, so the extents are exact
CASE = dict(
    H=0.04640,  # 高, measured: lid-hinge side to port side, outer, lying flat it is the Y extent
    W=0.05440,  # 宽, measured: side to side, outer (X extent)
    T=0.02138,  # 厚, measured: back face to front face, outer (Z extent)
    R_PLAN=0.015,  # plan-view corner radius (宽-高 plane), estimated from the photos
    R_EDGE=0.008,  # edge radius around the faces (厚 direction), estimated
    LID=0.016,  # lid depth from the hinge side to the seam, estimated
    SEAM_W=0.0004, SEAM_D=0.0003,  # seam groove width and depth, estimated
    PORT_W=0.0084, PORT_H=0.0026,  # USB-C opening, standard size
    MASS=0.038,  # measured
)  # fmt: skip
WHITE = (0.88, 0.88, 0.86)
SEAM_GREY = (0.30, 0.30, 0.30)
PORT_BLACK = (0.03, 0.03, 0.03)
HULL_MAX = 64  # PhysX GPU convex limit
FLAT_BAND = 0.0015  # collider points stay on a flat face or at least this far from it


def rounded_rect(a, b, r):
    """Rounded rectangle of half-sizes (a, b), corner radius r, N_THETA points uniform by arc length, CCW from +X."""
    poly = [(a, 0.0)]
    for k, (sx, sy) in enumerate(((1, 1), (-1, 1), (-1, -1), (1, -1))):
        t = np.linspace(k * np.pi / 2, (k + 1) * np.pi / 2, 64)
        poly += list(zip(sx * (a - r) + r * np.cos(t), sy * (b - r) + r * np.sin(t)))
    poly.append((a, 0.0))
    p = np.array(poly)
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    u = np.linspace(0, s[-1], N_THETA, endpoint=False)
    return np.c_[np.interp(u, s, p[:, 0]), np.interp(u, s, p[:, 1])]


def inset(r, s, half):
    """How far a corner of radius r pulls the surface in at distance s from the centre (half-length `half`)."""
    x = max(0.0, s - (half - r))
    return r - np.sqrt(max(0.0, r * r - x * x))


def section(w, off=0.0):
    """Shell section at loft coordinate w (along 高), grown by `off`."""
    g = CASE
    s = abs(w)
    dc, de = inset(g["R_PLAN"], s, g["H"] / 2), inset(g["R_EDGE"], s, g["H"] / 2)
    return rounded_rect(g["W"] / 2 - dc + off, g["T"] / 2 - de + off, g["R_EDGE"] - de + off)


def to_case(p):
    """Loft frame (u = 宽, v = 厚, w = 高) -> case frame (X, Y, Z) = (u, -w, v); a proper rotation, keeps winding."""
    return np.c_[p[:, 0], -p[:, 2], p[:, 1]]


def build_case():
    g = CASE
    half = g["H"] / 2
    phi = np.linspace(0, np.pi / 2, 25)
    ends = np.unique(np.r_[half - g["R_PLAN"] + g["R_PLAN"] * np.sin(phi), half - g["R_EDGE"]])
    w_seam = -(half - g["LID"])  # case Y = half - LID
    w0, w1 = w_seam - g["SEAM_W"] / 2, w_seam + g["SEAM_W"] / 2
    assert abs(w_seam) + g["SEAM_W"] < half - g["R_PLAN"], "seam must sit on the straight part of the shell"
    full, groove = section(0.0), section(0.0, -g["SEAM_D"])
    profile = (
        [(0.0, -half)]
        + [(section(w), w) for w in -ends[::-1]]
        + [(full, w0), (groove, w0), (groove, w1), (full, w1)]
        + [(section(w), w) for w in ends]
        + [(0.0, half)]
    )
    shell = pc.loft(profile)
    # dark band just under the groove floor: shows through the groove, stays inside the shell's extents
    band_out, band_in = section(0.0, -g["SEAM_D"] + 0.00005), section(0.0, -g["SEAM_D"] - 0.0003)
    seam = pc.loft([(band_out, w0), (band_out, w1), (band_in, w1), (band_in, w0), (band_out, w0)])
    pw, ph = g["PORT_W"] / 2, g["PORT_H"] / 2
    port = mp.prism([(-pw, -ph), (pw, -ph), (pw, ph), (-pw, ph)], half - 0.0005, half + 0.00003)

    parts = {}
    for name, (pts, faces) in (("model", shell), ("seam", seam), ("port", port)):
        assert mp.signed_volume(pts, faces) > 0, f"{name} has inward-facing winding"
        pts = to_case(np.asarray(pts, dtype=float))
        assert mp.signed_volume(pts, faces) > 0, f"{name} winding flipped by to_case"
        parts[name] = (pts, faces)
    return dict(parts=parts, hulls=[hull_points(parts["model"][0])], contact_r=half - g["R_EDGE"])


def hull_points(pts):
    """<= HULL_MAX shell points for one convex collider: the corners of the flat back and front faces (so the case
    rests flat), then greedily the shell point farthest outside the current hull. Points just off a flat face are
    skipped: PhysX cooking merges points within its plane tolerance (0.7 mm) of a face, which tilted the resting case
    by 2.4 deg."""
    hv = pts[ConvexHull(pts).vertices]
    chosen = []
    for z in (hv[:, 2].min(), hv[:, 2].max()):
        flat = np.flatnonzero(np.abs(hv[:, 2] - z) < 1e-7)
        chosen += [int(flat[np.argmax(hv[flat, :2] @ d)]) for d in ((1, 1), (-1, 1), (-1, -1), (1, -1))]
    near = np.zeros(len(hv), bool)
    for k in (1, 2):  # flat end caps (Y) and faces (Z)
        d = hv[:, k].max() - np.abs(hv[:, k])
        near |= (d > 1e-7) & (d < FLAT_BAND)
    while len(chosen) < HULL_MAX:
        eq = ConvexHull(hv[chosen]).equations
        out = (hv @ eq[:, :3].T + eq[:, 3]).max(1)
        out[near] = -np.inf
        chosen.append(int(np.argmax(out)))
    return hv[chosen]


def self_check(c):
    shell = c["parts"]["model"][0]
    (hull,) = c["hulls"]
    eq = ConvexHull(hull).equations
    gap = (shell @ eq[:, :3].T + eq[:, 3]).max(1).max()  # how far the shell reaches past the collider
    assert len(hull) <= HULL_MAX and gap < 0.0015, f"collision hull too coarse: {gap * 1000:.2f} mm"
    lo = shell[:, 2].min()
    assert np.ptp(hull[np.abs(hull[:, 2] - lo) < 1e-7][:, :2], axis=0).min() > 0.02, "hull bottom is not flat"
    return float(gap)


def plastic(path, rgb, roughness):
    def material(st):
        return mp.add_material(
            st,
            path,
            "OmniPBR",
            {"diffuse_color_constant": rgb, "reflection_roughness_constant": roughness, "metallic_constant": 0.0},
            rgb,
            1.0,
            roughness,
        )

    return material


MATERIALS = {
    "model": plastic("/root/_materials/GlossyWhite", WHITE, 0.15),
    "seam": plastic("/root/_materials/Seam", SEAM_GREY, 0.5),
    "port": plastic("/root/_materials/Port", PORT_BLACK, 0.6),
}


def write_usdz(out, c):
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
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(MATERIALS[name](st))
    pc.add_collision(st, c["hulls"])
    st.Save()
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(out))
    zw.AddFile(str(tmp), "object.usdc")
    zw.Save()


def description(uid):
    caption = {
        "color": ["white", "glossy white", "pure white"],
        "material": ["plastic", "glossy plastic", "PP"],
        "size": ["palm-sized", "about 5 cm wide", "about 2 cm thick"],
        "name": ["earbud case", "AirPods charging case", "wireless earphone case"],
        "shape": ["rounded rectangle", "flat pebble-like box", "flip lid with a seam"],
        "function": ["storing earbuds", "charging earbuds", "being picked up and put away"],
    }
    desc = [
        "white glossy charging case for wireless earbuds",
        "small rounded plastic box lying flat with a lid seam near one edge",
        "palm-sized earphone case with a USB-C port on one end",
        "smooth white pebble-shaped case about 5 cm wide and 2 cm thick",
        "light plastic earbud case grasped across its thin sides",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def save(rigid_dir, idx, c):
    every = np.concatenate([p for p, _ in c["parts"].values()])
    shift = -(every.min(0) + every.max(0)) / 2
    parts = {k: (p + shift, f) for k, (p, f) in c["parts"].items()}
    c = dict(c, parts=parts, hulls=[h + shift for h in c["hulls"]])
    d = Path(rigid_dir) / "airpods" / f"{idx:05d}"
    d.mkdir(parents=True, exist_ok=True)
    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, f"robodojo/rigid/airpods/{idx}"))
    write_usdz(d / "object.usdz", c)
    faces = [f for _, fs in parts.values() for f in fs]
    meta = pc.metadata(uid, dict(pts=every + shift, faces=faces, contact_r=c["contact_r"]), CASE["MASS"], shift)
    del meta["physics"]["friction"]  # no measured friction: runtime defaults apply
    ext = np.ptp(parts["model"][0], axis=0) * 1000  # shell only
    meta["measurements_mm"] = {k: round(float(ext[i]), 1) for k, i in (("高", 1), ("宽", 0), ("厚", 2))}
    (d / "metadata.json").write_text(json.dumps(meta, indent=4, ensure_ascii=False))
    (d / "description.json").write_text(json.dumps(description(uid), indent=4))
    return d, meta


def main(rigid_dir):
    c = build_case()
    gap = self_check(c)
    d, meta = save(rigid_dir, 0, c)
    print(
        f"{d}: size {meta['physics']['size']}, measurements_mm {meta['measurements_mm']}, "
        f"1 collision hull ({len(c['hulls'][0])} points, max gap to shell {gap * 1000:.2f} mm)"
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
