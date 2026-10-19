#!/usr/bin/env python3
"""Procedural red (adzuki) bean as a RoboDojo `Rigid` asset, also used to draw settled bean particles as beans.

Blunt-ended superellipsoid, 7 x 5 x 5 mm and 0.1 g (measured adzuki), glossy dark maroon, with the white hilum stripe
along the +Y side. The bean particles of the g1d tasks are 4 mm PBD spheres; this mesh is not their collider.

Usage:
    python scripts/objects/make_red_bean.py [RIGID_DIR]    # default Assets/Object/RoboDojo/Rigid
Writes RIGID_DIR/red_bean/00000/{object.usdz, metadata.json, description.json}.

Frame: Z up (thickness), X along the bean, hilum on +Y, metres, origin at the centre.
USD:   /root (RigidBodyAPI) -> visual/model (seed coat), visual/hilum -> collision/hull_00
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
import make_plastic_cups as pc  # noqa: E402

A, B, C = 0.0035, 0.0025, 0.0025  # half length, half width, half thickness
E_END = 0.85  # < 1: blunt ends
HILUM_LEN, HILUM_HALF_W, HILUM_RAISE = 0.45, 0.10, 1.012  # fraction of length, half-width (rad), radial scale
NU, NV = 40, 48
MASS = 1e-4  # = RED_BEAN_PHYSICS particle_mass
COAT = (0.075, 0.006, 0.008)
HILUM = (0.80, 0.77, 0.68)


def spow(t, e):
    return np.sign(t) * np.abs(t) ** e


def surface(u, v):
    """u in [0, pi] along X, v around X (v = 0 on +Y)."""
    u, v = np.broadcast_arrays(u, v)
    s = spow(np.sin(u), E_END)
    return np.stack([A * spow(np.cos(u), E_END), B * s * np.cos(v), C * s * np.sin(v)], -1)


def grid_faces(nu, nv, closed_v):
    faces = []
    for i in range(nu - 1):
        for j in range(nv if closed_v else nv - 1):
            k = (j + 1) % nv
            faces.append([i * nv + j, i * nv + k, (i + 1) * nv + k, (i + 1) * nv + j])
    return faces


def build():
    """Visual parts [(pts, faces, material)] and the collision hull points."""
    u = np.linspace(0, np.pi, NU + 1)[1:-1]
    v = np.linspace(0, 2 * np.pi, NV, endpoint=False)
    rings = surface(u[:, None], v[None, :]).reshape(-1, 3)
    n = len(rings)
    pts = np.vstack([rings, [[A, 0, 0], [-A, 0, 0]]])
    faces = grid_faces(len(u), NV, True)
    faces += [[n, (j + 1) % NV, j] for j in range(NV)]
    last = (len(u) - 1) * NV
    faces += [[n + 1, last + j, last + (j + 1) % NV] for j in range(NV)]
    if mp.signed_volume(pts, faces) < 0:
        faces = [f[::-1] for f in faces]

    u0 = np.arccos(HILUM_LEN ** (1 / E_END))
    hu = np.linspace(u0, np.pi - u0, 32)
    x = spow(np.cos(hu), E_END) / HILUM_LEN
    half_w = HILUM_HALF_W * np.sqrt(np.clip(1 - x**2, 0.02, 1))
    hv = np.linspace(-1, 1, 7)[None, :] * half_w[:, None]
    h = surface(hu[:, None], hv).reshape(-1, 3) * [1, HILUM_RAISE, HILUM_RAISE]
    h_faces = grid_faces(len(hu), 7, False)
    p = h[h_faces[len(h_faces) // 2]]
    if np.dot(np.cross(p[1] - p[0], p[2] - p[0]), p.mean(0)) < 0:
        h_faces = [f[::-1] for f in h_faces]
    return [(pts, faces, "coat"), (h, h_faces, "hilum")], pts


def define_visual(st, root):
    """Bean meshes and materials under `root` (an existing Xform path) on stage `st`."""
    mats = {
        name: mp.add_material(
            st,
            f"{root}/_materials/{name.capitalize()}",
            "OmniPBR",
            {"diffuse_color_constant": rgb, "reflection_roughness_constant": rough, "metallic_constant": 0.0},
            rgb,
            1.0,
            rough,
        )
        for name, rgb, rough in (("coat", COAT, 0.22), ("hilum", HILUM, 0.55))
    }
    mp.add_xform(st, f"{root}/visual")
    parts, _ = build()
    for (pts, faces, name), prim in zip(parts, ("model", "hilum")):
        mesh = mp.add_mesh(st, f"{root}/visual/{prim}", pts, faces, mp.vertex_normals(pts, faces))
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mats[name])


def description(uid):
    caption = {
        "color": ["dark red", "maroon", "white stripe on the side"],
        "material": ["bean", "seed", "glossy seed coat"],
        "size": ["about 7 mm long", "tiny", "smaller than a fingertip"],
        "name": ["red bean", "adzuki bean", "azuki bean"],
        "shape": ["short rounded oblong", "blunt ends", "slightly flattened"],
        "function": ["cooking", "making red bean paste", "pouring between cups"],
    }
    desc = [
        "glossy dark red adzuki bean with a white stripe along one side",
        "small maroon bean, about 7 mm long",
        "single red bean with blunt rounded ends",
        "tiny oblong seed with a smooth shiny coat",
        "azuki bean as poured from one cup into another",
    ]
    return {"uuid": uid, "caption": caption, "description": desc}


def main(rigid_dir):
    parts, hull = build()
    for pts, faces, _ in parts[:1]:
        assert mp.signed_volume(pts, faces) > 0, "coat has inward-facing winding"
    tmp = Path(tempfile.mkdtemp()) / "object.usdc"
    st = Usd.Stage.CreateNew(str(tmp))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    root = mp.add_xform(st, "/root")
    st.SetDefaultPrim(root.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    define_visual(st, "/root")
    pc.add_collision(st, [hull])
    st.Save()
    d = Path(rigid_dir) / "red_bean" / "00000"
    d.mkdir(parents=True, exist_ok=True)
    zip_writer = getattr(Sdf, "ZipFileWriter", None) or Usd.ZipFileWriter
    zw = zip_writer.CreateNew(str(d / "object.usdz"))
    zw.AddFile(str(tmp), "object.usdc")
    zw.Save()

    uid = str(uuid.uuid5(uuid.NAMESPACE_URL, "robodojo/rigid/red_bean/0"))
    c = dict(pts=np.vstack([p for p, _, _ in parts]), faces=parts[0][1] + parts[1][1], contact_r=0.6 * C)
    meta = pc.metadata(uid, c, MASS, np.zeros(3))
    (d / "metadata.json").write_text(json.dumps(meta, indent=4))
    (d / "description.json").write_text(json.dumps(description(uid), indent=4))
    print(f"{d}: size {meta['physics']['size']}")


if __name__ == "__main__":
    try:
        main(sys.argv[1] if len(sys.argv) > 1 else pc.RIGID_DIR)
    except Exception:
        if mp.APP is None:
            raise
        traceback.print_exc()
        sys.stderr.flush()
        os._exit(1)
    if mp.APP is not None:
        mp.APP.close()
