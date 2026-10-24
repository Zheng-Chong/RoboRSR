"""Build the pantry-counter scene (real_scene2) as modular USD layers, from two phone photos (reference/photo.jpg from
the front right, reference/photo2.jpg from the front left).

Scene frame: Z up, meters. Origin on the floor below the countertop centre: the countertop spans x in [-L/2, L/2],
y in [-D/2, D/2] with the wall at y = D/2; +Y points from the robot side into the wall. A robot facing +Y stands at
y < -D/2. Same convention as real_scene1_office (real_scene1).

Sizes and placements come from fit_photo.json (both photos fitted jointly, scale from the microwave's 0.485 m width;
fit_photo.py puts the wall at y = 0.30, shifted here to D/2); decor on the walls is placed by back-projecting photo
pixels through the fitted cameras. Re-run with --scale-correction once a real length is measured
(measured_counter_length / COUNTER["length"]).
"""

import argparse
import json
import math
from pathlib import Path
import shutil
import sys

import cv2
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt
from shapely.geometry import box as sbox
import trimesh

HERE = Path(__file__).resolve().parent
PHOTOS = HERE / "reference"
OUT = HERE / "usd"
TEX = OUT / "textures"
FIT = json.loads((HERE / "fit_photo.json").read_text())
F = FIT["dims"]
FIT_WALL_Y = 0.30  # fit_photo.py frame
DY = F["depth"] / 2 - FIT_WALL_Y  # fit frame -> scene frame


def r3(v):
    return round(float(v), 3)


COUNTER = dict(
    length=r3(F["length"]),
    depth=r3(F["depth"]),
    top_z=r3(F["top_z"]),
    slab=r3(F["slab"]),  # the visible front edge (a dropped apron on a thinner slab)
    door_front=r3(-F["depth"] / 2 + 0.02),  # handleless doors, flush with the columns
    carcass_front=r3(-F["depth"] / 2 + 0.04),
    kick_h=0.08,
    doors=5,  # equal widths (photo2 door seams back-project 0.46-0.49 m apart)
    door_groove=0.025,  # finger groove between the slab and the door tops
    upstand_h=0.03,
)
SINK = dict(
    cx=r3(F["sink_cx"]), cy=r3(F["sink_cy"] + DY), sx=r3(F["sink_sx"]), sy=r3(F["sink_sy"]), depth=0.20, r=0.025
)
# The faucet stands on the back rim right of the sink centre (both photos); the fit sees its visible base front.
FAUCET = dict(x=r3(F["faucet_x"]), y=r3(SINK["cy"] + SINK["sy"] / 2 + 0.035), height=0.30, reach=0.17, r=0.012)
BASKET = dict(dx=0.12, dy=-0.02, r=0.09, h=0.085)  # strainer basket hanging in the sink's right half
MICROWAVE = dict(w=0.485, d=r3(F["mw_d"]), h=0.29, x0=r3(F["mw_x0"]), front=r3(F["mw_front"] + DY))
WALL = dict(y=COUNTER["depth"] / 2, height=r3(F["alcove_top"]))  # alcove ceiling (wood soffit) height
# Shelves run from the left column and stop short of the right one; z: top surface heights.
SHELVES = dict(
    x0=-COUNTER["length"] / 2,
    x1=r3(F["shelf_x1"]),
    depth=r3(F["shelf_depth"]),
    thickness=0.010,
    z=(r3(F["shelf_z1"]), r3(F["shelf_z2"])),
)
# left_w: neither counter photo sees the left column's far side; its front runs >= 0.71 m left of the counter in photo2
# and >= 0.99 m right of its left face in lounge2 (back-projected), so 1.0 m, estimated (was assumed 0.45 m)
COLUMNS = dict(left_w=1.0, left_front=r3(F["left_front"] + DY), right_front=r3(F["right_front"] + DY))
# Right of the counter a wall of tall wood cabinets runs on in the right column's plane, with a two-bay water
# dispenser (photo2 pixels back-projected onto that plane): x ranges, z range, recess depth.
CABINETS = dict(x1=4.2, seams_x=(2.10, 2.95), groove_x=2.45, upper_z=2.04, dispenser_top_z=1.55)
DISPENSER = dict(bays=((2.97, 3.40), (3.45, 3.85)), z=(0.78, 1.36), depth=0.30)
# Lounge left of the counter (fit_lounge.py, three photos; scale from its round table, top assumed 0.75 m): a wallpaper
# alcove between the counter's left column and a wood column further left, with round cafe tables in front. Its frame
# has x = 0 on the left column's left face and y = 0 on the wallpaper; here it is placed at LOUNGE_ORIGIN.
LF = json.loads((HERE / "fit_lounge.json").read_text())["dims"]
LOUNGE_ORIGIN = (r3(-COUNTER["length"] / 2 - COLUMNS["left_w"]), r3(COLUMNS["left_front"] + LF["col_depth"]))


def lounge_xy(x, y):
    return r3(x + LOUNGE_ORIGIN[0]), r3(y + LOUNGE_ORIGIN[1])


LOUNGE = dict(
    x0=lounge_xy(-LF["width"], 0)[0],
    x1=LOUNGE_ORIGIN[0],
    wall_y=LOUNGE_ORIGIN[1],
    wainscot_z=r3(LF["wainscot_z"]),
    paper_z=r3(LF["paper_z"]),
    band_z=r3(LF["band_z"]),
    soffit_front=r3(COLUMNS["left_front"]),
    left_col_w=0.60,  # its width is not in the photos
    left_col_front=lounge_xy(0, -LF["left_depth"])[1],
    skirting_h=0.08,
)
# Top of the room, back-projected through the lounge1 camera: the alcove's wood soffit (its underside lit warm by a
# hidden strip) runs from the wallpaper to the columns' front at band_z; a 10 cm wood fascia, then a 0.22 m white
# plaster strip, then the perforated wood ceiling (~3.2 m, estimated: only its edge is in the photos) with a dark
# downlight slot about 0.8 m out from the columns.
CEILING = dict(z=3.20, fascia_z=3.11, strip_w=0.22, slot_y=r3(COLUMNS["left_front"] - 0.80), slot_w=0.12)
DOWNLIGHT = dict(r=0.045, dx=1.0)
# Round cafe tables: wood top, brass column and base disc (2 cm high, under the chassis' 3.4 cm clearance). The second
# table (left, white chairs) is only partly in the photos: placed by eye, its size copied from the first.
TABLE = dict(top_z=0.75, top_r=r3(LF["table_r"]), top_t=0.03, col_r=0.035, base_r=r3(LF["base_r"]), base_h=0.02)
TABLES = {"blue": lounge_xy(LF["table_x"], LF["table_y"]), "white": lounge_xy(-4.0, -2.9)}
# Chairs (Rigid lounge_chair/<index>, scripts/objects/make_lounge_chair.py) face their table: (table, index, bearing
# deg of the chair seen from the table centre, distance). The blue four stand in a cross turned a little off the axes
# (floor contacts back-projected through the lounge3 camera, 0.45-0.9 m out: evened to 0.75 m); the white two by eye.
CHAIRS = [
    ("blue", 0, 185, 0.75),
    ("blue", 0, 265, 0.75),
    ("blue", 0, 350, 0.75),
    ("blue", 0, 85, 0.75),
    ("white", 1, 30, 0.75),
    ("white", 1, -30, 0.75),
]
# Two stainless pedal bins side by side against the wainscot (back-projected from lounge2 and lounge3), and a socket.
BINS = dict(x=lounge_xy(-4.1, 0)[0], w=0.36, d=0.30, h=0.62, gap=0.01)
LOUNGE_SOCKET = (lounge_xy(-0.6, 0)[0], 0.25)
ROOM = dict(x0=r3(LOUNGE["x0"] - LOUNGE["left_col_w"] - 2.5), x1=CABINETS["x1"], y0=-5.5)  # photos taken ~4.5 m out
POSTER = dict(x0=r3(F["poster_x0"]), x1=r3(F["poster_x1"]), z0=r3(F["poster_z0"]), z1=r3(F["poster_z1"]))
# name: (x, z centre, w, h); back-projected from both photos onto the wall
SOCKETS = {"single": (-0.96, 0.95, 0.086, 0.086), "double": (-0.30, 0.91, 0.17, 0.086)}
# Door 1 is the bin flap: an inset panel and its "垃圾投入桶" label (decal cut from photo2).
BIN_FLAP = dict(margin=0.07, bottom=0.20, label=(-1.10, -0.84, 0.47, 0.67))

# Shelf clutter (decor, static): name: (shape, x, y, size, material); size = (sx, sy, sz) for boxes, (r, h) for cyls.
SHELF_ITEMS = {
    "lower": {
        "tray": ("box", -0.68, 0.17, (0.10, 0.12, 0.015), "plastic_black"),
        "container_a": ("box", -0.58, 0.17, (0.12, 0.10, 0.045), "plastic_clear"),
        "container_b": ("box", -0.44, 0.17, (0.12, 0.10, 0.045), "plastic_clear"),
        "tea_bottle": ("cyl", -0.31, 0.18, (0.032, 0.22), "bottle_green"),
        "cloth_holder": ("cyl", -0.12, 0.20, (0.035, 0.12), "glass_dark"),
        "cloth": ("box", -0.12, 0.19, (0.10, 0.06, 0.15), "cloth_yellow"),
        "small_bottle": ("cyl", 0.19, 0.20, (0.022, 0.12), "plastic_clear"),
        "toothpaste": ("box", 0.27, 0.12, (0.15, 0.035, 0.03), "tube_white"),
        "food_box": ("box", 0.60, 0.17, (0.40, 0.17, 0.09), "plastic_clear"),
    },
    "upper": {
        "paper_bag_small": ("box", -0.57, 0.20, (0.14, 0.10, 0.20), "paper_bag"),
        "sponge": ("box", -0.27, 0.15, (0.09, 0.06, 0.02), "sponge_yellow"),
        "paper_bag_large": ("box", 0.40, 0.18, (0.26, 0.15, 0.30), "paper_bag"),  # in front of the poster's left
    },
}

# Not calibrated: 9000 / 4500 K / 250 overexposed the marble and turned the wood orange in the first RTX render.
LIGHTS = dict(
    panels=[(-0.6, -0.55), (0.6, -0.55), (0.0, -1.8), (TABLES["blue"][0], TABLES["blue"][1] + 0.4), (-5.0, -1.6)],
    panel_size=(1.2, 0.6),
    intensity=6000.0,
    color_temperature=5000.0,
    dome=150.0,
)

SCALE = 1.0

# name: (diffuse linear RGB, roughness, metallic, opacity, texture)
MATERIALS = {
    "quartz": ((0.80, 0.80, 0.79), 0.30, 0.0, 1.0, None),
    "skirting": ((0.62, 0.60, 0.56), 0.50, 0.0, 1.0, None),
    "wood": ((1.0, 1.0, 1.0), 0.55, 0.0, 1.0, "wood.png"),
    "door_seam": ((0.05, 0.03, 0.02), 0.8, 0.0, 1.0, None),
    "marble": ((1.0, 1.0, 1.0), 0.25, 0.0, 1.0, "marble.png"),
    "terrazzo": ((1.0, 1.0, 1.0), 0.45, 0.0, 1.0, "terrazzo.png"),
    "wall_paint": ((0.72, 0.71, 0.68), 0.85, 0.0, 1.0, None),
    "steel": ((0.62, 0.62, 0.64), 0.30, 1.0, 1.0, None),
    "chrome": ((0.70, 0.70, 0.72), 0.12, 1.0, 1.0, None),
    "shelf_steel": ((0.22, 0.21, 0.20), 0.35, 0.6, 1.0, None),
    "basket_mesh": ((0.35, 0.35, 0.36), 0.45, 1.0, 0.6, None),
    "dispenser_back": ((0.55, 0.56, 0.58), 0.25, 1.0, 1.0, None),
    "bin_label": ((1.0, 1.0, 1.0), 0.6, 0.0, 1.0, "bin_label.png"),
    "microwave_black": ((0.015, 0.015, 0.017), 0.20, 0.0, 1.0, None),
    "microwave_front": ((1.0, 1.0, 1.0), 0.08, 0.0, 1.0, "microwave_front.png"),
    "poster": ((1.0, 1.0, 1.0), 0.6, 0.0, 1.0, "poster.png"),
    "frame_white": ((0.82, 0.82, 0.80), 0.4, 0.0, 1.0, None),
    "socket_white": ((0.84, 0.84, 0.82), 0.4, 0.0, 1.0, None),
    "plastic_black": ((0.03, 0.03, 0.03), 0.4, 0.0, 1.0, None),
    "plastic_clear": ((0.80, 0.82, 0.82), 0.10, 0.0, 0.35, None),
    "bottle_green": ((0.62, 0.70, 0.35), 0.08, 0.0, 0.55, None),
    "glass_dark": ((0.05, 0.22, 0.18), 0.10, 0.0, 0.8, None),
    "cloth_yellow": ((0.78, 0.55, 0.08), 0.95, 0.0, 1.0, None),
    "sponge_yellow": ((0.85, 0.72, 0.05), 0.9, 0.0, 1.0, None),
    "tube_white": ((0.85, 0.85, 0.85), 0.4, 0.0, 1.0, None),
    "paper_bag": ((0.84, 0.82, 0.77), 0.9, 0.0, 1.0, None),
    "card": ((0.88, 0.88, 0.86), 0.6, 0.0, 1.0, None),
    "wallpaper": ((1.0, 1.0, 1.0), 0.8, 0.0, 1.0, "wallpaper.png"),
    "green_band": ((1.0, 1.0, 1.0), 0.7, 0.0, 1.0, "green_band.png"),
    "brass": ((0.62, 0.48, 0.24), 0.30, 1.0, 1.0, None),
    "table_wood": ((0.42, 0.24, 0.10), 0.45, 0.0, 1.0, None),
    "wood_lit": ((1.0, 1.0, 1.0), 0.55, 0.0, 1.0, "wood.png"),
    "plaster_white": ((0.86, 0.86, 0.84), 0.9, 0.0, 1.0, None),
    "ceiling_wood": ((1.0, 1.0, 1.0), 0.8, 0.0, 1.0, "ceiling_wood.png"),
    "slot_dark": ((0.04, 0.04, 0.04), 0.6, 0.0, 1.0, None),
    "downlight": ((1.0, 1.0, 1.0), 0.3, 0.0, 1.0, "glow.png"),
}
CUTOUT = {"bin_label"}  # textures whose alpha cuts the surface (opacityThreshold)
# back-lit: the texture also drives emissiveColor (the green band, the soffit's underside, the downlights)
EMISSIVE = {"green_band", "wood_lit", "downlight"}
PHYS = {
    "default": (0.5, 0.45, 0.0),
    "quartz": (0.55, 0.45, 0.0),
    "steel": (0.40, 0.35, 0.0),
    "terrazzo": (0.7, 0.6, 0.0),
}


# ----------------------------------------------------------------------------- geometry helpers
def T(x=0.0, y=0.0, z=0.0, yaw=0.0):
    m = trimesh.transformations.rotation_matrix(math.radians(yaw), [0, 0, 1])
    m[:3, 3] = [x, y, z]
    return m


def box_xyz(x0, x1, y0, y1, z0, z1):
    return trimesh.creation.box(
        extents=[x1 - x0, y1 - y0, z1 - z0], transform=T((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
    )


def box_c(cx, cy, z0, sx, sy, sz):
    return box_xyz(cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, z0, z0 + sz)


def cyl(cx, cy, z0, r, h, sections=32):
    return trimesh.creation.cylinder(radius=r, height=h, sections=sections, transform=T(cx, cy, z0 + h / 2))


def quad(p0, p1, p2, p3):
    return trimesh.Trimesh(vertices=[p0, p1, p2, p3], faces=[[0, 1, 2], [0, 2, 3]], process=False)


def rounded_rect(cx, cy, sx, sy, r):
    return sbox(cx - sx / 2 + r, cy - sy / 2 + r, cx + sx / 2 - r, cy + sy / 2 - r).buffer(r, quad_segs=6)


def tube_along(points, r, sections=16):
    """Chain of cylinders through `points` (a bent pipe), with spheres at the joints."""
    parts = []
    for a, b in zip(points[:-1], points[1:]):
        a, b = np.asarray(a, float), np.asarray(b, float)
        seg = trimesh.creation.cylinder(radius=r, segment=[a, b], sections=sections)
        parts.append(seg)
        parts.append(trimesh.creation.icosphere(subdivisions=1, radius=r).apply_translation(b))
    return trimesh.util.concatenate(parts)


def swept_tube(points, r, sections=24):
    """One continuous pipe mesh swept along `points` (parallel-transport frames, capped ends)."""
    p = np.asarray(points, float)
    t = np.gradient(p, axis=0)
    t /= np.linalg.norm(t, axis=1, keepdims=True)
    n = np.cross(t[0], [1.0, 0.0, 0.0] if abs(t[0][0]) < 0.9 else [0.0, 1.0, 0.0])
    n /= np.linalg.norm(n)
    ang = np.linspace(0, 2 * math.pi, sections, endpoint=False)
    rings = []
    for i in range(len(p)):
        if i:
            n = n - np.dot(n, t[i]) * t[i]
            n /= np.linalg.norm(n)
        b = np.cross(t[i], n)
        rings.append(p[i] + r * (np.cos(ang)[:, None] * n + np.sin(ang)[:, None] * b))
    v = np.vstack(rings + [p[:1], p[-1:]])
    k, m = sections, len(p)
    faces = []
    for i in range(m - 1):
        for j in range(k):
            a, b2, c, d = i * k + j, i * k + (j + 1) % k, (i + 1) * k + j, (i + 1) * k + (j + 1) % k
            faces += [(a, b2, d), (a, d, c)]
    for j in range(k):
        faces.append((m * k, (j + 1) % k, j))
        faces.append((m * k + 1, (m - 1) * k + j, (m - 1) * k + (j + 1) % k))
    mesh = trimesh.Trimesh(v, faces, process=True)
    mesh.fix_normals()
    return mesh


# ----------------------------------------------------------------------------- USD helpers
class Layer:
    def __init__(self, path, root):
        self.path = path
        self.stage = Usd.Stage.CreateNew(str(path))
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(self.stage, 1.0)
        self.root = UsdGeom.Xform.Define(self.stage, f"/{root}")
        self.stage.SetDefaultPrim(self.root.GetPrim())
        self.rootpath = f"/{root}"
        self.mats, self.pmats = {}, {}

    def material(self, key):
        if key in self.mats:
            return self.mats[key]
        color, rough, metal, opacity, tex = MATERIALS[key]
        mat = UsdShade.Material.Define(self.stage, f"{self.rootpath}/Looks/{key}")
        sh = UsdShade.Shader.Define(self.stage, f"{self.rootpath}/Looks/{key}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        if opacity < 1.0:
            sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
            sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.49)
        if tex:
            st = UsdShade.Shader.Define(self.stage, f"{self.rootpath}/Looks/{key}/stReader")
            st.CreateIdAttr("UsdPrimvarReader_float2")
            st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            st.CreateOutput("result", Sdf.ValueTypeNames.Float2)
            tx = UsdShade.Shader.Define(self.stage, f"{self.rootpath}/Looks/{key}/Texture")
            tx.CreateIdAttr("UsdUVTexture")
            tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"./textures/{tex}")
            tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
            for w in ("wrapS", "wrapT"):
                tx.CreateInput(w, Sdf.ValueTypeNames.Token).Set("repeat")
            tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
            tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
            if key in EMISSIVE:
                sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
            if key in CUTOUT:
                tx.CreateOutput("a", Sdf.ValueTypeNames.Float)
                sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).ConnectToSource(tx.ConnectableAPI(), "a")
                sh.CreateInput("opacityThreshold", Sdf.ValueTypeNames.Float).Set(0.5)
        else:
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[key] = mat
        return mat

    def phys_material(self, key):
        if key in self.pmats:
            return self.pmats[key]
        sf, df, rs = PHYS.get(key, PHYS["default"])
        pm = UsdShade.Material.Define(self.stage, f"{self.rootpath}/PhysicsMaterials/{key}")
        api = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
        api.CreateStaticFrictionAttr(sf)
        api.CreateDynamicFrictionAttr(df)
        api.CreateRestitutionAttr(rs)
        self.pmats[key] = pm
        return pm

    def mesh(self, path, m, mat=None, smooth=False, uv=None, guide=False):
        m = m.copy()
        if smooth:
            m.merge_vertices()
        um = UsdGeom.Mesh.Define(self.stage, f"{self.rootpath}/{path}")
        pts = (m.vertices * SCALE).astype(np.float32)
        um.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
        um.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(m.faces)))
        um.CreateFaceVertexIndicesAttr(Vt.IntArray(m.faces.reshape(-1).tolist()))
        um.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        um.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))]))
        if smooth:
            um.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(m.vertex_normals.astype(np.float32)))
            um.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        else:
            um.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(np.repeat(m.face_normals, 3, axis=0).astype(np.float32)))
            um.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        if uv is not None:
            fv = m.vertices[m.faces.reshape(-1)]
            fn = np.repeat(m.face_normals, 3, axis=0)
            pv = UsdGeom.PrimvarsAPI(um).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
            )
            pv.Set(Vt.Vec2fArray.FromNumpy(uv(fv, fn).astype(np.float32)))
        if guide:
            um.CreatePurposeAttr(UsdGeom.Tokens.guide)
        elif mat:
            UsdShade.MaterialBindingAPI.Apply(um.GetPrim()).Bind(self.material(mat))
        return um

    def collider(self, path, m, approx="convexHull", pkey="default"):
        prim = self.mesh(path, m, guide=True).GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approx)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            self.phys_material(pkey), UsdShade.Tokens.weakerThanDescendants, "physics"
        )

    def save(self):
        self.stage.GetRootLayer().Save()


def planar_uv(tile_u, tile_v):
    """Box-projected UVs: each face uses the two world axes it is most parallel to (meters / tile size)."""

    def uv(fv, fn):
        a = np.abs(fn)
        out = np.empty((len(fv), 2))
        zf = a[:, 2] >= np.maximum(a[:, 0], a[:, 1])
        xf = ~zf & (a[:, 0] >= a[:, 1])
        yf = ~zf & ~xf
        out[zf] = np.c_[fv[zf, 0] / tile_u, fv[zf, 1] / tile_v]
        out[xf] = np.c_[fv[xf, 1] / tile_u, fv[xf, 2] / tile_v]
        out[yf] = np.c_[fv[yf, 0] / tile_u, fv[yf, 2] / tile_v]
        return out

    return uv


def rect_uv(u_axis, u0, u1, v0, v1):
    """Stretch a single image over a planar face: u along world axis `u_axis` in [u0, u1], v along z in [v0, v1]."""

    def uv(fv, fn):
        return np.c_[(fv[:, u_axis] - u0) / (u1 - u0), (fv[:, 2] - v0) / (v1 - v0)]

    return uv


# ----------------------------------------------------------------------------- textures (from the photo)
def flatten_light(img, sigma):
    img = img.astype(np.float32)
    g = cv2.GaussianBlur(img, (0, 0), sigma)
    return np.clip(img / np.maximum(g, 1) * g.reshape(-1, 3).mean(0), 0, 255).astype(np.uint8)


def warp(img, quad_px, size):
    w, h = size
    m = cv2.getPerspectiveTransform(np.float32(quad_px), np.float32([[0, 0], [w, 0], [w, h], [0, h]]))
    return cv2.warpPerspective(img, m, (w, h), flags=cv2.INTER_CUBIC)


def photo_px(photo, pts):
    """Pixels of scene-frame points in a reference photo, through its camera fitted by fit_photo.py."""
    cam, f = FIT[photo], FIT["focal_px"]
    k = np.array([[f, 0, 512], [0, f, 384], [0, 0, 1]], np.float64)
    fit_pts = np.array(pts, np.float64) - [0, DY, 0]
    px, _ = cv2.projectPoints(fit_pts, np.array(cam["rvec"]), np.array(cam["tvec"]), k, None)
    return px.reshape(-1, 2)


def rectify(photo, img, corners, size):
    """Warp the planar patch `corners` (scene frame: top-left, top-right, bottom-right, bottom-left) to `size`."""
    return warp(img, photo_px(photo, corners), size)


def wall_patch(x0, x1, z0, z1, y):
    return [(x0, y, z1), (x1, y, z1), (x1, y, z0), (x0, y, z0)]


def tiled_noise(rng, size, octaves):
    """Seamless fractal noise in [0, 1]: random grids upsampled with wrap-around."""
    out = np.zeros((size, size), np.float32)
    for o in octaves:
        n = 2**o
        g = rng.random((n, n)).astype(np.float32)
        g = np.pad(g, 2, mode="wrap")
        k = size / n
        up = cv2.resize(g, (int((n + 4) * k), int((n + 4) * k)), interpolation=cv2.INTER_CUBIC)
        out += up[int(2 * k) : int(2 * k) + size, int(2 * k) : int(2 * k) + size] / 2 ** (o - octaves[0])
    out -= out.min()
    return out / out.max()


def marble(size=1024, seed=7):
    """Procedural white marble with soft grey veins (the photo's wall is mostly hidden behind shelves and clutter)."""
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:size, 0:size] / size
    # long, nearly straight diagonal veins as on the photo's wall slabs; integer freqs keep the tile seamless
    veins = np.abs(np.sin((xx + yy + 0.25 * tiled_noise(rng, size, range(1, 5))) * math.pi))
    fine = np.abs(np.sin((xx * 2 + yy + 0.4 * tiled_noise(rng, size, range(2, 6))) * math.pi))
    fade = tiled_noise(rng, size, range(1, 3)) ** 2
    v = 1.0 - 0.16 * np.exp(-((veins / 0.08) ** 2)) * fade - 0.08 * np.exp(-((fine / 0.02) ** 2)) * (1 - fade)
    v -= 0.02 * tiled_noise(rng, size, range(3, 8))
    img = np.clip(v[..., None] * np.array([228, 229, 231], np.float32), 0, 255).astype(np.uint8)  # BGR
    return cv2.GaussianBlur(img, (0, 0), 4.0)


def terrazzo(photo, size=1024, seed=3):
    """Seamless terrazzo: base colour and chip palette sampled from the photo's floor."""
    rng = np.random.default_rng(seed)
    floor = photo[650:768, 0:150].reshape(-1, 3).astype(np.float32)
    lum = floor.mean(1)
    base = np.median(floor[lum > np.percentile(lum, 40)], 0)
    base = 0.75 * base + 0.25 * base.mean()  # the near floor is lit warm by the wood cabinets; desaturate a bit
    chips = [
        floor[lum < np.percentile(lum, 8)].mean(0),
        floor[(lum > np.percentile(lum, 10)) & (lum < np.percentile(lum, 30))].mean(0),
        np.array([235, 235, 232], np.float32),
    ]
    img = np.empty((size, size, 3), np.float32)
    img[:] = base
    img *= (0.96 + 0.08 * tiled_noise(rng, size, range(2, 6)))[..., None]
    for _ in range(6000):  # 1024 px per 0.8 m tile: chips of ~3-12 mm as in the photo
        x, y = rng.integers(0, size, 2)
        r = int(rng.choice([2, 3, 3, 4, 5, 7]))
        col = chips[rng.choice(3, p=[0.55, 0.30, 0.15])] * rng.uniform(0.85, 1.1)
        for dx in (-size, 0, size):
            for dy in (-size, 0, size):
                cv2.ellipse(
                    img,
                    (int(x + dx), int(y + dy)),
                    (r, max(1, int(r * rng.uniform(0.5, 1)))),
                    float(rng.uniform(0, 180)),
                    0,
                    360,
                    col.tolist(),
                    -1,
                )
    return np.clip(cv2.GaussianBlur(img, (0, 0), 0.8), 0, 255).astype(np.uint8)


def poster_texture(im1, im2, size=(512, 790)):
    """photo2 sees the whole poster (low-res, oblique); photo.jpg its bottom fifth sharper. Blend photo.jpg in where it
    has valid pixels right of the paper bag standing in front of the poster."""
    p, y = POSTER, WALL["y"] - 0.0125
    corners = wall_patch(p["x0"], p["x1"], p["z0"], p["z1"], y)
    t2 = rectify("photo2.jpg", im2, corners, size).astype(np.float32)
    t1 = rectify("photo.jpg", im1, corners, size).astype(np.float32)
    valid = np.zeros(im1.shape[:2], np.float32)
    valid[2:, 606:] = 1.0  # the bag covers x < 605 in photo.jpg
    w1 = cv2.GaussianBlur(cv2.erode(rectify("photo.jpg", valid, corners, size), np.ones((9, 9))), (0, 0), 6)[..., None]
    m = w1[..., 0] > 0.9
    if m.any():  # exposure match on the overlap
        t1 *= np.median(t2[m], 0) / np.maximum(np.median(t1[m], 0), 1)
    return np.clip(w1 * t1 + (1 - w1) * t2, 0, 255).astype(np.uint8)


def bin_label_texture(im2, size=(260, 200)):
    """The bin flap's icon and text from photo2 as a cutout decal: alpha where darker than the surrounding wood."""
    x0, x1, z0, z1 = BIN_FLAP["label"]
    patch = rectify("photo2.jpg", im2, wall_patch(x0, x1, z0, z1, COUNTER["door_front"]), size)
    lum = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY).astype(np.float32)
    bg = cv2.medianBlur(lum.astype(np.uint8), 41).astype(np.float32)
    alpha = np.clip((bg - lum - 18) * 12, 0, 255).astype(np.uint8)
    ink = np.full_like(patch, (70, 72, 74))  # BGR dark grey print
    return np.dstack([ink, alpha])


def lounge_texture(x0, x1, z0, z1, size):
    """Wall patch of the lounge (scene frame x, z on the wallpaper plane) rectified from lounge1, which sees the whole
    wall: from where it was taken the counter's left column hides the last ~10 cm at the right end, so the patch stops
    LOUNGE_HIDDEN short of it and is stretched over the full width."""
    fit = json.loads((HERE / "fit_lounge.json").read_text())
    cam, f = fit["cameras"]["lounge1.jpg"], fit["focal_px"]
    k = np.array([[f, 0, 512], [0, f, 384], [0, 0, 1]], np.float64)
    ox = LOUNGE_ORIGIN[0]
    x1 = min(x1, ox - LOUNGE_HIDDEN)
    pts = np.array([(x0 - ox, 0, z1), (x1 - ox, 0, z1), (x1 - ox, 0, z0), (x0 - ox, 0, z0)], np.float64)
    px, _ = cv2.projectPoints(pts, np.array(cam["rvec"]), np.array(cam["tvec"]), k, None)
    im = cv2.imread(str(PHOTOS / "lounge1.jpg"))
    return flatten_light(warp(im, px.reshape(-1, 2), size), 60)


LOUNGE_HIDDEN = 0.12


def build_textures():
    TEX.mkdir(parents=True, exist_ok=True)
    lg = LOUNGE
    cv2.imwrite(
        str(TEX / "wallpaper.png"), lounge_texture(lg["x0"], lg["x1"], lg["wainscot_z"], lg["paper_z"], (2048, 620))
    )
    cv2.imwrite(
        str(TEX / "green_band.png"), lounge_texture(lg["x0"], lg["x1"], lg["paper_z"], lg["band_z"], (2048, 240))
    )
    im = cv2.imread(str(PHOTOS / "photo.jpg"))
    im2 = cv2.imread(str(PHOTOS / "photo2.jpg"))
    wood = flatten_light(im[30:740, 845:1015], 40)
    hsv = cv2.cvtColor(wood, cv2.COLOR_BGR2HSV).astype(np.float32)
    hsv[..., 1] *= 0.85  # RTX under warm light rendered the photo-sampled oak too orange
    wood = cv2.cvtColor(np.clip(hsv, 0, 255).astype(np.uint8), cv2.COLOR_HSV2BGR)
    cv2.imwrite(str(TEX / "wood.png"), cv2.resize(wood, (256, 1024)))
    cv2.imwrite(str(TEX / "terrazzo.png"), terrazzo(im))
    cv2.imwrite(str(TEX / "marble.png"), marble())
    cv2.imwrite(
        str(TEX / "microwave_front.png"), warp(im, [(614, 472), (812, 497), (812, 640), (619, 585)], (512, 300))
    )
    cv2.imwrite(str(TEX / "poster.png"), poster_texture(im, im2))
    cv2.imwrite(str(TEX / "bin_label.png"), bin_label_texture(im2))
    cv2.imwrite(str(TEX / "ceiling_wood.png"), perforated_wood())
    cv2.imwrite(str(TEX / "glow.png"), np.full((8, 8, 3), 255, np.uint8))


def perforated_wood(size=512, seed=5):
    """One 0.6 m acoustic ceiling panel: pale oak (the lounge1 ceiling's colour), fine grain, a 15 mm grid of 3 mm
    holes and a dark joint along two edges, so that tiling shows the panel joints."""
    rng = np.random.default_rng(seed)
    grain = cv2.GaussianBlur(rng.normal(0, 1, (size, size // 8)).astype(np.float32), (0, 0), 1.5)
    grain = cv2.resize(grain, (size, size), interpolation=cv2.INTER_CUBIC)
    base = np.array([118, 168, 205], np.float32)  # BGR
    im = np.clip(base[None, None] * (1 + 0.04 * grain[..., None]), 0, 255)
    pitch = size / 40
    yy, xx = np.mgrid[0:size, 0:size]
    holes = np.hypot((xx % pitch) - pitch / 2, (yy % pitch) - pitch / 2) < 1.3
    im[holes] *= 0.35
    im[:3], im[:, :3] = im[:3] * 0.45, im[:, :3] * 0.45
    return im.astype(np.uint8)


# ----------------------------------------------------------------------------- scene parts
def add_counter(L):
    c, s = COUNTER, SINK
    hl, d2, top = c["length"] / 2, c["depth"] / 2, c["top_z"]
    z_slab = top - c["slab"]
    hole = rounded_rect(s["cx"], s["cy"], s["sx"], s["sy"], s["r"])
    slab = sbox(-hl, -d2, hl, d2).difference(hole)
    L.mesh("Top", trimesh.creation.extrude_polygon(slab, c["slab"]).apply_translation([0, 0, z_slab]), "quartz")
    L.mesh("Upstand", box_xyz(-hl, hl, d2 - 0.015, d2, top, top + c["upstand_h"]), "quartz")
    hx, hy = s["sx"] / 2, s["sy"] / 2
    z0 = z_slab - s["depth"]
    # carcass hollowed above the sink bowl, otherwise its top shows through the opening
    for i, (x0, x1, y0, y1, zt) in enumerate(
        [
            (-hl, s["cx"] - hx, c["carcass_front"], d2, z_slab),
            (s["cx"] + hx, hl, c["carcass_front"], d2, z_slab),
            (s["cx"] - hx, s["cx"] + hx, c["carcass_front"], s["cy"] - hy, z_slab),
            (s["cx"] - hx, s["cx"] + hx, s["cy"] + hy, d2, z_slab),
            (s["cx"] - hx, s["cx"] + hx, s["cy"] - hy, s["cy"] + hy, z0 - 0.002),
        ]
    ):
        L.mesh(f"Carcass/part_{i}", box_xyz(x0, x1, y0, y1, c["kick_h"], zt), "wood", uv=planar_uv(0.25, 1.0))
    L.mesh("Kick", box_xyz(-hl, hl, c["carcass_front"] + 0.03, d2, 0.0, c["kick_h"]), "skirting")
    w = c["length"] / c["doors"]
    door_top = z_slab - c["door_groove"]
    for i in range(c["doors"]):
        x0 = -hl + i * w
        L.mesh(
            f"Doors/door_{i}",
            box_xyz(x0 + 0.0015, x0 + w - 0.0015, c["door_front"], c["carcass_front"], c["kick_h"] + 0.002, door_top),
            "wood",
            uv=planar_uv(0.25, 1.0),
        )
    L.mesh(
        "Doors/groove", box_xyz(-hl, hl, c["carcass_front"] - 0.001, c["carcass_front"], door_top, z_slab), "door_seam"
    )
    b, yf = BIN_FLAP, c["door_front"] - 0.0005
    fx0, fx1, fz0, fz1 = -hl + b["margin"], -hl + w - b["margin"], b["bottom"], door_top - b["margin"] + 0.01
    for nm, (gx0, gx1, gz0, gz1) in {
        "top": (fx0, fx1, fz1 - 0.003, fz1),
        "bottom": (fx0, fx1, fz0, fz0 + 0.003),
        "left": (fx0, fx0 + 0.003, fz0, fz1),
        "right": (fx1 - 0.003, fx1, fz0, fz1),
    }.items():
        L.mesh(f"Doors/bin_flap_{nm}", box_xyz(gx0, gx1, yf, yf + 0.0005, gz0, gz1), "door_seam")
    lx0, lx1, lz0, lz1 = b["label"]
    L.mesh(
        "Doors/bin_label",
        quad((lx0, yf - 0.0003, lz0), (lx1, yf - 0.0003, lz0), (lx1, yf - 0.0003, lz1), (lx0, yf - 0.0003, lz1)),
        "bin_label",
        uv=rect_uv(0, lx0, lx1, lz0, lz1),
    )
    # sink basin: undermount stainless bowl hanging below the slab opening
    bowl = [box_xyz(s["cx"] - hx, s["cx"] + hx, s["cy"] - hy, s["cy"] + hy, z0, z0 + 0.003)]
    for x0, x1, y0, y1 in [
        (-hx, hx, -hy, -hy + 0.003),
        (-hx, hx, hy - 0.003, hy),
        (-hx, -hx + 0.003, -hy, hy),
        (hx - 0.003, hx, -hy, hy),
    ]:
        bowl.append(box_xyz(s["cx"] + x0, s["cx"] + x1, s["cy"] + y0, s["cy"] + y1, z0, z_slab))
    L.mesh("Sink/bowl", trimesh.util.concatenate(bowl), "steel")
    L.mesh("Sink/drain", cyl(s["cx"], s["cy"] + 0.08, z0 + 0.003, 0.04, 0.002), "chrome")
    for i, b in enumerate(bowl):
        L.collider(f"Colliders/sink_{i}", b, pkey="steel")
    k = BASKET
    bx, by, bz = s["cx"] + k["dx"], s["cy"] + k["dy"], z_slab - 0.01 - k["h"]
    basket = trimesh.util.concatenate(
        [
            trimesh.creation.annulus(k["r"] - 0.002, k["r"], k["h"], sections=40, transform=T(bx, by, bz + k["h"] / 2)),
            cyl(bx, by, bz, k["r"], 0.002, 40),
            trimesh.creation.annulus(k["r"], k["r"] + 0.008, 0.004, sections=40, transform=T(bx, by, bz + k["h"])),
        ]
    )
    L.mesh("Sink/basket", basket, "basket_mesh", smooth=True)
    L.collider("Colliders/sink_basket", cyl(bx, by, bz, k["r"], k["h"], 16), pkey="steel")
    # countertop collision: four slabs around the sink opening, plus the carcass below
    for i, (x0, x1, y0, y1) in enumerate(
        [
            (-hl, s["cx"] - hx, -d2, d2),
            (s["cx"] + hx, hl, -d2, d2),
            (s["cx"] - hx, s["cx"] + hx, -d2, s["cy"] - hy),
            (s["cx"] - hx, s["cx"] + hx, s["cy"] + hy, d2),
        ]
    ):
        L.collider(f"Colliders/top_{i}", box_xyz(x0, x1, y0, y1, z_slab, top), pkey="quartz")
    L.collider("Colliders/upstand", box_xyz(-hl, hl, d2 - 0.015, d2, top, top + c["upstand_h"]), pkey="quartz")
    for i, (x0, x1) in enumerate([(-hl, s["cx"] - hx), (s["cx"] + hx, hl)]):
        L.collider(f"Colliders/carcass_{i}", box_xyz(x0, x1, c["door_front"], d2, 0.0, z_slab - 0.001))
    L.collider(
        "Colliders/carcass_front",
        box_xyz(s["cx"] - hx, s["cx"] + hx, c["door_front"], s["cy"] - hy - 0.003, 0.0, z_slab - 0.001),
    )
    L.collider("Colliders/carcass_under_sink", box_xyz(s["cx"] - hx, s["cx"] + hx, s["cy"] - hy, d2, 0.0, z0 - 0.001))

    f = FAUCET
    z_arc = top + f["height"] - 0.08
    pts = [(f["x"], f["y"], z) for z in np.linspace(top + 0.05, z_arc, 12)[:-1]]
    for a in np.linspace(0, math.pi, 33):
        pts.append((f["x"], f["y"] - f["reach"] / 2 * (1 - math.cos(a)), z_arc + 0.08 * math.sin(a)))
    pts += [(f["x"], f["y"] - f["reach"], z) for z in np.linspace(z_arc, top + f["height"] - 0.16, 5)[1:]]
    L.mesh("Faucet/base", cyl(f["x"], f["y"], top, 0.025, 0.05), "chrome", smooth=True)
    L.mesh("Faucet/spout", swept_tube(pts, f["r"]), "chrome", smooth=True)
    L.mesh("Faucet/lever", box_c(f["x"] + 0.03, f["y"], top + 0.04, 0.06, 0.012, 0.012), "chrome")
    L.collider("Colliders/faucet_riser", box_c(f["x"], f["y"], top, 0.05, 0.05, f["height"]))
    L.collider("Colliders/faucet_arc", swept_tube(pts[11:], f["r"], sections=8).convex_hull)
    m = MICROWAVE
    x0, x1, y0, y1 = m["x0"], m["x0"] + m["w"], m["front"], m["front"] + m["d"]
    L.mesh("Microwave/body", box_xyz(x0, x1, y0 + 0.004, y1, top + 0.012, top + m["h"]), "microwave_black")
    L.mesh(
        "Microwave/front",
        quad((x0, y0, top + 0.012), (x1, y0, top + 0.012), (x1, y0, top + m["h"]), (x0, y0, top + m["h"])),
        "microwave_front",
        uv=rect_uv(0, x0, x1, top + 0.012, top + m["h"]),
    )
    for i, (fx, fy) in enumerate([(fx, fy) for fx in (x0 + 0.04, x1 - 0.04) for fy in (y0 + 0.05, y1 - 0.05)]):
        L.mesh(f"Microwave/foot_{i}", cyl(fx, fy, top, 0.012, 0.012, 12), "plastic_black")
    card = box_c(x0 + 0.17, y0 + 0.10, top + m["h"], 0.075, 0.004, 0.10)
    card.apply_transform(
        trimesh.transformations.rotation_matrix(math.radians(-12), [1, 0, 0], [x0 + 0.17, y0 + 0.10, top + m["h"]])
    )
    L.mesh("Microwave/card", card, "card")
    L.collider("Colliders/microwave", box_xyz(x0, x1, y0, y1, top, top + m["h"]))


def add_background(L):
    c, wl, cl, r, cb, dp = COUNTER, WALL, COLUMNS, ROOM, CABINETS, DISPENSER
    hl, H, wy = c["length"] / 2, wl["height"], wl["y"]
    lx = -hl - cl["left_w"]
    rf = cl["right_front"]
    col_back = max(wy, LOUNGE["wall_y"]) + 0.02  # the left column also bounds the lounge alcove, whose wall is deeper
    HR = room_height()
    L.mesh(
        "Floor",
        quad([r["x0"], r["y0"], 0], [r["x1"], r["y0"], 0], [r["x1"], col_back, 0], [r["x0"], col_back, 0]),
        "terrazzo",
        uv=lambda fv, fn: fv[:, :2] / 0.8,
    )
    L.collider(
        "Colliders/floor", box_xyz(r["x0"] - 1, r["x1"] + 1, r["y0"] - 1, col_back + 1, -0.10, 0.0), pkey="terrazzo"
    )
    L.mesh("Backsplash", box_xyz(-hl, hl, wy, wy + 0.02, c["top_z"], H), "marble", uv=planar_uv(1.2, 1.2))
    L.mesh("ColumnLeft", box_xyz(lx, -hl, cl["left_front"], col_back, 0.0, HR), "wood", uv=planar_uv(0.6, 2.4))
    # right cabinet wall, the dispenser bays cut into it
    wall = box_xyz(hl, cb["x1"], rf, wy + 0.02, 0.0, HR)  # up to the ceiling
    bays = [box_xyz(x0, x1, rf - 0.01, rf + dp["depth"], *dp["z"]) for x0, x1 in dp["bays"]]
    L.mesh("CabinetsRight/body", wall.difference(trimesh.util.concatenate(bays)), "wood", uv=planar_uv(0.6, 2.4))
    yf = rf - 0.0005
    seams = [(x, x, 0.0, H) for x in cb["seams_x"]]
    seams += [
        (hl, cb["seams_x"][1], cb["upper_z"], cb["upper_z"]),
        (cb["seams_x"][1], cb["x1"], *[cb["dispenser_top_z"]] * 2),
    ]
    for i, (x0, x1, z0, z1) in enumerate(seams):
        L.mesh(f"CabinetsRight/seam_{i}", box_xyz(x0 - 0.002, x1 + 0.002, yf, rf, z0 - 0.002, z1 + 0.002), "door_seam")
    L.mesh(
        "CabinetsRight/groove", box_xyz(cb["groove_x"] - 0.006, cb["groove_x"] + 0.006, yf, rf, 0.65, 1.35), "door_seam"
    )
    for i, (x0, x1) in enumerate(dp["bays"]):
        z0, z1 = dp["z"]
        L.mesh(
            f"Dispenser/back_{i}", box_xyz(x0, x1, rf + dp["depth"] - 0.01, rf + dp["depth"], z0, z1), "dispenser_back"
        )
        L.mesh(
            f"Dispenser/tray_{i}",
            box_xyz(x0 + 0.03, x1 - 0.03, rf + 0.05, rf + dp["depth"] - 0.01, z0, z0 + 0.02),
            "steel",
        )
        L.mesh(
            f"Dispenser/tap_{i}",
            box_c((x0 + x1) / 2, rf + dp["depth"] - 0.06, z1 - 0.16, 0.05, 0.10, 0.06),
            "plastic_black",
        )
    L.mesh(
        "Soffit", box_xyz(-hl, hl, min(cl["left_front"], rf), wy + 0.02, H, HR), "wood", uv=planar_uv(0.6, 2.4)
    )
    L.mesh("Ceiling", down_quad(r["x0"], r["x1"], r["y0"], col_back, HR), "ceiling_wood", uv=lambda fv, fn: fv[:, :2] / 0.6)
    # office wall beyond the lounge's left column (not in the photos), on the lounge wallpaper plane
    ly, lc = LOUNGE["wall_y"], LOUNGE["x0"] - LOUNGE["left_col_w"]
    L.mesh("WallLeft/upper", box_xyz(r["x0"], lc, ly, ly + 0.02, 0.0, HR), "wall_paint")
    L.mesh("WallLeft/skirting", box_xyz(r["x0"], lc, ly - 0.01, ly, 0.0, 0.06), "skirting")
    L.mesh("WallBack", box_xyz(r["x0"], r["x1"], r["y0"] - 0.1, r["y0"], 0.0, HR), "wall_paint")
    L.mesh("WallW", box_xyz(r["x0"] - 0.1, r["x0"], r["y0"], ly, 0.0, HR), "wall_paint")
    L.mesh("WallE", box_xyz(r["x1"], r["x1"] + 0.1, r["y0"], wy, 0.0, HR), "wall_paint")
    p = POSTER
    L.mesh(
        "Poster/frame",
        box_xyz(p["x0"] - 0.012, p["x1"] + 0.012, wy - 0.012, wy, p["z0"] - 0.012, p["z1"] + 0.012),
        "frame_white",
    )
    L.mesh(
        "Poster/print",
        quad(
            (p["x0"], wy - 0.0125, p["z0"]),
            (p["x1"], wy - 0.0125, p["z0"]),
            (p["x1"], wy - 0.0125, p["z1"]),
            (p["x0"], wy - 0.0125, p["z1"]),
        ),
        "poster",
        uv=rect_uv(0, p["x0"], p["x1"], p["z0"], p["z1"]),
    )
    for nm, (x, z, w, h) in SOCKETS.items():
        L.mesh(f"Sockets/{nm}", box_c(x, wy - 0.006, z - h / 2, w, 0.012, h), "socket_white")
    for nm, b in {
        "backsplash": (-hl, hl, wy, wy + 0.02, c["top_z"], H),
        "column_left": (lx, -hl, cl["left_front"], col_back, 0.0, HR),
        "cabinets_right": (hl, dp["bays"][0][0], rf, wy + 0.02, 0.0, H),
        "cabinets_right_far": (dp["bays"][0][0], cb["x1"], rf + dp["depth"], wy + 0.02, 0.0, H),
        "wall_left": (r["x0"], lc, ly, ly + 0.03, 0.0, HR),
        "wall_back": (r["x0"], r["x1"], r["y0"] - 0.1, r["y0"], 0.0, HR),
        "wall_w": (r["x0"] - 0.1, r["x0"], r["y0"], ly, 0.0, HR),
        "wall_e": (r["x1"], r["x1"] + 0.1, r["y0"], wy, 0.0, HR),
    }.items():
        L.collider(f"Colliders/{nm}", box_xyz(*b))

    s = SHELVES
    for level, z in zip(("lower", "upper"), s["z"]):
        sh = box_xyz(s["x0"], s["x1"], wy - s["depth"], wy, z - s["thickness"], z)
        L.mesh(f"Shelves/{level}", sh, "shelf_steel")
        L.collider(f"Colliders/shelf_{level}", sh)
        for nm, (shape, x, y, size, mat) in SHELF_ITEMS[level].items():
            if shape == "box":
                m = box_c(x, y, z, *size)
                L.collider(f"Colliders/{level}_{nm}", m)
            else:
                rr, hh = size
                m = cyl(x, y, z, rr, hh)
                L.mesh(
                    f"Shelves/{level}_items/{nm}_cap",
                    cyl(x, y, z + hh, rr * 0.55, 0.02, 16),
                    "bottle_green" if nm == "tea_bottle" else "plastic_clear",
                )
                L.collider(f"Colliders/{level}_{nm}", box_c(x, y, z, 2 * rr, 2 * rr, hh))
            L.mesh(f"Shelves/{level}_items/{nm}", m, mat, smooth=shape == "cyl")


def room_height():
    return CEILING["z"]


def down_quad(x0, x1, y0, y1, z):
    """Horizontal quad facing down (a ceiling seen from the room; culled when looking in from above)."""
    return quad((x0, y0, z), (x0, y1, z), (x1, y1, z), (x1, y0, z))


def round_table(L, name, cx, cy):
    t = TABLE
    z1 = t["top_z"] - t["top_t"]
    L.mesh(f"{name}/top", cyl(cx, cy, z1, t["top_r"], t["top_t"], 64), "table_wood", smooth=True)
    L.mesh(f"{name}/column", cyl(cx, cy, t["base_h"], t["col_r"], z1 - t["base_h"], 24), "brass", smooth=True)
    L.mesh(f"{name}/base", cyl(cx, cy, 0.0, t["base_r"], t["base_h"], 48), "brass", smooth=True)
    L.collider(f"Colliders/{name}_top", cyl(cx, cy, z1, t["top_r"], t["top_t"], 30))  # <= 64 hull vertices
    L.collider(f"Colliders/{name}_column", cyl(cx, cy, t["base_h"], t["col_r"], z1 - t["base_h"], 16))
    L.collider(f"Colliders/{name}_base", cyl(cx, cy, 0.0, t["base_r"], t["base_h"], 30))


def add_lounge(L):
    lg, b = LOUNGE, BINS
    x0, x1, y, sk = lg["x0"], lg["x1"], lg["wall_y"], lg["skirting_h"]
    wz, pz, bz = lg["wainscot_z"], lg["paper_z"], lg["band_z"]
    wood = planar_uv(0.6, 2.4)
    L.mesh("Alcove/skirting", box_xyz(x0, x1, y - 0.025, y, 0.0, sk), "skirting")
    L.mesh("Alcove/wainscot", box_xyz(x0, x1, y - 0.02, y, sk, wz), "wood", uv=wood)
    L.mesh("Alcove/trim", box_xyz(x0, x1, y - 0.025, y - 0.02, wz - 0.012, wz), "door_seam")
    L.mesh(
        "Alcove/wallpaper",
        quad((x0, y - 0.001, wz), (x1, y - 0.001, wz), (x1, y - 0.001, pz), (x0, y - 0.001, pz)),
        "wallpaper",
        uv=rect_uv(0, x0, x1, wz, pz),
    )
    L.mesh(
        "Alcove/green_band",
        quad((x0, y - 0.001, pz), (x1, y - 0.001, pz), (x1, y - 0.001, bz), (x0, y - 0.001, bz)),
        "green_band",
        uv=rect_uv(0, x0, x1, pz, bz),
    )
    L.mesh("Alcove/wall", box_xyz(x0, x1, y, y + 0.02, 0.0, room_height()), "wall_paint")
    lc, ce, front, top = x0 - lg["left_col_w"], CEILING, lg["soffit_front"], room_height()
    L.mesh("Alcove/soffit", box_xyz(x0, x1, front, y, bz, top), "wood", uv=wood)
    L.mesh("Alcove/soffit_lit", down_quad(x0, x1, front, y, bz - 0.001), "wood_lit", uv=wood)
    # white plaster strip in front of the soffit (and the left column), then the downlight slot in the ceiling
    L.mesh("Ceiling/strip", box_xyz(lc, x1, front - ce["strip_w"], front, ce["fascia_z"], top), "plaster_white")
    sy, sw = ce["slot_y"], ce["slot_w"]
    L.mesh("Ceiling/slot", box_xyz(lc, x1, sy - sw / 2, sy + sw / 2, top - 0.01, top), "slot_dark")
    for i, x in enumerate(np.arange(lc + DOWNLIGHT["dx"] / 2, x1, DOWNLIGHT["dx"])):
        L.mesh(f"Ceiling/downlight_{i}", cyl(x, sy, top - 0.013, DOWNLIGHT["r"], 0.003, 24), "downlight", smooth=True)
    L.mesh("Alcove/column_left", box_xyz(lc, x0, lg["left_col_front"], y + 0.02, 0.0, room_height()), "wood", uv=wood)
    L.collider("Colliders/alcove_wall", box_xyz(x0, x1, y - 0.025, y + 0.02, 0.0, room_height()))
    L.collider("Colliders/alcove_column_left", box_xyz(lc, x0, lg["left_col_front"], y + 0.02, 0.0, room_height()))
    sx, sz = LOUNGE_SOCKET
    L.mesh("Alcove/socket", box_c(sx, y - 0.026, sz - 0.043, 0.086, 0.006, 0.086), "socket_white")
    for name, (cx, cy) in TABLES.items():
        round_table(L, f"Table_{name}", cx, cy)
    for i, dx in enumerate((-(b["w"] + b["gap"]) / 2, (b["w"] + b["gap"]) / 2)):
        cx, cy = b["x"] + dx, y - 0.03 - b["d"] / 2
        L.mesh(f"Bins/body_{i}", box_c(cx, cy, 0.03, b["w"], b["d"], b["h"] - 0.05), "steel")
        L.mesh(f"Bins/lid_{i}", box_c(cx, cy, b["h"] - 0.02, b["w"] - 0.01, b["d"] - 0.01, 0.02), "plastic_black")
        L.mesh(f"Bins/foot_{i}", box_c(cx, cy, 0.0, b["w"] - 0.03, b["d"] - 0.03, 0.03), "plastic_black")
        L.collider(f"Colliders/bin_{i}", box_c(cx, cy, 0.0, b["w"], b["d"], b["h"]))


def chair_poses():
    """Lounge chair placements: (index, (x, y), yaw deg) in the scene frame, each turned to face its table (the chair
    asset faces +X)."""
    out = []
    for table, idx, bearing, dist in CHAIRS:
        tx, ty = TABLES[table]
        a = math.radians(bearing)
        out.append((idx, (r3(tx + dist * math.cos(a)), r3(ty + dist * math.sin(a))), (bearing + 180) % 360))
    return out


def add_chairs(L):
    """The lounge chairs at their nominal poses, for usd/preview.usda only (renders and the platform's 3D view): in the
    sim they are rigid layout objects (scripts/objects/make_lounge_chair.py, placed by scripts/g1d/build_asset.py), so
    the Room entry leaves them out. Same geometry and centring as the asset."""
    sys.path.insert(0, str(HERE.parents[2] / "scripts" / "objects"))
    import make_lounge_chair as chair

    MATERIALS["chair_oak"] = (chair.OAK, 0.5, 0.0, 1.0, None)
    for idx, (_, rgb) in chair.COLOURS.items():
        MATERIALS[f"chair_{idx}"] = (rgb, 0.45, 0.0, 1.0, None)
    parts = chair.build()["parts"]
    every = np.concatenate([p for p, _ in parts.values()])
    centre = (every.min(0) + every.max(0)) / 2 * [1, 1, 0]
    for i, (idx, (x, y), yaw) in enumerate(chair_poses()):
        rot = trimesh.transformations.rotation_matrix(math.radians(yaw), [0, 0, 1])[:3, :3]
        for name, (p, f) in parts.items():
            m = trimesh.Trimesh((p - centre) @ rot.T + [x, y, 0.0], f, process=False)
            L.mesh(f"chair_{i}/{name}", m, "chair_oak" if name.startswith("leg") else f"chair_{idx}", smooth=True)


def add_lights(L):
    lt = LIGHTS
    for i, (x, y) in enumerate(lt["panels"]):
        rl = UsdLux.RectLight.Define(L.stage, f"{L.rootpath}/CeilingPanel_{i}")
        rl.CreateWidthAttr(lt["panel_size"][0] * SCALE)
        rl.CreateHeightAttr(lt["panel_size"][1] * SCALE)
        rl.CreateIntensityAttr(lt["intensity"])
        rl.CreateEnableColorTemperatureAttr(True)
        rl.CreateColorTemperatureAttr(lt["color_temperature"])
        UsdGeom.Xformable(rl).AddTranslateOp().Set(Gf.Vec3d(x * SCALE, y * SCALE, (room_height() - 0.02) * SCALE))
    dome = UsdLux.DomeLight.Define(L.stage, f"{L.rootpath}/AmbientDome")
    dome.CreateIntensityAttr(lt["dome"])
    dome.CreateColorAttr(Gf.Vec3f(0.95, 0.95, 1.0))


SCENE_LAYERS = ("Background", "Counter", "Lounge", "Lights")


def main():
    global SCALE
    ap = argparse.ArgumentParser()
    ap.add_argument("--scale-correction", type=float, default=1.0)
    SCALE = ap.parse_args().scale_correction
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)
    build_textures()
    for name, fn in [
        ("background", add_background),
        ("counter", add_counter),
        ("lounge", add_lounge),
        ("lights", add_lights),
        ("chairs", add_chairs),
    ]:
        layer = Layer(OUT / f"{name}.usda", name.capitalize())
        fn(layer)
        layer.save()

    # scene.usda: what the sim's Room holds; preview.usda adds the chairs (layout objects in the sim) for renders and
    # the platform's 3D view (scripts/scenes/render_scenes.py, export_glb.py)
    for stage_name, names in (("scene", SCENE_LAYERS), ("preview", SCENE_LAYERS + ("Chairs",))):
        st = Usd.Stage.CreateNew(str(OUT / f"{stage_name}.usda"))
        UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(st, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(st, 1.0)
        world = UsdGeom.Xform.Define(st, "/World")
        st.SetDefaultPrim(world.GetPrim())
        UsdPhysics.Scene.Define(st, "/World/PhysicsScene").CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
        for nm in names:
            st.DefinePrim(f"/World/{nm}").GetReferences().AddReference(f"./{nm.lower()}.usda")
        st.GetRootLayer().Save()
    # RoboDojo Room entry (layout_manager.select_room loads the only .usd in Assets/Room/<name>/): no PhysicsScene,
    # the env owns it. Install by copying this directory to Assets/Room/real_scene2/.
    room = Usd.Stage.CreateNew(str(HERE / "real_scene2.usd"))
    UsdGeom.SetStageUpAxis(room, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(room, 1.0)
    scene = UsdGeom.Xform.Define(room, "/Scene")
    room.SetDefaultPrim(scene.GetPrim())
    for nm in SCENE_LAYERS:
        room.DefinePrim(f"/Scene/{nm}").GetReferences().AddReference(f"./usd/{nm.lower()}.usda")
    room.GetRootLayer().Save()

    k = SCALE
    params = dict(
        frame="Z-up, meters; origin on the floor under the countertop centre; +X along the counter; +Y into the wall",
        scale_correction=k,
        table=dict(
            top_z=COUNTER["top_z"] * k,
            length=COUNTER["length"] * k,
            width=COUNTER["depth"] * k,
            front_y=-COUNTER["depth"] / 2 * k,
            cabinet_front_y=COUNTER["door_front"] * k,
            slab=COUNTER["slab"] * k,
        ),
        columns={n: v * k for n, v in COLUMNS.items()},
        sink={n: v * k for n, v in SINK.items()},
        microwave={n: v * k for n, v in MICROWAVE.items()},
        shelf_top_z=[z * k for z in SHELVES["z"]],
        # the lounge for mobile tasks (layouts place the chairs, scripts/g1d/build_asset.py)
        lounge=dict(
            wall_y=LOUNGE["wall_y"] * k,
            x_range=[LOUNGE["x0"] * k, LOUNGE["x1"] * k],
            table_top_z=TABLE["top_z"] * k,
            table_r=TABLE["top_r"] * k,
            tables={n: [v * k for v in xy] for n, xy in TABLES.items()},
            chairs=[dict(index=i, xy=[v * k for v in xy], yaw_deg=yaw) for i, xy, yaw in chair_poses()],
        ),
        layers=dict(
            scene="scene.usda",
            background="background.usda",
            table="counter.usda",
            lounge="lounge.usda",
            lights="lights.usda",
            preview="preview.usda",
        ),
    )
    (OUT / "build_params.json").write_text(json.dumps(params, indent=1), encoding="utf-8")
    for f in OUT.glob("*.usda"):  # USD ends text layers with a blank line, which `git diff --check` rejects
        f.write_text(f.read_text().rstrip("\n") + "\n")
    print("wrote", OUT)


if __name__ == "__main__":
    main()
