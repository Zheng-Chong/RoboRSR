"""Build the lab operating-table scene (real_scene3) as modular USD layers, from four phone photos taken around the
table (reference/p1-p4.jpg, see fit_photo.py).

Scene frame: Z up, meters. Origin on the floor under the tabletop centre; +X along the long side; +Y towards the
modesty panel. The robot stands on the open -Y side facing +Y (same convention as real_scene1/2).

Sizes come from fit_photo.json (scale from the 0.75 m tabletop height prior). The tabletop and modesty panel carry
textures rectified from the photos; the floor is a carpet patch rectified at true scale; the walls are plain finishes
(paint, acoustic panels, glass) in colours sampled from the photos, with the doors, whiteboards and TV seen there
modelled on them. The G1-D parked in the corner is the team's URDF baked into a static layer (usd/g1d.usdc). Re-run
with --scale-correction once a real length is measured (measured_length / TABLE["length"]).
"""

import argparse
import json
import math
from pathlib import Path
import shutil
import xml.etree.ElementTree as ET

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


def r3(v):
    return round(float(v), 3)


TABLE = dict(
    length=r3(F["length"]),
    width=r3(F["width"]),
    top_z=r3(F["top_z"]),
    thick=r3(F["thick"]),
    corner_r=0.01,
)
GROMMET = dict(x=r3(F["grom_x"]), y=r3(F["grom_y"]), l=r3(F["grom_l"]), w=r3(F["grom_w"]))
PANEL = dict(half=r3(F["panel_x"]), y=r3(F["panel_y"]), z0=r3(F["panel_z"]), t=0.016)
# Vertical leg columns, V feet in the YZ plane, casters.
LEGS = dict(
    x=r3(F["col_x"]),
    y=r3(F["col_y"]),
    col=(0.05, 0.06),  # column section (x, y)
    hub_z=0.075,
    tip_z=0.055,
    caster_x=r3(F["caster_x"]),
    caster_y=r3(F["caster_y"]),
    wheel_r=0.025,
)
BEAM = dict(section=0.04)  # flip-top axle between the column tops; not directly visible, low confidence
# Walls from photo pixels back-projected onto the floor (fit cameras): -Y glass partition (p1), +Y wall behind the
# desk row (p3), -X acoustic panels (p2), +X wall (p4). Height not observed.
ROOM = dict(x0=-2.9, x1=3.3, y0=-2.45, y1=3.8, height=2.8)
# Wall finishes and fittings from photo pixels back-projected onto the wall planes. Spans of the full-height surface
# along each wall (x for south / north, y for west / east): -Y glass partition between white wall ends (p1), +Y white
# wall (p3), -X / +X dark acoustic fabric panels (p2, p4), white past the -X panels' +Y end (p2).
WALL_SPANS = {
    "south": [(1.30, ROOM["x1"], "paint"), (-1.56, 1.30, "glass"), (ROOM["x0"], -1.56, "paint")],
    "north": [(ROOM["x0"], ROOM["x1"], "paint")],
    "west": [(ROOM["y0"], 2.5, "acoustic"), (2.5, ROOM["y1"], "paint")],
    "east": [(ROOM["y0"], ROOM["y1"], "acoustic")],
}
MULLIONS = [1.30, 0.30, -0.63, -1.56]  # glass partition posts, x (p1)
DOOR = dict(x0=-2.77, x1=-1.87, height=2.1, handle_x=-1.97, handle_z=1.0)  # p1; 0.9 m, its -X edge is outside p1
# Whiteboards: (span along the wall, bottom of the black frame). The bottom edge is seen at 1.03-1.18 m on the side
# walls (p2 and p4 extrapolate badly at their edges) and 1.29 m on the +Y wall; the tops are above every photo (2.2 m
# assumed).
BOARDS = {"west": (-2.07, 2.48, 1.08), "east": (-2.13, 2.91, 1.08), "north": (-1.35, 1.16, 1.29)}
BOARD_TOP = 2.2
TV = dict(x0=0.20, x1=0.99, z0=1.03, z1=1.27)  # black screen on the +Y wall under the board (p3)
# The G1-D parked in the -X/+Y corner (p2 right edge, p3 left edge), facing -Y as in p3, arms hanging as in p3 (other
# joints at zero, lift at the bottom). Chassis front and +X side from p3 (front y~2.65, side x~-1.96); p2 puts it
# 0.4-0.7 m off, as with the other far corner furniture. Static: visual meshes decimated to G1D_FACES, box colliders.
G1D = dict(urdf=HERE.parents[1] / "robot" / "g1d_v4" / "g1_d_full_dex1_v4.urdf", x=-2.22, y=2.92, facing=-90)
G1D_POSE = {"left_elbow_joint": 1.2, "right_elbow_joint": 1.2}  # rad; at 0 the forearms point forward
G1D_FACES = 40000
G1D_MATERIALS = {  # URDF visual colour -> material
    (0.792, 0.82, 0.933): "g1d_shell",
    (0.7, 0.7, 0.7): "g1d_body",
    (0.2, 0.2, 0.2): "g1d_dark",
    (0.898, 0.918, 0.929): "g1d_light",
    (0.298, 0.298, 0.298): "g1d_pad",
}
# clean 0.6 m floor patch, right of the table in p4; gain: the photo pixels already carry the room light, used as albedo
# they rendered the carpet ~1.5x too bright against the tabletop
CARPET = dict(patch=(-0.45, 0.15, -1.2, -0.6), photo="p4.jpg", gain=0.67)

# Room furniture, placed from floor contacts and corners back-projected through the fitted phone cameras (PLAN.md);
# sizes are catalogue sizes checked against the photos. facing: world direction (deg from +X) of the front: the
# user side of desks, where a sitter looks for chairs, drawers and doors. Off by 5-15 cm, more in p2.
FURNITURE = [
    # desk row along the +Y wall (p3, the +X end in p4)
    dict(kind="desk", x=-0.64, y=3.15, facing=-90, length=1.65),
    dict(kind="desk", x=1.06, y=3.10, facing=-90, length=1.65),
    dict(kind="desk", x=2.57, y=3.30, facing=-90, length=1.3),
    dict(kind="chair", x=-0.64, y=2.29, facing=90),
    dict(kind="chair", x=0.66, y=2.46, facing=90),
    dict(kind="chair", x=1.26, y=2.12, facing=90),
    dict(kind="chair", x=2.20, y=1.78, facing=135),
    # desk pair along the -X wall (p2 only)
    dict(kind="desk", x=-2.50, y=-0.57, facing=0, length=1.3),
    dict(kind="desk", x=-2.50, y=0.75, facing=0, length=1.26),
    dict(kind="chair", x=-1.95, y=-0.55, facing=180),
    dict(kind="chair", x=-1.85, y=0.68, facing=180),
    dict(kind="pedestal", x=-2.225, y=1.60, facing=0),  # p2, p3; half under the desk, the box clear of it
    dict(kind="cabinet", x=1.775, y=-2.165, facing=90),  # p1, p4
    dict(kind="side_table", x=0.09, y=-2.07, facing=90),  # p1, against the glass
    dict(kind="round_table", x=2.05, y=0.07, facing=0),  # p4 only; height assumed 0.75
]

LIGHTS = dict(
    panels=[(-1.2, -1.2), (1.2, -1.2), (-1.2, 1.2), (1.2, 1.2)],
    panel_size=(1.2, 0.6),
    intensity=24000.0,  # tabletop / carpet brightness as in p3 under RTX (6000 / 12000 / 24000 swept)
    color_temperature=6500.0,  # the robot cameras have no white balance in sim; 5000 K rendered visibly yellow
    dome=150.0,
)

SCALE = 1.0


def lin(srgb):
    c = np.asarray(srgb, float) / 255
    return tuple(float(v) for v in np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4))


# name: (diffuse linear RGB, roughness, metallic, opacity, texture). Wall colours: sRGB medians of the photos, used as
# albedo like the photo textures.
MATERIALS = {
    "laminate": ((1.0, 1.0, 1.0), 0.35, 0.0, 1.0, "tabletop.png"),
    "edge_band": ((0.62, 0.61, 0.59), 0.45, 0.0, 1.0, None),
    "panel": ((1.0, 1.0, 1.0), 0.45, 0.4, 1.0, "panel.png"),
    "frame_black": ((0.025, 0.025, 0.027), 0.45, 0.3, 1.0, None),
    "plastic_black": ((0.02, 0.02, 0.02), 0.6, 0.0, 1.0, None),
    "carpet": ((1.0, 1.0, 1.0), 0.95, 0.0, 1.0, "carpet.png"),
    "ceiling": ((0.80, 0.80, 0.80), 0.9, 0.0, 1.0, None),
    "paint": (lin((170, 168, 155)), 0.85, 0.0, 1.0, None),
    "acoustic": (lin((58, 61, 63)), 0.95, 0.0, 1.0, None),
    "glass": (lin((104, 129, 112)), 0.05, 0.0, 1.0, None),  # opaque: nothing is modelled behind the partition
    "door": (lin((145, 134, 124)), 0.5, 0.0, 1.0, None),
    "whiteboard": (lin((166, 165, 149)), 0.25, 0.0, 1.0, None),
    "g1d_shell": ((0.792, 0.82, 0.933), 0.4, 0.3, 1.0, None),
    "g1d_body": ((0.7, 0.7, 0.7), 0.4, 0.3, 1.0, None),
    "g1d_dark": ((0.2, 0.2, 0.2), 0.5, 0.2, 1.0, None),
    "g1d_light": ((0.898, 0.918, 0.929), 0.4, 0.2, 1.0, None),
    "g1d_pad": ((0.298, 0.298, 0.298), 0.7, 0.0, 1.0, None),
    "desk_white": ((0.72, 0.72, 0.70), 0.4, 0.0, 1.0, None),
    "frame_white": ((0.55, 0.55, 0.53), 0.4, 0.3, 1.0, None),
    "chair_black": ((0.02, 0.02, 0.022), 0.7, 0.0, 1.0, None),
    "pedestal": ((0.66, 0.63, 0.55), 0.45, 0.0, 1.0, None),
    "cabinet": ((0.62, 0.62, 0.60), 0.4, 0.2, 1.0, None),
    "seam": ((0.05, 0.05, 0.05), 0.6, 0.0, 1.0, None),
    "cardboard": ((0.45, 0.27, 0.12), 0.8, 0.0, 1.0, None),
    "wood": ((0.60, 0.48, 0.32), 0.5, 0.0, 1.0, None),
    "frame_gray": ((0.18, 0.18, 0.19), 0.45, 0.4, 1.0, None),
}
WRAP = {"carpet.png": "mirror"}  # the patch is not seamless; mirroring hides the seams
PHYS = {
    "default": (0.5, 0.45, 0.0),
    "laminate": (0.5, 0.4, 0.0),
    "carpet": (0.9, 0.8, 0.0),
}


# ----------------------------------------------------------------------------- geometry helpers
def T(x=0.0, y=0.0, z=0.0):
    m = np.eye(4)
    m[:3, 3] = [x, y, z]
    return m


def box_xyz(x0, x1, y0, y1, z0, z1):
    return trimesh.creation.box(
        extents=[x1 - x0, y1 - y0, z1 - z0], transform=T((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)
    )


def quad(p0, p1, p2, p3):
    return trimesh.Trimesh(vertices=[p0, p1, p2, p3], faces=[[0, 1, 2], [0, 2, 3]], process=False)


def rounded_rect(cx, cy, sx, sy, r):
    return sbox(cx - sx / 2 + r, cy - sy / 2 + r, cx + sx / 2 - r, cy + sy / 2 - r).buffer(r, quad_segs=4)


def bar(a, b, sx, sy):
    """Box of section sx (x) by sy (y) whose axis runs from a to b (both ends horizontal)."""
    pts = [np.add(p, (dx, dy, 0)) for p in (a, b) for dx in (-sx / 2, sx / 2) for dy in (-sy / 2, sy / 2)]
    return trimesh.convex.convex_hull(np.array(pts))


def strut(a, b, w, h):
    """Box of width w (x) and height h (z) from a to b, ends cut vertically (V-foot arms)."""
    pts = [np.add(p, (dx, 0, dz)) for p in (a, b) for dx in (-w / 2, w / 2) for dz in (-h / 2, h / 2)]
    return trimesh.convex.convex_hull(np.array(pts))


def rod(a, b, w):
    """Square rod of side w from a to b, ends square to its axis."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    m = trimesh.creation.box(extents=[w, w, np.linalg.norm(b - a)])
    m.apply_transform(trimesh.geometry.align_vectors([0, 0, 1], b - a))
    return m.apply_translation((a + b) / 2)


def cyl(r, z0, z1, x=0.0, y=0.0):
    return trimesh.creation.cylinder(radius=r, height=z1 - z0, sections=24).apply_translation([x, y, (z0 + z1) / 2])


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
                tx.CreateInput(w, Sdf.ValueTypeNames.Token).Set(WRAP.get(tex, "clamp"))
            tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
            tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
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
            pv = UsdGeom.PrimvarsAPI(um).CreatePrimvar(
                "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.faceVarying
            )
            pv.Set(Vt.Vec2fArray.FromNumpy(uv(fv).astype(np.float32)))
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


def rect_uv(u_axis, u0, u1, v_axis, v0, v1):
    """Stretch one image over a planar face: u along world axis u_axis over [u0, u1], v along v_axis over [v0, v1]."""
    return lambda fv: np.c_[(fv[:, u_axis] - u0) / (u1 - u0), (fv[:, v_axis] - v0) / (v1 - v0)]


# ----------------------------------------------------------------------------- textures (from the photos)
def camera(photo):
    c, f = FIT[photo], FIT["focal_px"]
    return np.array(c["rvec"]), np.array(c["tvec"]), np.array([[f, 0, 512], [0, f, 384], [0, 0, 1]], np.float64)


def photo_px(photo, pts):
    rvec, tvec, k = camera(photo)
    px, _ = cv2.projectPoints(np.array(pts, np.float64), rvec, tvec, k, None)
    return px.reshape(-1, 2)


def rectify(photo, img, corners, size, interp=cv2.INTER_CUBIC):
    """Warp the planar patch `corners` (scene frame: top-left, top-right, bottom-right, bottom-left) to `size`."""
    w, h = size
    m = cv2.getPerspectiveTransform(np.float32(photo_px(photo, corners)), np.float32([[0, 0], [w, 0], [w, h], [0, h]]))
    return cv2.warpPerspective(img, m, (w, h), flags=interp, borderMode=cv2.BORDER_CONSTANT, borderValue=0)


def flatten_light(img, sigma, valid=None):
    """Divide out low-frequency shading, keep the mean colour."""
    img = img.astype(np.float32)
    v = np.ones(img.shape[:2], np.float32) if valid is None else valid.astype(np.float32)
    num = cv2.GaussianBlur(img * v[..., None], (0, 0), sigma)
    den = cv2.GaussianBlur(v, (0, 0), sigma)[..., None]
    g = num / np.maximum(den, 1e-3)
    mean = img[v > 0].mean(0)
    return np.clip(img / np.maximum(g, 1) * mean, 0, 255)


def tabletop_texture(imgs, size=(1370, 686)):
    """Per-pixel median of the tabletop rectified from all four photos: drops view-dependent glare and shadows."""
    t, g = TABLE, GROMMET
    hl, hw, z = t["length"] / 2, t["width"] / 2, t["top_z"]
    corners = [(-hl, hw, z), (hl, hw, z), (hl, -hw, z), (-hl, -hw, z)]  # u -> +X, image rows -> -Y
    w, h = size
    shade_ref = np.ones((h, w), bool)  # shading is estimated on the bare laminate only (else the grommet halos)
    gx0, gx1 = [int((g["x"] + s * (g["l"] / 2 + 0.02) + hl) / t["length"] * w) for s in (-1, 1)]
    gy0, gy1 = [int((hw - g["y"] - s * (g["w"] / 2 + 0.02)) / t["width"] * h) for s in (1, -1)]
    shade_ref[gy0:gy1, gx0:gx1] = False
    layers = []
    for name, img in imgs.items():
        ones = np.full(img.shape[:2], 255, np.uint8)
        valid = cv2.erode(rectify(name, ones, corners, size, cv2.INTER_NEAREST), np.ones((9, 9))) > 0
        flat = flatten_light(rectify(name, img, corners, size), 25, valid & shade_ref)
        flat[~valid] = np.nan
        layers.append(flat)
    tex = np.nanmedian(np.stack(layers), axis=0)
    tex *= 222.0 / np.nanmedian(tex.mean(2))  # white laminate, sRGB ~222 in the photos
    tex = np.clip(np.nan_to_num(tex, nan=222.0), 0, 255).astype(np.uint8)
    m = 10  # the outermost pixels pick up the edge band and the floor behind it
    tex[:m], tex[-m:], tex[:, :m], tex[:, -m:] = tex[m], tex[-m - 1], tex[:, m : m + 1], tex[:, -m - 1 : -m]
    return tex


def panel_texture(img, size=(1024, 236)):
    """The ribbed modesty panel's +Y face from p1 (taken square-on from that side); image left is +X."""
    p, zt = PANEL, TABLE["top_z"] - TABLE["thick"]
    y = p["y"] + p["t"] / 2
    corners = [(p["half"], y, zt), (-p["half"], y, zt), (-p["half"], y, p["z0"]), (p["half"], y, p["z0"])]
    tex = rectify("p1.jpg", img, corners, size)
    rows = np.median(tex, axis=1, keepdims=True)  # the ribs run along X: keep the row profile, drop the legs behind
    hidden = np.argmax(rows.mean(axis=(1, 2)) < 100)  # top rows show the tabletop edge band in front of the panel
    rows[:hidden] = rows[hidden]
    return np.repeat(rows, size[0], axis=1).astype(np.uint8)


def carpet_texture(img, size=(1024, 1024)):
    x0, x1, y0, y1 = CARPET["patch"]
    tex = rectify(CARPET["photo"], img, [(x0, y1, 0), (x1, y1, 0), (x1, y0, 0), (x0, y0, 0)], size)
    return np.clip(flatten_light(tex, 120) * CARPET["gain"], 0, 255).astype(np.uint8)


WALLS = {  # name: (u axis 0=x/1=y, u0, u1 as seen facing the wall, fixed coordinate)
    "south": (0, ROOM["x1"], ROOM["x0"], ROOM["y0"]),  # p1 looks -Y: image left is +X
    "north": (0, ROOM["x0"], ROOM["x1"], ROOM["y1"]),
    "west": (1, ROOM["y0"], ROOM["y1"], ROOM["x0"]),  # p2 looks -X: image left is -Y
    "east": (1, ROOM["y1"], ROOM["y0"], ROOM["x1"]),
}
WALL_COLLIDERS = {
    "south": (ROOM["x0"], ROOM["x1"], ROOM["y0"] - 0.1, ROOM["y0"]),
    "north": (ROOM["x0"], ROOM["x1"], ROOM["y1"], ROOM["y1"] + 0.1),
    "west": (ROOM["x0"] - 0.1, ROOM["x0"], ROOM["y0"], ROOM["y1"]),
    "east": (ROOM["x1"], ROOM["x1"] + 0.1, ROOM["y0"], ROOM["y1"]),
}


def wall_point(name, u, z):
    axis, _, _, c = WALLS[name]
    return (u, c, z) if axis == 0 else (c, u, z)


def on_wall(name, a, b, z0, z1, depth):
    """Box against the inner face of a wall: a..b along it (a < b), z0..z1 high, `depth` into the room."""
    axis, _, _, c = WALLS[name]
    d0, d1 = sorted((c, c + (depth if name in ("south", "west") else -depth)))
    return box_xyz(a, b, d0, d1, z0, z1) if axis == 0 else box_xyz(d0, d1, a, b, z0, z1)


def build_textures():
    TEX.mkdir(parents=True, exist_ok=True)
    imgs = {f"p{i}.jpg": cv2.imread(str(PHOTOS / f"p{i}.jpg")) for i in range(1, 5)}
    cv2.imwrite(str(TEX / "tabletop.png"), tabletop_texture(imgs))
    cv2.imwrite(str(TEX / "panel.png"), panel_texture(imgs["p1.jpg"]))
    cv2.imwrite(str(TEX / "carpet.png"), carpet_texture(imgs[CARPET["photo"]]))


# ----------------------------------------------------------------------------- scene parts
def add_table(L):
    t, g = TABLE, LEGS
    hl, hw, top = t["length"] / 2, t["width"] / 2, t["top_z"]
    z_under = top - t["thick"]
    outline = rounded_rect(0, 0, t["length"], t["width"], t["corner_r"])
    # slab 0.5 mm below the textured cap so the visible surface and the collider are both at top_z
    slab = trimesh.creation.extrude_polygon(outline, t["thick"] - 0.0005).apply_translation([0, 0, z_under])
    L.mesh("Top/slab", slab, "edge_band")
    v2, f2 = trimesh.creation.triangulate_polygon(outline)
    cap = trimesh.Trimesh(np.c_[v2, np.full(len(v2), top)], f2, process=False)
    if cap.face_normals[:, 2].mean() < 0:
        cap.invert()
    L.mesh("Top/surface", cap, "laminate", uv=rect_uv(0, -hl, hl, 1, -hw, hw))
    L.collider("Colliders/top", box_xyz(-hl, hl, -hw, hw, z_under, top), pkey="laminate")

    p = PANEL
    pz1 = z_under
    panel = box_xyz(-p["half"], p["half"], p["y"] - p["t"] / 2, p["y"] + p["t"] / 2, p["z0"], pz1)
    L.mesh("Panel/board", panel, "panel", uv=rect_uv(0, p["half"], -p["half"], 2, p["z0"], pz1))
    L.collider("Colliders/panel", panel)

    cx, cy = g["col"]
    s = BEAM["section"]
    beam = box_xyz(-g["x"], g["x"], g["y"] - s / 2, g["y"] + s / 2, z_under - s, z_under)
    L.mesh("Frame/beam", beam, "frame_black")
    L.collider("Colliders/beam", beam)
    for sx, side in ((1, "pos"), (-1, "neg")):
        # column from the beam down to the foot hub
        hub = (sx * g["x"], g["y"], g["hub_z"])
        col = bar((sx * g["x"], g["y"], z_under), hub, cx, cy)
        L.mesh(f"Legs/{side}/column", col, "frame_black")
        L.collider(f"Colliders/column_{side}", col)
        bracket = box_xyz(sx * g["x"] - 0.04, sx * g["x"] + 0.04, -hw + 0.06, hw - 0.06, z_under - 0.02, z_under)
        L.mesh(f"Legs/{side}/bracket", bracket, "frame_black")
        L.collider(f"Colliders/bracket_{side}", bracket)
        for sy, end in ((1, "pos"), (-1, "neg")):
            tip = (sx * g["caster_x"], sy * (g["caster_y"] + 0.03), g["tip_z"])
            arm = strut(hub, tip, 0.05, 0.03)
            L.mesh(f"Legs/{side}/foot_{end}", arm, "frame_black")
            L.collider(f"Colliders/foot_{side}_{end}", arm)
            wx, wy, r = sx * g["caster_x"], sy * g["caster_y"], g["wheel_r"]
            wheel = trimesh.creation.cylinder(
                radius=r,
                height=0.022,
                sections=20,
                transform=trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]),
            ).apply_translation([wx, wy, r])
            fork = box_xyz(wx - 0.02, wx + 0.02, wy - 0.02, wy + 0.02, 2 * r, g["tip_z"] - 0.015)
            L.mesh(f"Legs/{side}/caster_{end}", trimesh.util.concatenate([wheel, fork]), "plastic_black")
            L.collider(f"Colliders/caster_{side}_{end}", box_xyz(wx - 0.02, wx + 0.02, wy - r, wy + r, 0.0, g["tip_z"]))


# Furniture builders: local frame on the floor at the footprint centre, front at +X. Each returns
# ([(part, mesh, material)], [collider meshes]); colliders are boxes or small convex hulls.
def caster(x, y):
    return ("caster", cyl(0.025, 0.0, 0.05, x, y), "plastic_black")


def desk(length, depth=0.6, height=0.75, thick=0.025):
    """Training desk: top on a splayed leg pair at each end (feet forward and back), casters, rails under the top."""
    hd, hl, zu = depth / 2, length / 2, height - thick
    top = box_xyz(-hd, hd, -hl, hl, zu, height)
    parts = [
        ("top", top, "desk_white"),
        ("rail_back", box_xyz(-hd + 0.05, -hd + 0.08, -hl + 0.06, hl - 0.06, zu - 0.06, zu), "frame_white"),
    ]
    colliders = [top]
    for sy in (1, -1):
        y = sy * (hl - 0.06)
        parts.append(("rail_end", box_xyz(-hd + 0.05, hd - 0.05, y - 0.015, y + 0.015, zu - 0.04, zu), "frame_white"))
        for sx in (1, -1):
            leg = rod((sx * (hd - 0.13), y, zu), (sx * (hd - 0.03), y, 0.05), 0.035)
            parts += [("leg", leg, "frame_white"), caster(sx * (hd - 0.03), y)]
            colliders.append(leg)
    return parts, colliders


def chair():
    """Training chair: mesh back, armrests, four splayed legs on casters."""
    back = trimesh.convex.convex_hull(
        np.array(
            [(x, y, z) for x, z in ((-0.25, 0.53), (-0.22, 0.53), (-0.32, 1.0), (-0.29, 1.0)) for y in (-0.23, 0.23)]
        )
    )
    seat = box_xyz(-0.22, 0.24, -0.24, 0.24, 0.42, 0.48)
    parts = [("seat", seat, "chair_black"), ("back", back, "chair_black")]
    parts.append(("column", rod((0, 0, 0.42), (0, 0, 0.34), 0.05), "chair_black"))
    for sy in (1, -1):
        y = sy * 0.27
        parts += [
            ("arm", rod((-0.27, y, 0.68), (0.17, y, 0.66), 0.03), "chair_black"),
            ("arm_post", rod((0.15, y, 0.66), (0.08, y, 0.48), 0.03), "chair_black"),
        ]
        for sx in (1, -1):
            parts += [
                ("leg", rod((sx * 0.06, sy * 0.06, 0.36), (sx * 0.29, sy * 0.27, 0.06), 0.03), "chair_black"),
                caster(sx * 0.29, sy * 0.27),
            ]
    return parts, [box_xyz(-0.32, 0.32, -0.30, 0.30, 0.0, 0.48), back]


def pedestal(width=0.48, depth=0.55, height=0.70):
    """Three-drawer pedestal on casters with a flat cardboard box on top."""
    hd, hw = depth / 2, width / 2
    body = box_xyz(-hd, hd, -hw, hw, 0.05, height)
    box = box_xyz(hd - 0.33, hd - 0.03, -0.21, 0.21, height, height + 0.09)
    parts = [("body", body, "pedestal"), ("box", box, "cardboard")]
    for z0, z1 in ((0.07, 0.37), (0.38, 0.53), (0.54, 0.685)):
        parts += [
            ("drawer", box_xyz(hd, hd + 0.006, -hw + 0.008, hw - 0.008, z0, z1), "pedestal"),
            ("seam", box_xyz(hd, hd + 0.003, -hw + 0.004, hw - 0.004, z0 - 0.004, z1 + 0.004), "seam"),
            ("pull", box_xyz(hd + 0.006, hd + 0.012, -0.08, 0.08, z1 - 0.03, z1 - 0.015), "seam"),
        ]
    parts += [caster(sx * (hd - 0.05), sy * (hw - 0.05)) for sx in (1, -1) for sy in (1, -1)]
    return parts, [box_xyz(-hd, hd, -hw, hw, 0.0, height), box]


def cabinet(width=0.89, depth=0.39, height=1.85):
    """Two-door steel file cabinet (1850 x 900 x 390 catalogue size; the top is outside both photos)."""
    hd, hw = depth / 2, width / 2
    body = box_xyz(-hd, hd, -hw, hw, 0.0, height)
    parts = [("body", body, "cabinet"), ("plinth", box_xyz(hd, hd + 0.002, -hw + 0.01, hw - 0.01, 0.0, 0.05), "seam")]
    for sy in (1, -1):
        y0, y1 = sorted((sy * 0.003, sy * (hw - 0.01)))
        parts += [
            ("door", box_xyz(hd, hd + 0.005, y0, y1, 0.06, height - 0.02), "cabinet"),
            ("seam", box_xyz(hd, hd + 0.003, y0 - 0.003, y1 + 0.003, 0.057, height - 0.017), "seam"),
            ("handle", box_xyz(hd + 0.005, hd + 0.02, sy * 0.03 - 0.008, sy * 0.03 + 0.008, 0.75, 0.90), "seam"),
        ]
    return parts, [body]


def side_table(length=1.12, depth=0.5, height=0.66):
    """Wood-top table on two T-shaped grey end frames."""
    hd, hl = depth / 2, length / 2
    top = box_xyz(-hd, hd, -hl, hl, height - 0.03, height)
    parts, colliders = [("top", top, "wood")], [top]
    for sy in (1, -1):
        y = sy * 0.49
        frame = [
            box_xyz(-0.025, 0.025, y - 0.025, y + 0.025, 0.03, height - 0.03),
            box_xyz(-hd + 0.03, hd - 0.03, y - 0.025, y + 0.025, 0.0, 0.03),
            box_xyz(-hd + 0.04, hd - 0.04, y - 0.02, y + 0.02, height - 0.07, height - 0.03),
        ]
        parts += [("frame", m, "frame_gray") for m in frame]
        colliders += frame[:2]
    return parts, colliders


def round_table(radius=0.38, height=0.75):
    """Round cafe table on a single column and a disc foot."""
    top, column, foot = cyl(radius, height - 0.03, height), cyl(0.04, 0.02, height - 0.03), cyl(0.22, 0.0, 0.02)
    return [("top", top, "desk_white"), ("column", column, "desk_white"), ("foot", foot, "desk_white")], [
        top,
        column,
        foot,
    ]


BUILDERS = dict(
    desk=desk, chair=chair, pedestal=pedestal, cabinet=cabinet, side_table=side_table, round_table=round_table
)


def furniture_parts():
    """[(name, [(part, mesh, material)], [colliders])] in the scene frame, one entry per FURNITURE item."""
    out, seen = [], {}
    for f in FURNITURE:
        seen[f["kind"]] = seen.get(f["kind"], -1) + 1
        parts, colliders = BUILDERS[f["kind"]](**{k: v for k, v in f.items() if k not in ("kind", "x", "y", "facing")})
        m = trimesh.transformations.rotation_matrix(math.radians(f["facing"]), [0, 0, 1])
        m[:2, 3] = f["x"], f["y"]
        out.append(
            (
                f"{f['kind']}_{seen[f['kind']]}",
                [(p, mesh.copy().apply_transform(m), mat) for p, mesh, mat in parts],
                [c.copy().apply_transform(m) for c in colliders],
            )
        )
    return out


def add_furniture(L):
    for name, parts, colliders in furniture_parts():
        for i, (part, mesh, mat) in enumerate(parts):
            L.mesh(f"Furniture/{name}/{part}_{i}", mesh, mat)
        for j, c in enumerate(colliders):
            L.collider(f"Furniture/{name}/collider_{j}", c)


def add_background(L):
    r, hgt = ROOM, ROOM["height"]
    L.mesh(
        "Floor",
        quad([r["x0"], r["y0"], 0], [r["x1"], r["y0"], 0], [r["x1"], r["y1"], 0], [r["x0"], r["y1"], 0]),
        "carpet",
        uv=lambda fv: (
            (fv[:, :2] - [CARPET["patch"][0], CARPET["patch"][2]]) / (CARPET["patch"][1] - CARPET["patch"][0])
        ),
    )
    # closes the room: without it RoboDojo's default brown studio HDR floods the scene from above
    L.mesh(
        "Ceiling",
        quad([r["x0"], r["y0"], hgt], [r["x0"], r["y1"], hgt], [r["x1"], r["y1"], hgt], [r["x1"], r["y0"], hgt]),
        "ceiling",
    )
    L.collider(
        "Colliders/floor", box_xyz(r["x0"] - 1, r["x1"] + 1, r["y0"] - 1, r["y1"] + 1, -0.10, 0.0), pkey="carpet"
    )
    for name, (axis, u0, u1, _) in WALLS.items():
        for i, (a, b, finish) in enumerate(WALL_SPANS[name]):
            if finish == "acoustic":
                L.mesh(f"Walls/{name}_skirting_{i}", on_wall(name, a, b, 0.0, 0.05, 0.01), "frame_black")
            if (b - a) * (u1 - u0) < 0:  # wound like the whole wall, so the face looks into the room
                a, b = b, a
            p, q = wall_point(name, a, 0), wall_point(name, b, 0)
            # metric UVs, for the platform's material swaps
            L.mesh(f"Walls/{name}_{i}", quad(p, q, (*q[:2], hgt), (*p[:2], hgt)), finish, uv=lambda v: v[:, [axis, 2]])
        L.collider(f"Colliders/wall_{name}", box_xyz(*WALL_COLLIDERS[name], 0.0, hgt))
    for i, x in enumerate(MULLIONS):
        L.mesh(f"Walls/south_mullion_{i}", on_wall("south", x - 0.02, x + 0.02, 0.0, hgt, 0.03), "frame_black")
    a, b = min(MULLIONS), max(MULLIONS)
    L.mesh("Walls/south_glass_rail", on_wall("south", a, b, 0.0, 0.05, 0.03), "frame_black")
    d = DOOR
    L.mesh("Walls/south_door", on_wall("south", d["x0"], d["x1"], 0.0, d["height"], 0.03), "door")
    hx, hz = d["handle_x"], d["handle_z"]
    handle = on_wall("south", hx - 0.07, hx + 0.07, hz - 0.012, hz + 0.012, 0.09)
    L.mesh("Walls/south_door_handle", handle, "frame_black")
    for name, (a, b, z) in BOARDS.items():
        L.mesh(f"Walls/{name}_board", on_wall(name, a, b, z + 0.08, BOARD_TOP, 0.02), "whiteboard")
        L.mesh(f"Walls/{name}_board_frame", on_wall(name, a, b, z, z + 0.08, 0.03), "frame_black")
        L.mesh(f"Walls/{name}_board_frame_top", on_wall(name, a, b, BOARD_TOP, BOARD_TOP + 0.03, 0.03), "frame_black")
    L.mesh("Walls/north_tv", on_wall("north", TV["x0"], TV["x1"], TV["z0"], TV["z1"], 0.06), "frame_black")
    add_furniture(L)
    g1d = UsdGeom.Xform.Define(L.stage, f"{L.rootpath}/Furniture/g1d_0")
    g1d.GetPrim().GetReferences().AddReference("./g1d.usdc")
    g1d.AddTranslateOp().Set(Gf.Vec3d(G1D["x"] * SCALE, G1D["y"] * SCALE, 0.0))
    g1d.AddRotateZOp().Set(G1D["facing"])


def urdf_origin(el):
    o = el.find("origin")
    o = {} if o is None else o.attrib
    xyz, rpy = ([float(v) for v in o.get(k, "0 0 0").split()] for k in ("xyz", "rpy"))
    m = trimesh.transformations.euler_matrix(*rpy, "sxyz")
    m[:3, 3] = xyz
    return m


def g1d_meshes():
    """{material: mesh} of the G1-D's URDF visuals at G1D_POSE (revolute joints), in its own frame (front +X), lowest
    point at z = 0, decimated to G1D_FACES triangles in all."""
    urdf = ET.parse(G1D["urdf"]).getroot()
    joint = {j.find("child").get("link"): j for j in urdf.iter("joint")}

    def pose(link):
        if link not in joint:
            return np.eye(4)
        j = joint[link]
        m = urdf_origin(j)
        if j.get("name") in G1D_POSE:
            axis = [float(v) for v in j.find("axis").get("xyz").split()]
            m = m @ trimesh.transformations.rotation_matrix(G1D_POSE[j.get("name")], axis)
        return pose(j.find("parent").get("link")) @ m

    groups = {}
    for link in urdf.iter("link"):
        for vis in link.iter("visual"):
            m = trimesh.load(G1D["urdf"].parent / vis.find("geometry/mesh").get("filename"), force="mesh")
            m.apply_transform(pose(link.get("name")) @ urdf_origin(vis))
            rgba = tuple(round(float(v), 3) for v in vis.find("material/color").get("rgba").split())
            groups.setdefault(G1D_MATERIALS[rgba[:3]], []).append(m)
    merged = {k: trimesh.util.concatenate(ms) for k, ms in groups.items()}
    total = sum(len(m.faces) for m in merged.values())
    target = {k: max(500, G1D_FACES * len(m.faces) // total) for k, m in merged.items()}
    out = {
        k: m.simplify_quadric_decimation(face_count=target[k]) if len(m.faces) > target[k] else m
        for k, m in merged.items()
    }
    z0 = min(m.bounds[0][2] for m in out.values())
    return {k: m.apply_translation([0.0, 0.0, -z0]) for k, m in out.items()}


def add_g1d(L):
    meshes = g1d_meshes()
    for k, m in meshes.items():
        L.mesh(f"Body/{k}", m, k, smooth=True)
    v = np.vstack([m.vertices for m in meshes.values()])
    for name, part in (("chassis", v[v[:, 2] < 0.25]), ("body", v[v[:, 2] >= 0.25])):
        (x0, y0, z0), (x1, y1, z1) = part.min(0), part.max(0)
        L.collider(f"Colliders/{name}", box_xyz(x0, x1, y0, y1, z0, z1))


def add_lights(L):
    lt = LIGHTS
    for i, (x, y) in enumerate(lt["panels"]):
        rl = UsdLux.RectLight.Define(L.stage, f"{L.rootpath}/CeilingPanel_{i}")
        rl.CreateWidthAttr(lt["panel_size"][0] * SCALE)
        rl.CreateHeightAttr(lt["panel_size"][1] * SCALE)
        rl.CreateIntensityAttr(lt["intensity"])
        rl.CreateEnableColorTemperatureAttr(True)
        rl.CreateColorTemperatureAttr(lt["color_temperature"])
        UsdGeom.Xformable(rl).AddTranslateOp().Set(Gf.Vec3d(x * SCALE, y * SCALE, (ROOM["height"] - 0.02) * SCALE))
    dome = UsdLux.DomeLight.Define(L.stage, f"{L.rootpath}/AmbientDome")
    dome.CreateIntensityAttr(lt["dome"])
    dome.CreateColorAttr(Gf.Vec3f(0.95, 0.95, 1.0))


def main():
    global SCALE
    ap = argparse.ArgumentParser()
    ap.add_argument("--scale-correction", type=float, default=1.0)
    SCALE = ap.parse_args().scale_correction
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)
    build_textures()
    layer = Layer(OUT / "g1d.usdc", "G1D")  # binary: 40k triangles; before background.usda, which references it
    add_g1d(layer)
    layer.save()
    for name, fn in [("background", add_background), ("table", add_table), ("lights", add_lights)]:
        layer = Layer(OUT / f"{name}.usda", name.capitalize())
        fn(layer)
        layer.save()

    st = Usd.Stage.CreateNew(str(OUT / "scene.usda"))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(st, 1.0)
    world = UsdGeom.Xform.Define(st, "/World")
    st.SetDefaultPrim(world.GetPrim())
    UsdPhysics.Scene.Define(st, "/World/PhysicsScene").CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    for nm in ("Background", "Table", "Lights"):
        st.DefinePrim(f"/World/{nm}").GetReferences().AddReference(f"./{nm.lower()}.usda")
    st.GetRootLayer().Save()
    # RoboDojo Room entry (layout_manager.select_room loads the only .usd in Assets/Room/<name>/): no PhysicsScene,
    # the env owns it. Install by copying this directory to Assets/Room/real_scene3/.
    room = Usd.Stage.CreateNew(str(HERE / "real_scene3.usd"))
    UsdGeom.SetStageUpAxis(room, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(room, 1.0)
    scene = UsdGeom.Xform.Define(room, "/Scene")
    room.SetDefaultPrim(scene.GetPrim())
    for nm in ("Background", "Table", "Lights"):
        room.DefinePrim(f"/Scene/{nm}").GetReferences().AddReference(f"./usd/{nm.lower()}.usda")
    room.GetRootLayer().Save()

    k = SCALE
    t = TABLE
    params = dict(
        frame="Z-up, meters; origin on the floor under the tabletop centre; +X along the table; +Y to the panel",
        scale_correction=k,
        table=dict(
            top_z=t["top_z"] * k,
            length=t["length"] * k,
            width=t["width"] * k,
            front_y=-t["width"] / 2 * k,
            thick=t["thick"] * k,
        ),
        grommet={n: v * k for n, v in GROMMET.items()},
        panel={n: v * k for n, v in PANEL.items()},
        legs={n: (v * k if isinstance(v, float) else v) for n, v in LEGS.items()},
        room={n: v * k for n, v in ROOM.items()},
        layers=dict(scene="scene.usda", background="background.usda", table="table.usda", lights="lights.usda"),
    )
    (OUT / "build_params.json").write_text(json.dumps(params, indent=1), encoding="utf-8")
    print("wrote", OUT)


if __name__ == "__main__":
    main()
