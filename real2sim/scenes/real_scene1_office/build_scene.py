"""Build the real_scene1 simulation scene as modular USD layers.

Scene frame: Z up, meters. Origin on the floor below the white task-table center.
+X runs along the table long edge (toward the perpendicular orange desk), +Y points from the
robot side of the table toward the sofa / window wall. A robot facing +Y at y < -0.31 faces the table.
All placements come from the SfM reconstruction of the observation video (see PLAN.md); the
"src" notes in LAYOUT say which measurement each number comes from.
"""
import argparse
import json
import math
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
import trimesh
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt
from shapely.geometry import box as sbox

HERE = Path(__file__).resolve().parent
OBS = HERE.parent
OUT = HERE / "usd"
TEX = OUT / "textures"

TABLE = dict(length=1.20, width=0.62, top_z=0.68, thickness=0.025, corner_r=0.05,
             column_x=0.42, foot_len=0.56)

# Object poses: base center (x, y) on the table top, yaw in degrees about +Z.
OBJECTS = {
    "blue_cup_a": dict(kind="cup", pos=(0.030, 0.028), yaw=0.0, height=0.166, r_top=0.044, r_bot=0.031,
                       wall=0.0015, mat="blue_cup", mass=0.022, friction=(0.45, 0.40)),
    "blue_cup_b": dict(kind="cup", pos=(-0.158, -0.175), yaw=0.0, height=0.166, r_top=0.044, r_bot=0.031,
                       wall=0.0015, mat="blue_cup", mass=0.022, friction=(0.45, 0.40)),
    "paper_cup": dict(kind="cup", pos=(0.165, -0.168), yaw=0.0, height=0.166, r_top=0.045, r_bot=0.030,
                      wall=0.0012, mat="paper_cup", mass=0.014, friction=(0.55, 0.50)),
    "tissue_pack": dict(kind="tissue", pos=(-0.406, -0.086), yaw=130.0, size=(0.140, 0.110, 0.083),
                        mat="tissue", mass=0.090, friction=(0.45, 0.40)),
    "plush_toy": dict(kind="toy", pos=(-0.238, 0.110), yaw=0.0, size=(0.120, 0.080, 0.097),
                      mat="plush", mass=0.035, friction=(0.85, 0.80)),
}

# Room and coarse furniture (meters, scene frame).
LAYOUT = dict(
    room=dict(x0=-2.65, x1=3.00, y0=-2.12, y1=1.98, height=2.80,
              src="W glass partition from point cloud x~-2.4..-2.7 and door-frame back-projection; "
                  "E wall from E-window sill start (2.92,1.29) seen in 3 views; N wall from sofa back; "
                  "S wall from TV corner triangulation (y=-2.11)"),
    north_window=dict(x1=1.28, sill_z=1.09, band_top=1.49, mullion_pitch=1.30, seams=(-1.36, -0.38, 0.61),
                      src="sill front edge and band top back-projected in v0160, checked by projection into v0260/p4 "
                          "(the 1.00/1.45 of the first build land below the sill there); dado seams at x -1.36/-0.38/0.61 "
                          "from p4, -0.38 also in v0160/v0260"),
    ne_block=dict(x0=1.28, y0=1.29, src="solid corner block between N window end and E window start"),
    east_window=dict(y0=-1.55, y1=1.29, sill_z=1.09, band_top=1.49, src="E sill start (2.92,1.29); y0 estimate; "
                                                                        "heights as the N window"),
    door=dict(y0=0.95, y1=1.88, height=2.10, src="door-frame bottom back-projected near (-2.6,1.65); width est."),
    tv=dict(x0=-2.50, x1=-0.60, z0=1.03, z1=2.10, src="+X bottom corner triangulated (-0.60,-2.11,1.02); width est. (86in)"),
    south_wall=dict(board_x1=-0.12, board_top=2.40, rail=(0.83, 0.935), skirting=0.05, seam_x=-0.63, seam_pitch=0.53,
                    src="p3 (v0400 agrees): wall/floor line at y=-2.13; glass whiteboard from the wood rail up, +X edge "
                        "x=-0.11/-0.13; rail 0.83-0.935; dado seams x -0.63/-1.15/-1.68; x > -0.12 a full-height panel; "
                        "board top above every view (2.40 est.)"),
    cabinets=dict(x0=0.30, x_split=1.28, x1=2.00, front_y=-1.775, height=2.05, plinth=0.09, split_z=0.835,
                  shelves=(1.23, 1.64), handle_low=(0.65, 0.78), handle_up=(1.22, 1.36),
                  src="door face from the lower-door handle tops triangulated in v0340+v0400+p2 (y=-1.745, residual "
                      "<2 cm, handles 3 cm proud); glass unit x 0.28-0.33..1.18-1.33, glass from z 0.83 (v0400, p2); "
                      "tall wood unit centre seam x 1.55-1.62, -X end x 2.0-2.1 (v0340 floor line and door face); "
                      "shelf z 1.23 (v0400); top 2.05 est. (above every view)"),
    bench=dict(x0=-0.45, x1=0.28, depth=0.36, height=0.44, src="behind the real robot; top z 0.42-0.44 at x -0.47..0.22 "
                                                              "in p2; depth est."),
    pedestal=dict(x0=2.03, x1=2.53, y0=-1.87, y1=-1.49, height=0.60,
                  src="white drawer pedestal, drawer face toward -X: face x=2.03, y -1.87..-1.49 back-projected in p1 "
                      "and v0340; top z 0.59 (v0340); depth est."),
    sofa=dict(x0=-1.66, x1=0.62, y0=1.10, y1=1.95, seat_z=0.44, back_z=0.81, arm_z=0.62,
              src="foot + arm back-projection in v0160, leather point extents"),
    plant=dict(x=0.93, y=1.45, pot_r=0.19, pot_h=0.75, top=1.45, src="foliage point centroid + rim triangulation"),
    desk_east=dict(x0=0.59, x1=2.15, y0=-0.55, y1=0.33, top=0.76,
                   src="front corners triangulated (0.60,0.33,0.77)/(0.59,-0.52); back edge v0300/v0340 at z=0.75"),
    desk_west=dict(x0=-1.95, x1=-1.22, y0=-2.10, y1=-0.30, top=0.75, src="legs back-projected (-1.23..-1.34, -0.26..-0.44); near corner (-1.16,-0.32) at z=0.75"),
    desk_west2=dict(x0=-2.62, x1=-2.02, y0=-2.10, y1=-0.40, top=0.75, src="orange points near x=-2.4; coarse"),
    chairs=dict(west_a=(-1.45, -0.84, 90, "task"), west_b=(-1.50, 0.01, -135, "task"), door=(-2.15, 1.64, -150, "task"),
                corner=(1.25, 0.84, 124, "exec"), woman=(1.23, -0.94, 0, "task"),
                src="first build: star-base centers back-projected to the floor (v0040/v0160/v0480). Rework: corner hub "
                    "triangulated in v0300+p1 (residual 4 mm), facing the v0260/v0300 cameras; door, woman and west_b "
                    "hubs back-projected at z=0.1 in two or three views each (v0160/v0100, v0300/p1, v0100/v0480/v0040); "
                    "west_a kept (its back top triangulates to (-1.07,-0.81) in v0480+p0, as the first build). "
                    "task: mesh-back chair with T arms and a black base (v0040, v0100, v0160, v0480); exec: the padded "
                    "high-back chair with chrome arms and base by the plant (v0260, v0300, p4)"),
)

SUN_TO = np.array([-0.166, 0.732, 0.660])

# Uniform correction of the SfM metric scale (1.0 = 0.151 m per SfM unit). Set via --scale-correction
# once a real length is measured, e.g. measured_table_length / 1.20.
SCALE = 1.0

def lin(srgb):
    c = np.asarray(srgb, float) / 255
    return tuple(np.round(np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4), 4))


# Walls, cabinets and chairs: sRGB medians of the photos (named in the comment), used as albedo.
CABINET_WOOD_SRGB = (126, 84, 44)  # lower doors, p3 (sunlit) and v0400 (shade)

# name: (diffuse linear RGB, roughness, metallic, opacity)
MATERIALS = {
    "table_white": ((0.78, 0.78, 0.76), 0.45, 0.0, 1.0),
    "frame_white": ((0.80, 0.80, 0.79), 0.35, 0.0, 1.0),
    "cap_gray": ((0.35, 0.36, 0.37), 0.5, 0.0, 1.0),
    "blue_cup": ((0.04, 0.50, 0.72), 0.25, 0.0, 0.55),
    "paper_cup": ((0.055, 0.053, 0.050), 0.6, 0.0, 1.0),
    "paper_inner": ((0.72, 0.68, 0.60), 0.7, 0.0, 1.0),
    "tissue": ((0.66, 0.60, 0.80), 0.5, 0.0, 1.0),
    "tissue_paper": ((0.88, 0.88, 0.90), 0.9, 0.0, 0.85),
    "plush": ((0.42, 0.14, 0.045), 1.0, 0.0, 1.0),
    "steel": ((0.6, 0.6, 0.62), 0.25, 1.0, 1.0),
    "carpet": ((1.0, 1.0, 1.0), 0.95, 0.0, 1.0),
    "wall_beige": (lin((143, 126, 120)), 0.8, 0.0, 1.0),  # window-wall dado, v0160/v0260
    "wall_light": (lin((147, 138, 128)), 0.8, 0.0, 1.0),  # full-height panel beside the cabinets, p3/v0400
    "wall_panel": (lin((141, 138, 138)), 0.7, 0.0, 1.0),  # dado under the wood rail, p3/v0400
    "whiteboard": (lin((172, 182, 177)), 0.12, 0.0, 1.0),  # glass whiteboard above the rail, p3/v0400
    "skirting": (lin((51, 34, 26)), 0.5, 0.0, 1.0),  # p3
    "seam": ((0.02, 0.02, 0.02), 0.9, 0.0, 1.0),
    "sill_white": (lin((180, 180, 184)), 0.4, 0.0, 1.0),  # v0160
    "spandrel": (lin((62, 61, 62)), 0.5, 0.0, 1.0),  # v0160/v0260
    "column_clad": (lin((209, 200, 185)), 0.6, 0.0, 1.0),  # NE corner column, v0260
    "mullion": ((0.06, 0.06, 0.065), 0.4, 0.3, 1.0),
    "exterior": ((1.0, 1.0, 1.0), 1.0, 0.0, 1.0),
    "leather": ((0.36, 0.10, 0.03), 0.45, 0.0, 1.0),
    "pot_white": ((0.80, 0.80, 0.78), 0.3, 0.0, 1.0),
    "leaf": ((0.04, 0.16, 0.03), 0.6, 0.0, 1.0),
    "chair_black": (lin((25, 22, 23)), 0.45, 0.0, 1.0),  # padded high-back chair, v0260
    "chair_seat": (lin((39, 42, 53)), 0.9, 0.0, 1.0),  # task-chair seat fabric, v0480
    "chair_mesh": (lin((40, 41, 45)), 0.8, 0.0, 0.6),  # task-chair back mesh, lets the room through (v0100)
    "chair_nylon": ((0.02, 0.02, 0.022), 0.4, 0.0, 1.0),
    "chrome": ((0.55, 0.55, 0.57), 0.2, 1.0, 1.0),
    "desk_orange": ((0.62, 0.25, 0.05), 0.45, 0.0, 1.0),
    "cabinet_wood": (lin(CABINET_WOOD_SRGB), 0.5, 0.0, 1.0),  # diffuse from textures/cabinet_wood.png
    "plinth": (lin((73, 49, 30)), 0.6, 0.0, 1.0),  # p3
    "handle_steel": ((0.55, 0.55, 0.56), 0.3, 1.0, 1.0),
    "pedestal_white": (lin((190, 190, 186)), 0.5, 0.0, 1.0),
    "glass": ((0.55, 0.60, 0.60), 0.05, 0.0, 0.25),
    "frosted": ((0.75, 0.77, 0.77), 0.6, 0.0, 0.7),
    "tv_black": ((0.01, 0.01, 0.012), 0.15, 0.0, 1.0),
    "wood_strip": (lin((84, 56, 38)), 0.45, 0.0, 1.0),  # rail under the TV, p3
    "basket": ((0.80, 0.78, 0.70), 0.7, 0.0, 1.0),
}

TEXTURED = {"carpet": "carpet_tile.png", "exterior": "window_exterior.png", "cabinet_wood": "cabinet_wood.png"}
WOOD_TILE_M = 0.6  # cabinet_wood.png covers 0.6 x 0.6 m, grain along v

PHYS = {"default": (0.5, 0.45, 0.0), "table_white": (0.5, 0.45, 0.0), "carpet": (0.8, 0.7, 0.0)}


# ----------------------------------------------------------------------------- geometry helpers
def T(x=0.0, y=0.0, z=0.0, yaw=0.0):
    m = trimesh.transformations.rotation_matrix(math.radians(yaw), [0, 0, 1])
    m[:3, 3] = [x, y, z]
    return m


def box(cx, cy, z0, sx, sy, sz, yaw=0.0):
    return trimesh.creation.box(extents=[sx, sy, sz], transform=T(cx, cy, z0 + sz / 2, yaw))


def box_xyz(x0, x1, y0, y1, z0, z1):
    return box((x0 + x1) / 2, (y0 + y1) / 2, z0, x1 - x0, y1 - y0, z1 - z0)


def cyl(cx, cy, z0, r, h, sections=32):
    return trimesh.creation.cylinder(radius=r, height=h, sections=sections, transform=T(cx, cy, z0 + h / 2))


def ellipsoid(c, radii, subdiv=3):
    s = trimesh.creation.icosphere(subdivisions=subdiv)
    s.apply_scale(radii)
    s.apply_translation(c)
    return s


def quad(p0, p1, p2, p3):
    return trimesh.Trimesh(vertices=[p0, p1, p2, p3], faces=[[0, 1, 2], [0, 2, 3]], process=False)


def rounded_slab(length, width, r, z0, z1):
    poly = sbox(-length / 2 + r, -width / 2 + r, length / 2 - r, width / 2 - r).buffer(r, quad_segs=8)
    m = trimesh.creation.extrude_polygon(poly, z1 - z0)
    m.apply_translation([0, 0, z0])
    return m


def planar_uv(tile_m):
    """UV callback for Layer.mesh: each triangle projected on its dominant plane, `tile_m` metres per unit;
    vertical faces get v = z (wood grain runs up the doors)."""
    def uv(fv):
        tri = fv.reshape(-1, 3, 3)
        ax = np.repeat(np.abs(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])).argmax(1), 3)[:, None]
        return np.where(ax == 0, fv[:, [1, 2]], np.where(ax == 1, fv[:, [0, 2]], fv[:, [0, 1]])) / tile_m
    return uv


def sheet(f, nu, nv, thick):
    """Closed thin shell over the surface f(u, v) -> xyz, u, v in [0, 1], thickened by `thick` toward -Y."""
    U, V = np.meshgrid(np.linspace(0, 1, nu), np.linspace(0, 1, nv), indexing="ij")
    P = np.array([f(u, v) for u, v in zip(U.ravel(), V.ravel())])
    n = len(P)
    faces = []
    for i in range(nu - 1):
        for j in range(nv - 1):
            a, b, c, d = i * nv + j, (i + 1) * nv + j, (i + 1) * nv + j + 1, i * nv + j + 1
            faces += [[a, b, c], [a, c, d], [a + n, c + n, b + n], [a + n, d + n, c + n]]
    ring = ([i * nv for i in range(nu)] + [(nu - 1) * nv + j for j in range(1, nv)]
            + [i * nv + nv - 1 for i in range(nu - 2, -1, -1)] + [j for j in range(nv - 2, 0, -1)])
    for a, b in zip(ring, ring[1:] + ring[:1]):
        faces += [[a, b, b + n], [a, b + n, a + n]]
    m = trimesh.Trimesh(np.vstack([P, P + [0.0, -thick, 0.0]]), faces, process=False)
    m.fix_normals()
    return m, P[ring]


def tube(points, r, closed=False, sections=8):
    pts = list(points) + ([points[0]] if closed else [])
    return trimesh.util.concatenate([trimesh.creation.cylinder(radius=r, segment=[a, b], sections=sections)
                                     for a, b in zip(pts[:-1], pts[1:])])


def cup_profile(H, rt, rb, wall, bottom):
    return np.array([[0, 0], [rb, 0], [rt, H], [rt - wall, H], [rb - wall, bottom], [0, bottom]])


def revolve(profile, sections=48):
    return trimesh.creation.revolve(profile, sections=sections)


def cup_wall_colliders(H, rt, rb, wall, n=12):
    parts = []
    for k in range(n):
        a0, a1 = 2 * math.pi * k / n, 2 * math.pi * (k + 1) / n
        pts = []
        for a in (a0, a1):
            for (r, z) in ((rb, 0.0), (rt, H)):
                for rr in (r, r - max(wall, 0.004)):
                    pts.append([rr * math.cos(a), rr * math.sin(a), z])
        parts.append(trimesh.convex.convex_hull(np.array(pts)))
    parts.append(trimesh.creation.cylinder(radius=rb, height=0.004, sections=24, transform=T(0, 0, 0.002)))
    return parts


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
        self.mats = {}
        self.pmats = {}
        self.tex_rel = "./textures" if path.parent == OUT else "../textures"

    def material(self, key):
        if key in self.mats:
            return self.mats[key]
        color, rough, metal, opacity = MATERIALS[key]
        mat = UsdShade.Material.Define(self.stage, f"{self.rootpath}/Looks/{key}")
        sh = UsdShade.Shader.Define(self.stage, f"{self.rootpath}/Looks/{key}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metal)
        if opacity < 1.0:
            sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(opacity)
            sh.CreateInput("ior", Sdf.ValueTypeNames.Float).Set(1.49)
        tex_file = TEXTURED.get(key)
        if tex_file:
            st = UsdShade.Shader.Define(self.stage, f"{self.rootpath}/Looks/{key}/stReader")
            st.CreateIdAttr("UsdPrimvarReader_float2")
            st.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
            st.CreateOutput("result", Sdf.ValueTypeNames.Float2)
            tx = UsdShade.Shader.Define(self.stage, f"{self.rootpath}/Looks/{key}/Texture")
            tx.CreateIdAttr("UsdUVTexture")
            tx.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(f"{self.tex_rel}/{tex_file}")
            tx.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
            tx.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
            tx.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
            tx.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(st.ConnectableAPI(), "result")
            tx.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
            if key == "exterior":
                sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tx.ConnectableAPI(), "rgb")
        else:
            sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[key] = mat
        return mat

    def phys_material(self, key, fr=None):
        tag = key if fr is None else f"{key}_{fr[0]:.2f}_{fr[1]:.2f}".replace(".", "p")
        if tag in self.pmats:
            return self.pmats[tag]
        sf, df, rs = PHYS.get(key, PHYS["default"]) if fr is None else (fr[0], fr[1], 0.0)
        pm = UsdShade.Material.Define(self.stage, f"{self.rootpath}/PhysicsMaterials/{tag}")
        api = UsdPhysics.MaterialAPI.Apply(pm.GetPrim())
        api.CreateStaticFrictionAttr(sf)
        api.CreateDynamicFrictionAttr(df)
        api.CreateRestitutionAttr(rs)
        self.pmats[tag] = pm
        return pm

    def mesh(self, path, m, mat=None, smooth=False, uv=None, guide=False, no_shadow=False):
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
            fn = np.repeat(m.face_normals, 3, axis=0).astype(np.float32)
            um.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(fn))
            um.SetNormalsInterpolation(UsdGeom.Tokens.faceVarying)
        if uv is not None:
            fv = m.vertices[m.faces.reshape(-1)]
            st = uv(fv).astype(np.float32)
            pv = UsdGeom.PrimvarsAPI(um).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray,
                                                       UsdGeom.Tokens.faceVarying)
            pv.Set(Vt.Vec2fArray.FromNumpy(st))
        if no_shadow:
            UsdGeom.PrimvarsAPI(um).CreatePrimvar("doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)
        if guide:
            um.CreatePurposeAttr(UsdGeom.Tokens.guide)
        elif mat:
            UsdShade.MaterialBindingAPI.Apply(um.GetPrim()).Bind(self.material(mat))
        return um

    def collider(self, path, m, approx="convexHull", pkey="default", fr=None):
        um = self.mesh(path, m, guide=True)
        prim = um.GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr(approx)
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(self.phys_material(pkey, fr),
                                                     UsdShade.Tokens.weakerThanDescendants, "physics")
        return um

    def save(self):
        self.stage.GetRootLayer().Save()


# ----------------------------------------------------------------------------- textures
def wood_texture(mean_srgb, size=512, seed=7):
    """Tileable straight-grain veneer, sRGB mean = mean_srgb; grain along image rows' normal (v)."""
    rng = np.random.default_rng(seed)
    u = np.arange(size) / size
    grain = sum(rng.normal() / f * np.sin(2 * np.pi * (f * u + rng.uniform())) for f in range(6, 120))
    drift = sum(rng.normal() / f * np.sin(2 * np.pi * (f * u + rng.uniform())) for f in (1, 2))
    shift = (np.arange(size)[None, :] + 3 * drift[:, None]).astype(int) % size
    g = grain[shift]
    g = (g - g.mean()) / g.std()
    fine = cv2.GaussianBlur(np.pad(rng.normal(size=(size, size)), 32, mode="wrap"), (0, 0), sigmaX=0.7, sigmaY=8)
    fine = fine[32:-32, 32:-32]
    shade = 1 + 0.035 * g + 0.25 * fine
    rgb = np.asarray(mean_srgb, float)[None, None, :] * shade[..., None]
    rgb *= np.asarray(mean_srgb, float) / rgb.reshape(-1, 3).mean(0)
    return np.clip(rgb[..., ::-1], 0, 255).astype(np.uint8)


def build_textures():
    """Copy the packaged texture sources; regenerate them from the observation analysis only if missing."""
    TEX.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(TEX / "cabinet_wood.png"), wood_texture(CABINET_WOOD_SRGB))
    src = HERE / "texture_src"
    if (src / "carpet_tile.png").exists() and (src / "window_exterior.png").exists():
        for name in ("carpet_tile.png", "window_exterior.png"):
            shutil.copy(src / name, TEX / name)
        return
    sys.path.insert(0, str(OBS / "work"))
    from carpet import rectify
    tile = rectify("v0040", -1.15, 0.70, 0.5, px=560).astype(np.float32)
    g = cv2.GaussianBlur(tile, (0, 0), 60)
    tile = tile / np.maximum(g, 1) * g.reshape(-1, 3).mean(0)
    blend = 48
    w = np.linspace(0, 1, blend)[None, :, None]
    out = tile[:, :-blend].copy()
    out[:, :blend] = tile[:, :blend] * w + tile[:, -blend:] * (1 - w)
    out = cv2.resize(out, (512, 512))
    cv2.imwrite(str(TEX / "carpet_tile.png"), np.clip(out, 0, 255).astype(np.uint8))
    ext = cv2.imread(str(HERE / "reference" / "v0160.jpg"))[0:118, 140:1280]
    cv2.imwrite(str(TEX / "window_exterior.png"), cv2.resize(ext, (1536, 256)))


# ----------------------------------------------------------------------------- scene parts
def window_wall(L, name, axis, fixed, a0, a1, sill_z, band_top, H, pitch, inward, seams=()):
    """Window wall along `axis` ('x' or 'y') at coordinate `fixed`; inward = +1/-1 normal toward room."""
    def bx(u0, u1, d0, d1, z0, z1):
        if axis == "x":
            return box_xyz(u0, u1, fixed + inward * d1, fixed + inward * d0, z0, z1) if inward < 0 else \
                box_xyz(u0, u1, fixed + inward * d0, fixed + inward * d1, z0, z1)
        return box_xyz(fixed + inward * d1, fixed + inward * d0, u0, u1, z0, z1) if inward < 0 else \
            box_xyz(fixed + inward * d0, fixed + inward * d1, u0, u1, z0, z1)

    L.mesh(f"{name}/lower", bx(a0, a1, -0.12, 0.0, 0.0, sill_z - 0.04), "wall_beige", uv=planar_uv(1.0))
    for k, u in enumerate(seams):
        L.mesh(f"{name}/seam_{k}", bx(u - 0.002, u + 0.002, 0.0, 0.002, 0.0, sill_z - 0.04), "seam")
    L.mesh(f"{name}/sill", bx(a0, a1, -0.12, 0.10, sill_z - 0.04, sill_z), "sill_white")
    L.mesh(f"{name}/spandrel", bx(a0, a1, -0.10, -0.02, sill_z, band_top), "spandrel")
    for k, u in enumerate(np.arange(a0, a1 + 1e-6, pitch)):
        L.mesh(f"{name}/mullion_{k}", bx(max(a0, u - 0.03), min(a1, u + 0.03), -0.10, -0.02, band_top, H), "mullion")
    L.mesh(f"{name}/mullion_end", bx(a1 - 0.06, a1, -0.10, -0.02, band_top, H), "mullion")
    L.mesh(f"{name}/head", bx(a0, a1, -0.10, -0.02, H - 0.10, H), "mullion")


def add_room(L):
    r = LAYOUT["room"]
    X0, X1, Y0, Y1, H = r["x0"], r["x1"], r["y0"], r["y1"], r["height"]
    L.mesh("Floor", quad([X0, Y0, 0], [X1, Y0, 0], [X1, Y1, 0], [X0, Y1, 0]), "carpet",
           uv=lambda fv: fv[:, :2] / 0.5)
    L.collider("Colliders/floor", box_xyz(X0 - 1, X1 + 1, Y0 - 1, Y1 + 1, -0.10, 0.0), pkey="carpet")
    nw, ne, ew = LAYOUT["north_window"], LAYOUT["ne_block"], LAYOUT["east_window"]
    window_wall(L, "WallN", "x", Y1, X0, nw["x1"], nw["sill_z"], nw["band_top"], H, nw["mullion_pitch"], -1,
                nw["seams"])
    L.mesh("Exterior/north", quad([-6.0, Y1 + 2.0, 0], [-6.0, Y1 + 2.0, 4.5], [5.0, Y1 + 2.0, 4.5], [5.0, Y1 + 2.0, 0]),
           "exterior", uv=lambda fv: np.c_[(fv[:, 0] + 6.0) / 11.0, fv[:, 2] / 4.5], no_shadow=True)
    L.mesh("NECornerBlock", box_xyz(ne["x0"], X1 + 0.12, ne["y0"], Y1 + 0.12, 0.0, H), "column_clad")
    window_wall(L, "WallE", "y", X1, ew["y0"], ew["y1"], ew["sill_z"], ew["band_top"], H, 1.30, -1)
    L.mesh("WallE/solid_south", box_xyz(X1, X1 + 0.12, Y0, ew["y0"], 0.0, H), "wall_light", uv=planar_uv(1.0))
    L.mesh("Exterior/east", quad([X1 + 2.0, -6.0, 0], [X1 + 2.0, 5.0, 0], [X1 + 2.0, 5.0, 4.5], [X1 + 2.0, -6.0, 4.5]),
           "exterior", uv=lambda fv: np.c_[(fv[:, 1] + 6.0) / 11.0, fv[:, 2] / 4.5], no_shadow=True)
    tv, cb, sw = LAYOUT["tv"], LAYOUT["cabinets"], LAYOUT["south_wall"]
    bx1, (r0, r1), sk = sw["board_x1"], sw["rail"], sw["skirting"]
    L.mesh("WallS/wall", box_xyz(X0, X1, Y0 - 0.10, Y0, 0.0, H), "wall_light", uv=planar_uv(1.0))
    L.mesh("WallS/whiteboard", box_xyz(X0, bx1, Y0, Y0 + 0.008, r1, sw["board_top"]), "whiteboard")
    L.mesh("WallS/wood_strip", box_xyz(X0, bx1, Y0, Y0 + 0.025, r0, r1), "wood_strip")
    L.mesh("WallS/dado", box_xyz(X0, bx1, Y0, Y0 + 0.008, sk, r0), "wall_panel")
    for k, x in enumerate(np.arange(sw["seam_x"], X0, -sw["seam_pitch"])):
        L.mesh(f"WallS/dado_seam_{k}", box_xyz(x - 0.002, x + 0.002, Y0 + 0.008, Y0 + 0.010, sk, r0), "seam")
    L.mesh("WallS/skirting", box_xyz(X0, cb["x0"], Y0, Y0 + 0.012, 0.0, sk), "skirting")
    L.mesh("WallS/tv", box_xyz(tv["x0"], tv["x1"], Y0 + 0.01, Y0 + 0.06, tv["z0"], tv["z1"]), "tv_black")
    add_cabinets(L, Y0)
    d = LAYOUT["door"]
    L.mesh("WallW/glass_south", box_xyz(X0 - 0.02, X0, Y0, d["y0"], 0.0, H), "glass")
    L.mesh("WallW/glass_north", box_xyz(X0 - 0.02, X0, d["y1"], Y1, 0.0, H), "glass")
    L.mesh("WallW/transom", box_xyz(X0 - 0.02, X0, d["y0"], d["y1"], d["height"], H), "glass")
    L.mesh("WallW/door", box_xyz(X0 - 0.04, X0 + 0.01, d["y0"] + 0.04, d["y1"] - 0.04, 0.0, d["height"]), "frosted")
    for k, y in enumerate([Y0 + 0.02, d["y0"], d["y1"], Y1 - 0.02]):
        L.mesh(f"WallW/post_{k}", box_xyz(X0 - 0.05, X0 + 0.01, y - 0.025, y + 0.025, 0.0, H), "frame_white")
    L.mesh("WallW/base", box_xyz(X0 - 0.05, X0 + 0.01, Y0, Y1, 0.0, 0.08), "frame_white")
    bn, pd = LAYOUT["bench"], LAYOUT["pedestal"]
    for nm, b in {"wallN": (X0, X1, Y1, Y1 + 0.12, 0.0, H), "wallS": (X0, X1, Y0 - 0.12, Y0, 0.0, H),
                  "wallE": (X1, X1 + 0.12, Y0, Y1, 0.0, H), "wallW": (X0 - 0.05, X0, Y0, Y1, 0.0, H),
                  "ne_block": (ne["x0"], X1, ne["y0"], Y1, 0.0, H),
                  "cabinets": (cb["x0"], cb["x1"], Y0, cb["front_y"], 0.0, cb["height"]),
                  "bench": (bn["x0"], bn["x1"], Y0, Y0 + bn["depth"], 0.0, bn["height"]),
                  "pedestal": (pd["x0"], pd["x1"], pd["y0"], pd["y1"], 0.0, pd["height"])}.items():
        L.collider(f"Colliders/{nm}", box_xyz(*b))


def handle(x, y_face, z0, z1):
    """Bar pull standing 30 mm proud of a door face at y_face (doors face +Y)."""
    parts = [box_xyz(x - 0.006, x + 0.006, y_face + 0.018, y_face + 0.030, z0, z1)]
    for z in (z0 + 0.012, z1 - 0.027):
        parts.append(box_xyz(x - 0.005, x + 0.005, y_face, y_face + 0.018, z, z + 0.015))
    return trimesh.util.concatenate(parts)


def add_cabinets(L, Y0):
    """South-wall storage: a display unit (glass upper doors, x0..x_split) and a tall wood unit (x_split..x1), both
    with wood lower doors and a recessed plinth; the low open bench behind the real robot; the white drawer pedestal
    past the tall unit's +X end."""
    c = LAYOUT["cabinets"]
    yf, yc = c["front_y"], c["front_y"] - 0.02  # door face, carcass front (20 mm doors)
    zp, zm, zt, t, gap = c["plinth"], c["split_z"], c["height"], 0.018, 0.003
    xg0, xg1, xt1 = c["x0"], c["x_split"], c["x1"]
    wood = dict(uv=planar_uv(WOOD_TILE_M))
    L.mesh("Cabinets/plinth", box_xyz(xg0 + 0.02, xt1 - 0.02, Y0, yf - 0.045, 0.0, zp), "plinth")
    L.mesh("Cabinets/tall/carcass", box_xyz(xg1, xt1, Y0, yc, zp, zt), "cabinet_wood", **wood)
    L.mesh("Cabinets/display/carcass_lower", box_xyz(xg0, xg1, Y0, yc, zp, zm), "cabinet_wood", **wood)
    shell = [box_xyz(xg0, xg1, Y0, Y0 + t, zm, zt), box_xyz(xg0, xg0 + t, Y0, yc, zm, zt),
             box_xyz(xg1 - t, xg1, Y0, yc, zm, zt), box_xyz(xg0, xg1, Y0, yc, zt - t, zt)]
    shell += [box_xyz(xg0 + t, xg1 - t, Y0 + t, yc - 0.03, z - t / 2, z + t / 2) for z in c["shelves"]]
    L.mesh("Cabinets/display/shell", trimesh.util.concatenate(shell), "cabinet_wood", **wood)

    def door(path, x0, x1, z0, z1):
        L.mesh(path, box_xyz(x0 + gap, x1 - gap, yc, yf, z0 + gap, z1 - gap), "cabinet_wood", **wood)

    handles, fw = [], 0.035
    for unit, (a, b) in {"display": (xg0, xg1), "tall": (xg1, xt1)}.items():
        m = (a + b) / 2
        for side, (u0, u1) in (("p", (a, m)), ("n", (m, b))):
            door(f"Cabinets/{unit}/door_low_{side}", u0, u1, zp, zm)
            if unit == "tall":
                door(f"Cabinets/{unit}/door_up_{side}", u0, u1, zm, zt)
            else:
                g0, g1, z0, z1 = u0 + gap, u1 - gap, zm + gap, zt - gap
                frame = [box_xyz(g0, g0 + fw, yc, yf, z0, z1), box_xyz(g1 - fw, g1, yc, yf, z0, z1),
                         box_xyz(g0, g1, yc, yf, z0, z0 + fw), box_xyz(g0, g1, yc, yf, z1 - fw, z1)]
                L.mesh(f"Cabinets/{unit}/door_frame_{side}", trimesh.util.concatenate(frame), "frame_white")
                L.mesh(f"Cabinets/{unit}/door_glass_{side}",
                       box_xyz(g0 + fw, g1 - fw, yc + 0.008, yc + 0.012, z0 + fw, z1 - fw), "glass")
        for s in (-1, 1):
            handles += [handle(m + s * 0.03, yf, *c["handle_low"]), handle(m + s * 0.03, yf, *c["handle_up"])]
    L.mesh("Cabinets/handles", trimesh.util.concatenate(handles), "handle_steel")

    bn = LAYOUT["bench"]
    y1, h = Y0 + bn["depth"], bn["height"]
    parts = [box_xyz(bn["x0"], bn["x1"], Y0, y1, h - 0.025, h), box_xyz(bn["x0"], bn["x1"], Y0, y1, 0.03, 0.05),
             box_xyz(bn["x0"], bn["x1"], Y0, Y0 + t, 0.05, h - 0.025)]
    parts += [box_xyz(x, x + t, Y0, y1, 0.0, h - 0.025) for x in (bn["x0"], bn["x1"] - t)]
    L.mesh("Bench/body", trimesh.util.concatenate(parts), "cabinet_wood", **wood)

    pd = LAYOUT["pedestal"]
    x0, x1, py0, py1, h = pd["x0"], pd["x1"], pd["y0"], pd["y1"], pd["height"]
    L.mesh("Pedestal/body", box_xyz(x0 + 0.02, x1, py0, py1, 0.03, h - 0.02), "pedestal_white")
    L.mesh("Pedestal/top", box_xyz(x0, x1, py0, py1, h - 0.02, h), "desk_orange")
    fronts, knobs = [], []
    for z0, z1 in ((0.03, 0.30), (0.30, 0.45), (0.45, h - 0.02)):
        fronts.append(box_xyz(x0, x0 + 0.02, py0 + gap, py1 - gap, z0 + gap, z1 - gap))
        knobs.append(box_xyz(x0 - 0.02, x0, (py0 + py1) / 2 - 0.01, (py0 + py1) / 2 + 0.01, z1 - 0.06, z1 - 0.04))
    L.mesh("Pedestal/drawers", trimesh.util.concatenate(fronts), "pedestal_white")
    L.mesh("Pedestal/knobs", trimesh.util.concatenate(knobs), "chair_nylon")
    L.mesh("Pedestal/casters", trimesh.util.concatenate(
        [cyl(x, y, 0.0, 0.02, 0.03, 12) for x in (x0 + 0.05, x1 - 0.05) for y in (py0 + 0.05, py1 - 0.05)]),
        "chair_nylon")


def add_desk(L, name, x0, x1, y0, y1, top=0.75):
    L.mesh(f"{name}/top", box_xyz(x0, x1, y0, y1, top - 0.025, top), "desk_orange")
    for k, (lx, ly) in enumerate([(x0 + 0.05, y0 + 0.05), (x1 - 0.05, y0 + 0.05), (x0 + 0.05, y1 - 0.05),
                                  (x1 - 0.05, y1 - 0.05)]):
        L.mesh(f"{name}/leg_{k}", box(lx, ly, 0.0, 0.05, 0.05, top - 0.025), "frame_white")
    L.collider(f"Colliders/{name}_top", box_xyz(x0, x1, y0, y1, top - 0.025, top))


def star_base(reach=0.31):
    """Five-spoke base with twin-wheel casters: (spokes and hub, casters)."""
    spokes, casters = [cyl(0, 0, 0.085, 0.045, 0.05, 20)], []
    for k in range(5):
        a = math.pi / 2 + 2 * math.pi * k / 5
        d, t = np.array([math.cos(a), math.sin(a), 0.0]), np.array([-math.sin(a), math.cos(a), 0.0])
        pts = [d * r + t * s * w + [0, 0, z] for r, w, zs in ((0.03, 0.025, (0.09, 0.13)), (reach, 0.017, (0.06, 0.085)))
               for s in (-1, 1) for z in zs]
        spokes.append(trimesh.convex.convex_hull(np.array(pts)))
        c = d * reach + [0, 0, 0.026]
        casters.append(trimesh.creation.cylinder(radius=0.026, segment=[c - t * 0.022, c + t * 0.022], sections=16))
        casters.append(box(*(d * reach)[:2], 0.045, 0.02, 0.02, 0.02))
    return spokes, casters


def chair_parts(kind):
    """Office chair meshes by material, base centre on the floor, local +Y = the way the sitter faces."""
    spokes, casters = star_base()
    if kind == "task":
        p = {"chair_nylon": spokes + casters + [cyl(0, 0, 0.13, 0.030, 0.13, 16), box(0, -0.01, 0.38, 0.22, 0.22, 0.05)],
             "chrome": [cyl(0, 0, 0.26, 0.022, 0.13, 16)], "chair_seat": [], "chair_mesh": []}
        seat = rounded_slab(0.50, 0.48, 0.08, 0.43, 0.50)
        seat.apply_translation([0, 0.02, 0])
        p["chair_seat"].append(seat)
        z0, z1 = 0.56, 1.02  # back top 0.97-1.02 triangulated (west_a, west_b, door)
        back, rim = sheet(lambda u, v: ((u - 0.5) * (0.47 - 0.05 * v), -0.25 - 0.16 * v + 0.55 * ((u - 0.5) * 0.47) ** 2,
                                        z0 + v * (z1 - z0)), 9, 7, 0.006)
        p["chair_mesh"].append(back)
        p["chair_nylon"] += [tube(rim, 0.013, closed=True), box(0, -0.25, 0.42, 0.07, 0.04, 0.15),
                             box(0, -0.30, 0.64, 0.30, 0.03, 0.08)]
        for s in (-1, 1):
            p["chair_nylon"] += [box(s * 0.18, -0.02, 0.40, 0.18, 0.05, 0.025), box(s * 0.27, -0.02, 0.40, 0.035, 0.06, 0.25)]
            p["chair_seat"].append(box(s * 0.27, 0.0, 0.65, 0.08, 0.25, 0.03))
        return p
    p = {"chrome": spokes + [cyl(0, 0, 0.13, 0.028, 0.26, 16)], "chair_nylon": casters + [box(0, 0, 0.38, 0.24, 0.24, 0.05)],
         "chair_black": []}
    seat = rounded_slab(0.52, 0.50, 0.10, 0.43, 0.53)
    seat.apply_translation([0, 0.02, 0])
    z0, z1 = 0.55, 1.19  # back top 1.18 triangulated in v0260+v0300+p4
    back, _ = sheet(lambda u, v: ((u - 0.5) * (0.52 - 0.06 * v), -0.22 - 0.20 * v + 0.45 * ((u - 0.5) * 0.52) ** 2,
                                  z0 + v * (z1 - z0)), 9, 7, 0.08)
    p["chair_black"] += [seat, back, box(0, -0.26, 0.42, 0.10, 0.06, 0.16)]
    for s in (-1, 1):
        x = s * 0.29
        p["chrome"].append(tube([(x, -0.17, 0.45), (x, -0.13, 0.66), (x, 0.16, 0.66), (x, 0.20, 0.45)], 0.011))
        p["chair_black"].append(box(x, 0.01, 0.665, 0.06, 0.24, 0.025))
    return p


def add_chair(L, name, x, y, yaw, kind):
    for mat, parts in chair_parts(kind).items():
        m = trimesh.util.concatenate(parts)
        m.apply_transform(T(x, y, 0, yaw))
        L.mesh(f"{name}/{mat}", m, mat)
    c = box(0, -0.02, 0.0, 0.62, 0.62, 1.04 if kind == "task" else 1.20)
    c.apply_transform(T(x, y, 0, yaw))
    L.collider(f"Colliders/{name}", c)


def add_furniture(L):
    s = LAYOUT["sofa"]
    sx0, sx1, sy0, sy1 = s["x0"], s["x1"], s["y0"], s["y1"]
    L.mesh("Sofa/base", box_xyz(sx0, sx1, sy0 + 0.02, sy1, 0.08, 0.30), "leather")
    L.mesh("Sofa/seat", box_xyz(sx0 + 0.17, sx1 - 0.17, sy0, sy1 - 0.20, 0.30, s["seat_z"]), "leather")
    L.mesh("Sofa/back", box_xyz(sx0 + 0.17, sx1 - 0.17, sy1 - 0.22, sy1, 0.30, s["back_z"]), "leather")
    L.mesh("Sofa/arm_l", box_xyz(sx0, sx0 + 0.17, sy0 + 0.02, sy1, 0.30, s["arm_z"]), "leather")
    L.mesh("Sofa/arm_r", box_xyz(sx1 - 0.17, sx1, sy0 + 0.02, sy1, 0.30, s["arm_z"]), "leather")
    L.mesh("Sofa/pillow", box(sx1 - 0.40, sy0 + 0.38, s["seat_z"], 0.40, 0.26, 0.12, -20), "leather")
    for k, (lx, ly) in enumerate([(sx0 + 0.06, sy0 + 0.08), (sx1 - 0.06, sy0 + 0.08), (sx0 + 0.06, sy1 - 0.08),
                                  (sx1 - 0.06, sy1 - 0.08)]):
        L.mesh(f"Sofa/foot_{k}", cyl(lx, ly, 0.0, 0.02, 0.08), "chrome")
    L.collider("Colliders/sofa", box_xyz(sx0, sx1, sy0, sy1, 0.0, s["back_z"]))
    p = LAYOUT["plant"]
    px, py = p["x"], p["y"]
    L.mesh("Plant/pot", cyl(px, py, 0.0, p["pot_r"], p["pot_h"], 40), "pot_white")
    rng = np.random.default_rng(3)
    leaves = [cyl(px, py, p["pot_h"] - 0.05, 0.05, p["top"] - p["pot_h"] - 0.1, 12)]
    for k in range(80):
        h = rng.uniform(p["pot_h"] + 0.05, p["top"])
        a = rng.uniform(0, 2 * math.pi)
        r = rng.uniform(0.02, 0.30) * (h - p["pot_h"]) / (p["top"] - p["pot_h"]) + 0.03
        lf = ellipsoid([0, 0, 0], [0.055, 0.022, 0.006], 1)
        lf.apply_transform(trimesh.transformations.euler_matrix(rng.uniform(-0.6, 0.6), rng.uniform(-0.5, 0.5), a))
        lf.apply_translation([px + r * math.cos(a), py + r * math.sin(a), h])
        leaves.append(lf)
    L.mesh("Plant/foliage", trimesh.util.concatenate(leaves), "leaf")
    L.collider("Colliders/plant", cyl(px, py, 0.0, p["pot_r"], p["pot_h"], 16))
    de = LAYOUT["desk_east"]
    add_desk(L, "DeskEast", de["x0"], de["x1"], de["y0"], de["y1"], de["top"])
    L.mesh("DeskEast/basket", box(0.86, 0.12, de["top"], 0.40, 0.28, 0.12, 8), "basket")
    for nm in ("desk_west", "desk_west2"):
        d = LAYOUT[nm]
        add_desk(L, "DeskWest" if nm == "desk_west" else "DeskWest2", d["x0"], d["x1"], d["y0"], d["y1"], d["top"])
    for nm, pose in LAYOUT["chairs"].items():
        if nm != "src":
            add_chair(L, f"Chair_{nm}", *pose)


def add_table(L):
    t = TABLE
    z1 = t["top_z"]
    z0 = z1 - t["thickness"]
    L.mesh("Top", rounded_slab(t["length"], t["width"], t["corner_r"], z0, z1), "table_white")
    L.mesh("Frame/beam", box(0, 0, z0 - 0.04, 2 * t["column_x"], 0.06, 0.04), "frame_white")
    for sx in (-1, 1):
        x = sx * t["column_x"]
        n = "p" if sx > 0 else "n"
        L.mesh(f"Frame/crossbar_{n}", box(x, 0, z0 - 0.03, 0.06, t["width"] - 0.10, 0.03), "frame_white")
        L.mesh(f"Frame/column_upper_{n}", box(x, 0, 0.33, 0.065, 0.05, z0 - 0.03 - 0.33), "frame_white")
        L.mesh(f"Frame/column_lower_{n}", box(x, 0, 0.035, 0.08, 0.06, 0.33), "frame_white")
        L.mesh(f"Frame/foot_{n}", box(x, 0, 0.0, 0.07, t["foot_len"] - 0.06, 0.035), "frame_white")
        for sy in (-1, 1):
            L.mesh(f"Frame/cap_{n}_{'p' if sy > 0 else 'n'}",
                   box(x, sy * (t["foot_len"] / 2 - 0.015), 0.0, 0.072, 0.03, 0.037), "cap_gray")
        L.collider(f"Colliders/column_{n}", box(x, 0, 0.0, 0.08, 0.06, z0 - 0.03))
        L.collider(f"Colliders/foot_{n}", box(x, 0, 0.0, 0.07, t["foot_len"], 0.037))
    L.collider("Colliders/top", box(0, 0, z0, t["length"], t["width"], t["thickness"]), pkey="table_white")


def add_object(name, spec):
    L = Layer(OUT / "objects" / f"{name}.usda", name)
    prim = L.root.GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(spec["mass"])
    fr = spec["friction"]
    if spec["kind"] == "cup":
        H, rt, rb, w = spec["height"], spec["r_top"], spec["r_bot"], spec["wall"]
        L.mesh("Visual/cup", revolve(cup_profile(H, rt, rb, w, 0.002)), spec["mat"], smooth=True)
        if name == "paper_cup":
            inner = revolve(np.array([[0, 0.0021], [rb - w - 0.0002, 0.0021], [rt - w - 0.0002, H - 0.001],
                                      [rt - w - 0.0003, H - 0.001], [rb - w - 0.0003, 0.0022], [0, 0.0022]]))
            L.mesh("Visual/inner", inner, "paper_inner", smooth=True)
            rim = trimesh.creation.torus(major_radius=rt - 0.0005, minor_radius=0.0018, major_sections=48,
                                         minor_sections=8)
            rim.apply_translation([0, 0, H - 0.0015])
            L.mesh("Visual/rim", rim, "paper_inner", smooth=True)
        for i, part in enumerate(cup_wall_colliders(H, rt, rb, w)):
            L.collider(f"Collision/part_{i:02d}", part, fr=fr)
    elif spec["kind"] == "tissue":
        sx, sy, sz = spec["size"]
        L.mesh("Visual/box", box(0, 0, 0, sx, sy, sz), spec["mat"])
        puffs = [ellipsoid([dx, dy, sz + dz], r, 2) for dx, dy, dz, r in
                 [(0.0, 0.0, 0.012, [0.035, 0.022, 0.018]), (0.015, 0.006, 0.026, [0.022, 0.018, 0.016]),
                  (-0.012, -0.004, 0.022, [0.02, 0.016, 0.014])]]
        L.mesh("Visual/tissue_puff", trimesh.util.concatenate(puffs), "tissue_paper", smooth=True)
        L.collider("Collision/box", box(0, 0, 0, sx, sy, sz), fr=fr)
    elif spec["kind"] == "toy":
        body = ellipsoid([-0.012, 0, 0.036], [0.050, 0.038, 0.036])
        head = ellipsoid([0.034, 0, 0.060], [0.030, 0.032, 0.030])
        snout = ellipsoid([0.058, 0, 0.052], [0.014, 0.018, 0.013], 2)
        ears = [ellipsoid([0.030, s * 0.022, 0.088], [0.009, 0.009, 0.008], 2) for s in (-1, 1)]
        feet = [ellipsoid([fx, s * 0.025, 0.010], [0.016, 0.012, 0.010], 2) for fx in (-0.04, 0.03) for s in (-1, 1)]
        L.mesh("Visual/plush", trimesh.util.concatenate([body, head, snout] + ears + feet), spec["mat"], smooth=True)
        ring = trimesh.creation.torus(major_radius=0.012, minor_radius=0.0012, major_sections=24, minor_sections=6)
        ring.apply_transform(trimesh.transformations.rotation_matrix(math.pi / 2, [1, 0, 0]))
        ring.apply_translation([-0.07, 0.0, 0.013])
        L.mesh("Visual/key_ring", ring, "steel", smooth=True)
        L.collider("Collision/body",
                   trimesh.convex.convex_hull(np.vstack([body.vertices, head.vertices, snout.vertices])), fr=fr)
    L.save()
    return L.path


def add_lights(L):
    sun = UsdLux.DistantLight.Define(L.stage, f"{L.rootpath}/Sun")
    sun.CreateIntensityAttr(3000.0)
    sun.CreateAngleAttr(0.53)
    sun.CreateColorAttr(Gf.Vec3f(1.0, 0.95, 0.88))
    z = SUN_TO / np.linalg.norm(SUN_TO)
    x = np.cross([0, 0, 1], z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    rot = Gf.Matrix3d(*x, *y, *z)
    UsdGeom.Xformable(sun).AddOrientOp(UsdGeom.XformOp.PrecisionFloat).Set(Gf.Quatf(rot.ExtractRotation().GetQuat()))
    r, nw, ew = LAYOUT["room"], LAYOUT["north_window"], LAYOUT["east_window"]
    zc = (nw["band_top"] + r["height"]) / 2
    for nm, (pos, rotxyz, w, h, inten) in {
        "WindowFillNorth": (((r["x0"] + nw["x1"]) / 2, r["y1"] - 0.15, zc), (-90, 0, 0), nw["x1"] - r["x0"],
                            r["height"] - nw["band_top"], 400.0),
        "WindowFillEast": ((r["x1"] - 0.15, (ew["y0"] + ew["y1"]) / 2, zc), (-90, 0, -90), ew["y1"] - ew["y0"],
                           r["height"] - ew["band_top"], 250.0),
    }.items():
        rl = UsdLux.RectLight.Define(L.stage, f"{L.rootpath}/{nm}")
        rl.CreateWidthAttr(float(w))
        rl.CreateHeightAttr(float(h))
        rl.CreateIntensityAttr(inten)
        rl.CreateColorAttr(Gf.Vec3f(0.92, 0.96, 1.0))
        xf = UsdGeom.Xformable(rl)
        xf.AddTranslateOp().Set(Gf.Vec3d(*(float(v) * SCALE for v in pos)))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(*rotxyz))
    for i, (cx, cy) in enumerate([(-1.2, 0.0), (1.0, 0.0)]):
        rl = UsdLux.RectLight.Define(L.stage, f"{L.rootpath}/CeilingPanel_{i}")
        rl.CreateWidthAttr(0.6)
        rl.CreateHeightAttr(0.6)
        rl.CreateIntensityAttr(0.0)
        UsdGeom.Xformable(rl).AddTranslateOp().Set(Gf.Vec3d(cx * SCALE, cy * SCALE, (r["height"] - 0.02) * SCALE))
    dome = UsdLux.DomeLight.Define(L.stage, f"{L.rootpath}/AmbientDome")
    dome.CreateIntensityAttr(150.0)
    dome.CreateColorAttr(Gf.Vec3f(0.9, 0.93, 1.0))


def scaled_params():
    k = SCALE
    table = {n: v * k for n, v in TABLE.items()}
    objects = {}
    for n, s in OBJECTS.items():
        o = dict(s)
        o["pos"] = (s["pos"][0] * k, s["pos"][1] * k)
        for key in ("height", "r_top", "r_bot", "wall"):
            if key in o:
                o[key] = s[key] * k
        if "size" in o:
            o["size"] = tuple(v * k for v in s["size"])
        objects[n] = o
    return table, objects


def main():
    global SCALE
    ap = argparse.ArgumentParser()
    ap.add_argument("--scale-correction", type=float, default=1.0)
    SCALE = ap.parse_args().scale_correction
    if OUT.exists():
        shutil.rmtree(OUT)
    (OUT / "objects").mkdir(parents=True)
    build_textures()
    bg = Layer(OUT / "background.usda", "Background")
    add_room(bg)
    add_furniture(bg)
    bg.save()
    tb = Layer(OUT / "table.usda", "Table")
    add_table(tb)
    tb.save()
    lt = Layer(OUT / "lights.usda", "Lights")
    add_lights(lt)
    lt.save()
    for n, s in OBJECTS.items():
        add_object(n, s)

    st = Usd.Stage.CreateNew(str(OUT / "scene.usda"))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(st, 1.0)
    world = UsdGeom.Xform.Define(st, "/World")
    st.SetDefaultPrim(world.GetPrim())
    UsdPhysics.Scene.Define(st, "/World/PhysicsScene").CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    for nm, f in [("Background", "background.usda"), ("Table", "table.usda"), ("Lights", "lights.usda")]:
        st.DefinePrim(f"/World/{nm}").GetReferences().AddReference(f"./{f}")
    UsdGeom.Xform.Define(st, "/World/Objects")
    for n, s in OBJECTS.items():
        p = st.DefinePrim(f"/World/Objects/{n}")
        p.GetReferences().AddReference(f"./objects/{n}.usda")
        xf = UsdGeom.Xformable(p)
        xf.AddTranslateOp().Set(Gf.Vec3d(s["pos"][0] * SCALE, s["pos"][1] * SCALE, TABLE["top_z"] * SCALE))
        xf.AddRotateZOp().Set(float(s["yaw"]))
    st.GetRootLayer().Save()

    params = dict(
        frame="Z-up, meters; origin on floor under task-table center; +X along table length; +Y from robot side toward sofa/window",
        scale_correction=SCALE, meters_per_sfm_unit=0.151 * SCALE,
        table=scaled_params()[0], objects=scaled_params()[1], layout_unscaled=LAYOUT, sun_direction_to_sun=SUN_TO.tolist(),
        layers=dict(scene="scene.usda", background="background.usda", table="table.usda", lights="lights.usda",
                    objects={n: f"objects/{n}.usda" for n in OBJECTS}),
    )
    (OUT / "build_params.json").write_text(json.dumps(params, indent=1), encoding="utf-8")
    print("wrote", OUT)


if __name__ == "__main__":
    main()
