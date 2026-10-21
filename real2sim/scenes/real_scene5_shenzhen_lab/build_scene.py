"""Build the ShenZhen lab desk scene (real_scene5) as modular USD layers, from the tape measurements in
registration.json and the photo fit (fit_photo.json, reference/IMG_49*.jpeg).

Scene frame: Z up, meters. Origin on the floor under the tabletop centre; +X along the long side; +Y towards the wall
behind the desk. The robot stands on the -Y side facing +Y (same convention as real_scene1-3).

The tabletop is fixed to the tape (1.20 x 0.60 m, 0.75 m high, 2 cm thick); the leg columns, feet and the cabinet's
position come from the fit, the cabinet's size from the registration (75 x 120 x 45 cm). Wall, desk and cabinet are
plain colours sampled from the photos; the carpet tile floor is a patch rectified from IMG_4913 at true scale. The wall
jog to the right of the cabinet and the clutter on the floor are not modelled.
"""

import json
from pathlib import Path
import shutil

import cv2
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt
from shapely.geometry import box as sbox
import trimesh

HERE = Path(__file__).resolve().parent
PHOTOS = HERE / "reference"
OUT = HERE / "usd"
TEX = OUT / "textures"
ROOM_NAME = "real_scene5"
FIT = json.loads((HERE / "fit_photo.json").read_text())
FX, F = FIT["fixed"], FIT["dims"]


def r3(v):
    return round(float(v), 3)


TABLE = dict(length=FX["length"], width=FX["width"], top_z=FX["top_z"], thick=FX["thick"], corner_r=0.05)
# Lift-desk T legs: two-stage column on a foot bar along Y, side rails under the top, a cross beam between the columns.
LEGS = dict(
    x=r3(F["col_x"]),
    y=r3(F["col_y"]),
    foot_front=r3(F["foot_front"]),
    foot_back=r3(F["foot_back"]),
    upper=(0.055, 0.045),  # column sections (x, y), from the photos' pixel widths
    lower=(0.065, 0.055),
    split_z=0.45,  # where the inner column leaves the outer one (IMG_4918, IMG_4913)
    foot=(0.06, 0.035),  # foot bar width, height
)
CABINET = dict(x0=r3(F["cab_x0"]), w=FX["cab_w"], h=FX["cab_h"], d=FX["cab_d"], y1=FX["wall_y"])
SKIRTING = 0.08  # black marble skirting along the wall, height estimated from IMG_4916
# Back wall from the registration (clearance 0); the side and front walls were not photographed: placed out of view.
ROOM = dict(x0=-2.5, x1=2.5, y0=-3.0, y1=FX["wall_y"], height=2.8)
# clean carpet patch in front of the cabinet in IMG_4913 (scene x0, x1, y0, y1); the photo pixels already carry the
# room light; gain set so the RTX floor matches IMG_4916 (0.67 as in real_scene3 rendered it 78 vs 98 sRGB, 0.9 gave 105)
CARPET = dict(patch=(0.85, 1.15, -1.5, -1.2), photo="IMG_4913.jpeg", gain=0.85)

# Registered: ceiling strip lights (area lights), colour matched to the photos. Positions and brightness not recorded:
# four 1.2 x 0.15 m strips along X over the desk and the robot, at the ceiling.
LIGHTS = dict(
    strips=[(0.0, -0.2), (0.0, -1.0), (-1.6, -0.6), (1.6, -0.6)],
    strip_size=(1.2, 0.15),
    intensity=54000.0,  # wall / desk brightness against IMG_4916 under RTX (60000 rendered the wall 225 vs 206 sRGB)
    # Isaac cameras have no white balance; the colours are in the materials (sRGB from the photos)
    color_temperature=6000.0,  # 6500 K rendered the cream wall neutral (R/B 1.01 vs 1.05)
    dome=150.0,
)


def lin(srgb):
    c = np.asarray(srgb, float) / 255
    return tuple(float(v) for v in np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4))


# name: (diffuse linear RGB, roughness, metallic, opacity, texture). sRGB sampled in IMG_4916, scaled up like
# real_scene3's tabletop (photo pixels carry the room light): wall (206, 202, 195), desk (196, 196, 191), cabinet
# (162, 161, 157); the wall back to its photo value after the RTX check (222 rendered it 225 vs 206 sRGB).
MATERIALS = {
    "laminate": (lin((228, 228, 223)), 0.35, 0.0, 1.0, None),
    "frame_white": (lin((215, 216, 214)), 0.4, 0.2, 1.0, None),
    "cap_grey": (lin((150, 150, 148)), 0.6, 0.0, 1.0, None),
    "steel": (lin((120, 125, 130)), 0.35, 0.8, 1.0, None),
    "cabinet": (lin((190, 189, 184)), 0.45, 0.3, 1.0, None),
    "seam": (lin((40, 40, 40)), 0.6, 0.0, 1.0, None),
    "skirting": (lin((14, 15, 14)), 0.25, 0.0, 1.0, None),
    "wall": (lin((205, 201, 194)), 0.9, 0.0, 1.0, None),
    "ceiling": (lin((230, 230, 230)), 0.9, 0.0, 1.0, None),
    "carpet": ((1.0, 1.0, 1.0), 0.95, 0.0, 1.0, "carpet.png"),
}
WRAP = {"carpet.png": "mirror"}
PHYS = {"default": (0.5, 0.45, 0.0), "laminate": (0.5, 0.4, 0.0), "carpet": (0.9, 0.8, 0.0)}


# ----------------------------------------------------------------------------- geometry helpers
def box_xyz(x0, x1, y0, y1, z0, z1):
    m = np.eye(4)
    m[:3, 3] = [(x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2]
    return trimesh.creation.box(extents=[x1 - x0, y1 - y0, z1 - z0], transform=m)


def box_c(x, y, sx, sy, z0, z1):
    return box_xyz(x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2, z0, z1)


def quad(p0, p1, p2, p3):
    return trimesh.Trimesh(vertices=[p0, p1, p2, p3], faces=[[0, 1, 2], [0, 2, 3]], process=False)


def rounded_rect(cx, cy, sx, sy, r):
    return sbox(cx - sx / 2 + r, cy - sy / 2 + r, cx + sx / 2 - r, cy + sy / 2 - r).buffer(r, quad_segs=6)


# ----------------------------------------------------------------------------- USD helpers
class Layer:
    def __init__(self, path, root):
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

    def mesh(self, path, m, mat=None, uv=None, guide=False):
        um = UsdGeom.Mesh.Define(self.stage, f"{self.rootpath}/{path}")
        pts = m.vertices.astype(np.float32)
        um.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts))
        um.CreateFaceVertexCountsAttr(Vt.IntArray([3] * len(m.faces)))
        um.CreateFaceVertexIndicesAttr(Vt.IntArray(m.faces.reshape(-1).tolist()))
        um.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        um.CreateExtentAttr(Vt.Vec3fArray([Gf.Vec3f(*map(float, pts.min(0))), Gf.Vec3f(*map(float, pts.max(0)))]))
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

    def collider(self, path, m, pkey="default"):
        """Static convex-hull collider (guide purpose). All colliders here are boxes: 8 vertices, 4-vertex faces."""
        prim = self.mesh(path, m, guide=True).GetPrim()
        UsdPhysics.CollisionAPI.Apply(prim)
        UsdPhysics.MeshCollisionAPI.Apply(prim).CreateApproximationAttr("convexHull")
        UsdShade.MaterialBindingAPI.Apply(prim).Bind(
            self.phys_material(pkey), UsdShade.Tokens.weakerThanDescendants, "physics"
        )

    def part(self, path, m, mat, pkey="default"):
        self.mesh(path, m, mat)
        self.collider(f"Colliders/{path.replace('/', '_')}", m, pkey)

    def save(self):
        self.stage.GetRootLayer().Save()


# ----------------------------------------------------------------------------- floor texture (from a photo)
def load_photo(name):
    img = cv2.imread(str(PHOTOS / name))  # EXIF orientation applied
    k = 1024 / max(img.shape[:2])
    return cv2.resize(img, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)


def photo_px(name, pts):
    c, f = FIT[name], FIT["focal_px"]
    w, h = c["size"]
    k = np.array([[f, 0, w / 2], [0, f, h / 2], [0, 0, 1]], np.float64)
    px, _ = cv2.projectPoints(np.array(pts, np.float64), np.array(c["rvec"]), np.array(c["tvec"]), k, None)
    return px.reshape(-1, 2)


def flatten_light(img, sigma):
    img = img.astype(np.float32)
    g = cv2.GaussianBlur(img, (0, 0), sigma)
    return np.clip(img / np.maximum(g, 1) * img.reshape(-1, 3).mean(0), 0, 255)


def carpet_texture(size=(512, 512)):
    x0, x1, y0, y1 = CARPET["patch"]
    corners = [(x0, y1, 0), (x1, y1, 0), (x1, y0, 0), (x0, y0, 0)]
    w, h = size
    m = cv2.getPerspectiveTransform(
        np.float32(photo_px(CARPET["photo"], corners)), np.float32([[0, 0], [w, 0], [w, h], [0, h]])
    )
    tex = cv2.warpPerspective(
        load_photo(CARPET["photo"]), m, size, flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE
    )
    return np.clip(flatten_light(tex, 80) * CARPET["gain"], 0, 255).astype(np.uint8)


# ----------------------------------------------------------------------------- scene parts
def add_table(L):
    t, g = TABLE, LEGS
    hl, hw, top = t["length"] / 2, t["width"] / 2, t["top_z"]
    z_under = top - t["thick"]
    outline = rounded_rect(0, 0, t["length"], t["width"], t["corner_r"])
    slab = trimesh.creation.extrude_polygon(outline, t["thick"]).apply_translation([0, 0, z_under])
    L.mesh("Top/slab", slab, "laminate")
    L.collider("Colliders/top", box_xyz(-hl, hl, -hw, hw, z_under, top), pkey="laminate")

    (ux, uy), (lx, ly), (fw, fh) = g["upper"], g["lower"], g["foot"]
    rail_z0 = z_under - 0.04
    for sx, side in ((1, "pos"), (-1, "neg")):
        x = sx * g["x"]
        L.part(f"Legs/{side}/rail", box_c(x, 0, 0.05, t["width"] - 0.1, rail_z0, z_under), "frame_white")
        L.part(f"Legs/{side}/upper", box_c(x, g["y"], ux, uy, g["split_z"], rail_z0), "frame_white")
        L.part(f"Legs/{side}/lower", box_c(x, g["y"], lx, ly, fh, g["split_z"]), "frame_white")
        L.part(
            f"Legs/{side}/foot", box_xyz(x - fw / 2, x + fw / 2, g["foot_front"], g["foot_back"], 0, fh), "frame_white"
        )
        for end, (y0, y1) in (
            ("front", (g["foot_front"], g["foot_front"] + 0.04)),
            ("back", (g["foot_back"] - 0.04, g["foot_back"])),
        ):
            L.mesh(
                f"Legs/{side}/cap_{end}",
                box_xyz(x - fw / 2 - 0.002, x + fw / 2 + 0.002, y0, y1, 0, fh + 0.002),
                "cap_grey",
            )
    # steel cross beam between the column tops (visible in IMG_4918, IMG_4914)
    L.part("Frame/beam", box_c(0, g["y"] + 0.05, 2 * g["x"], 0.03, rail_z0 - 0.03, rail_z0), "steel")


def add_cabinet(L):
    c = CABINET
    x0, x1, y0, y1 = c["x0"], c["x0"] + c["w"], c["y1"] - c["d"], c["y1"]
    L.part("Cabinet/body", box_xyz(x0, x1, y0, y1, 0.0, c["h"]), "cabinet")
    # two by two locker doors, separated by dark seams on the front (-Y) face
    xm, zm, gap = (x0 + x1) / 2, c["h"] * 0.55, 0.004
    for i, (dx0, dx1) in enumerate(((x0 + 0.02, xm - gap), (xm + gap, x1 - 0.02))):
        for j, (dz0, dz1) in enumerate(((0.06, zm - gap), (zm + gap, c["h"] - 0.04))):
            L.mesh(f"Cabinet/door_{i}{j}", box_xyz(dx0, dx1, y0 - 0.004, y0, dz0, dz1), "cabinet")
            L.mesh(f"Cabinet/seam_{i}{j}", box_xyz(dx0 - gap, dx1 + gap, y0 - 0.002, y0, dz0 - gap, dz1 + gap), "seam")
            L.mesh(
                f"Cabinet/lock_{i}{j}",
                box_xyz(dx0 + 0.03, dx0 + 0.08, y0 - 0.012, y0 - 0.004, dz1 - 0.12, dz1 - 0.10),
                "steel",
            )


def add_background(L):
    r, hgt = ROOM, ROOM["height"]
    x0, x1, y0, y1 = CARPET["patch"]
    L.mesh(
        "Floor",
        quad([r["x0"], r["y0"], 0], [r["x1"], r["y0"], 0], [r["x1"], r["y1"], 0], [r["x0"], r["y1"], 0]),
        "carpet",
        uv=lambda fv: (fv[:, :2] - [x0, y0]) / [x1 - x0, y1 - y0],
    )
    # closes the room: without it RoboDojo's default brown studio HDR floods the scene from above
    L.mesh(
        "Ceiling",
        quad([r["x0"], r["y0"], hgt], [r["x0"], r["y1"], hgt], [r["x1"], r["y1"], hgt], [r["x1"], r["y0"], hgt]),
        "ceiling",
    )
    L.collider("Colliders/floor", box_xyz(r["x0"] - 1, r["x1"] + 1, r["y0"] - 1, r["y1"] + 1, -0.10, 0.0), "carpet")
    walls = {
        "back": ((r["x0"], r["y1"]), (r["x1"], r["y1"]), (0, 0.1)),
        "front": ((r["x1"], r["y0"]), (r["x0"], r["y0"]), (0, -0.1)),
        "left": ((r["x0"], r["y0"]), (r["x0"], r["y1"]), (-0.1, 0)),
        "right": ((r["x1"], r["y1"]), (r["x1"], r["y0"]), (0.1, 0)),
    }
    for name, (a, b, (ox, oy)) in walls.items():
        L.mesh(f"Walls/{name}", quad((*a, 0), (*b, 0), (*b, hgt), (*a, hgt)), "wall")
        xs, ys = sorted((a[0], b[0], a[0] + ox, b[0] + ox)), sorted((a[1], b[1], a[1] + oy, b[1] + oy))
        L.collider(f"Colliders/wall_{name}", box_xyz(xs[0], xs[-1], ys[0], ys[-1], 0.0, hgt))
        # skirting, 1 cm proud of the wall
        sx, sy = (
            sorted((a[0], b[0], a[0] - ox / 10, b[0] - ox / 10)),
            sorted((a[1], b[1], a[1] - oy / 10, b[1] - oy / 10)),
        )
        L.mesh(f"Walls/skirting_{name}", box_xyz(sx[0], sx[-1], sy[0], sy[-1], 0.0, SKIRTING), "skirting")


def add_lights(L):
    lt = LIGHTS
    for i, (x, y) in enumerate(lt["strips"]):
        rl = UsdLux.RectLight.Define(L.stage, f"{L.rootpath}/CeilingStrip_{i}")
        rl.CreateWidthAttr(lt["strip_size"][0])
        rl.CreateHeightAttr(lt["strip_size"][1])
        rl.CreateIntensityAttr(lt["intensity"])
        rl.CreateEnableColorTemperatureAttr(True)
        rl.CreateColorTemperatureAttr(lt["color_temperature"])
        UsdGeom.Xformable(rl).AddTranslateOp().Set(Gf.Vec3d(x, y, ROOM["height"] - 0.02))
    dome = UsdLux.DomeLight.Define(L.stage, f"{L.rootpath}/AmbientDome")
    dome.CreateIntensityAttr(lt["dome"])
    dome.CreateColorAttr(Gf.Vec3f(0.95, 0.95, 1.0))


def main():
    if OUT.exists():
        shutil.rmtree(OUT)
    TEX.mkdir(parents=True)
    cv2.imwrite(str(TEX / "carpet.png"), carpet_texture())
    layers = [("background", add_background), ("table", add_table), ("cabinet", add_cabinet), ("lights", add_lights)]
    for name, fn in layers:
        layer = Layer(OUT / f"{name}.usda", name.capitalize())
        fn(layer)
        layer.save()
    names = [n.capitalize() for n, _ in layers]

    st = Usd.Stage.CreateNew(str(OUT / "scene.usda"))
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
    # the env owns it. Installed by scripts/g1d/build_asset.py.
    room = Usd.Stage.CreateNew(str(HERE / f"{ROOM_NAME}.usd"))
    UsdGeom.SetStageUpAxis(room, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(room, 1.0)
    scene = UsdGeom.Xform.Define(room, "/Scene")
    room.SetDefaultPrim(scene.GetPrim())
    for nm in names:
        room.DefinePrim(f"/Scene/{nm}").GetReferences().AddReference(f"./usd/{nm.lower()}.usda")
    room.GetRootLayer().Save()

    t = TABLE
    params = dict(
        frame="Z-up, meters; origin on the floor under the tabletop centre; +X along the table; +Y to the wall",
        table=dict(top_z=t["top_z"], length=t["length"], width=t["width"], front_y=-t["width"] / 2, thick=t["thick"]),
        legs=LEGS,
        cabinet=CABINET,
        room=ROOM,
        lights=LIGHTS,
        layers=dict(scene="scene.usda", **{n: f"{n}.usda" for n, _ in layers}),
    )
    (OUT / "build_params.json").write_text(json.dumps(params, indent=1), encoding="utf-8")
    for f in OUT.glob("*.usda"):  # USD ends text layers with a blank line, which `git diff --check` rejects
        f.write_text(f.read_text().rstrip("\n") + "\n")
    print("wrote", OUT)


if __name__ == "__main__":
    main()
