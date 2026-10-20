"""Build the school lab green-screen table (real_scene4) as modular USD layers, from the colleague's MuJoCo twin
(sys01 g1d_food_sorting bundle, copied to 5090-2:/root/g1d_sim_assets_20261020; see PLAN.md).

That twin fitted the tabletop in the frame of the robot's initial torso_link (+X forward, +Y left, +Z up) from the real
wrist and head camera frames: bundle/environment_manifest.json `table` (four corners, top z, thickness) and the floor
under the robot's nominal CAD. The green backdrop is the box `backdrop` of scenes/food_episode_0000.xml. Here they move
into the chassis frame (floor under AGV_link) with the torso yaw recorded at episode start, the four corners get a
least-squares rectangle, and the scene frame follows real_scene1-3.

Scene frame: Z up, meters. Origin on the floor under the tabletop centre; +X along the long side (the robot's right);
+Y away from the robot. The robot stands on the -Y side, yawed by the recorded torso yaw left over after the fit
(build_params.json `robot`, copied by hand into env_cfg/robot/g1d_school.yml).

Usage (pxr and numpy only, e.g. `uv run --with usd-core --with numpy python build_scene.py`):
    python real2sim/scenes/real_scene4_school/build_scene.py
"""

import json
import math
from pathlib import Path
import shutil

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics, UsdShade, Vt

HERE = Path(__file__).resolve().parent
OUT = HERE / "usd"
MANIFEST = json.loads((HERE / "bundle/environment_manifest.json").read_text())

# Torso yaw at the start of episode_0000: raw LowState motor 12 (waist yaw, which is torso_Joint on G1-D), rad
# (demonstrations/episode_0000/reference.npz raw_motor_qpos[0, 12]; episode_0001 started at 0.0604).
TORSO_YAW = 0.06698
# torso_link origin seen from the floor point under AGV_link, zero lift and waist: x, y from the g1d_v4 URDF chain,
# z = AGV_link height 0.1115 + 0.7335 up to the torso (= -nominal_z_torso_m of the bundle floor, 0.8448).
TORSO_IN_CHASSIS = np.array([0.0534, 0.0002, -MANIFEST["floor"]["nominal_z_torso_m"]])
# Green backdrop in the torso frame (food_episode_0000.xml geom `backdrop`): centre, half sizes; visual only there.
BACKDROP = dict(centre=(0.7827, -0.0885, 0.4408), half=(0.01, 0.9885, 1.0))
# bundle rgba (no texture was fitted) is display sRGB; the shaders take linear colour
TABLE_RGB = tuple(round(c**2.2, 4) for c in (0.70, 0.61, 0.45))
BACKDROP_RGB = tuple(round(c**2.2, 4) for c in (0.025, 0.32, 0.23))
# Not observed: a closed neutral room (keeps RoboDojo's brown HDR out) and ceiling panels as in real_scene3.
ROOM = dict(half_x=2.5, y0=-2.0, y1=2.5, height=2.8)
LIGHTS = dict(
    panels=[(-1.0, -1.0), (1.0, -1.0), (-1.0, 1.0), (1.0, 1.0)],
    size=(1.2, 0.6),
    intensity=24000.0,
    color_temperature=6500.0,
    dome=150.0,
)
LEGS = dict(inset=0.05, side=0.04)  # not observed: four square legs so the board does not float in the head view


def r4(v):
    return round(float(v), 4)


def chassis(p_torso):
    c, s = math.cos(TORSO_YAW), math.sin(TORSO_YAW)
    x, y, z = p_torso
    return np.array([c * x - s * y, s * x + c * y, z]) + TORSO_IN_CHASSIS


def fit_table():
    """Rectangle through the four fitted corners (near-left, far-left, far-right, near-right as the robot sees them)."""
    t = MANIFEST["table"]
    a, b, c, d = (chassis(p)[:2] for p in t["polygon_root_m"])
    u = (d - a) + (c - b)
    u /= np.linalg.norm(u)  # long axis, towards the robot's right
    v = np.array([-u[1], u[0]])  # away from the robot
    centre = (a + b + c + d) / 4
    length = (np.dot(d - a, u) + np.dot(c - b, u)) / 2
    width = (np.dot(b - a, v) + np.dot(c - d, v)) / 2
    corners = [a, b, c, d]
    rect = [centre + su * length / 2 * u + sv * width / 2 * v for su, sv in ((-1, -1), (-1, 1), (1, 1), (1, -1))]
    resid = max(np.linalg.norm(p - q) for p, q in zip(corners, rect))
    top_z = chassis((0, 0, t["table_top_z_m"]))[2]
    return dict(centre=centre, u=u, v=v, length=length, width=width, thick=t["thickness_m"], top_z=top_z, resid=resid)


def to_scene(fit, p):
    q = np.asarray(p[:2]) - fit["centre"]
    return np.array([np.dot(q, fit["u"]), np.dot(q, fit["v"]), p[2]])


# ----------------------------------------------------------------------------- USD
def box(x0, x1, y0, y1, z0, z1):
    pts = np.array([(x, y, z) for z in (z0, z1) for y in (y0, y1) for x in (x0, x1)])
    faces = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    return pts, faces


def box_at(centre, half, yaw=0.0):
    pts, faces = box(-half[0], half[0], -half[1], half[1], -half[2], half[2])
    c, s = math.cos(yaw), math.sin(yaw)
    pts = pts @ np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]]) + centre
    return pts, faces


class Layer:
    def __init__(self, path, root):
        self.stage = Usd.Stage.CreateNew(str(path))
        UsdGeom.SetStageUpAxis(self.stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(self.stage, 1.0)
        UsdPhysics.SetStageKilogramsPerUnit(self.stage, 1.0)
        self.root = f"/{root}"
        self.stage.SetDefaultPrim(UsdGeom.Xform.Define(self.stage, self.root).GetPrim())
        self.mats = {}

    def material(self, rgb, rough):
        if rgb in self.mats:
            return self.mats[rgb]
        key = f"m{len(self.mats)}"
        mat = UsdShade.Material.Define(self.stage, f"{self.root}/Looks/{key}")
        sh = UsdShade.Shader.Define(self.stage, f"{self.root}/Looks/{key}/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        self.mats[rgb] = mat
        return mat

    def mesh(self, path, pts, faces, rgb=None, rough=0.6, collide=False):
        m = UsdGeom.Mesh.Define(self.stage, f"{self.root}/{path}")
        m.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(np.asarray(pts, np.float32)))
        m.CreateFaceVertexCountsAttr(Vt.IntArray([len(f) for f in faces]))
        m.CreateFaceVertexIndicesAttr(Vt.IntArray([i for f in faces for i in f]))
        m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        m.CreateExtentAttr(
            Vt.Vec3fArray([Gf.Vec3f(*map(float, np.min(pts, 0))), Gf.Vec3f(*map(float, np.max(pts, 0)))])
        )
        if rgb is not None:
            UsdShade.MaterialBindingAPI.Apply(m.GetPrim()).Bind(self.material(rgb, rough))
        if collide:
            UsdPhysics.CollisionAPI.Apply(m.GetPrim())
            UsdPhysics.MeshCollisionAPI.Apply(m.GetPrim()).CreateApproximationAttr("convexHull")
        return m

    def save(self):
        self.stage.GetRootLayer().Save()


def add_table(L, fit):
    hl, hw, top, th = fit["length"] / 2, fit["width"] / 2, fit["top_z"], fit["thick"]
    L.mesh("Top", *box(-hl, hl, -hw, hw, top - th, top), TABLE_RGB, 0.55, collide=True)
    s, i = LEGS["side"] / 2, LEGS["inset"]
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = sx * (hl - i), sy * (hw - i)
            L.mesh(
                f"Leg_{'pn'[sx < 0]}{'pn'[sy < 0]}",
                *box(x - s, x + s, y - s, y + s, 0.0, top - th),
                (0.2, 0.2, 0.2),
                collide=True,
            )


def add_background(L, fit):
    r = ROOM
    L.mesh("Floor", *box(-r["half_x"], r["half_x"], r["y0"], r["y1"], -0.1, 0.0), (0.4, 0.4, 0.4), 0.9, collide=True)
    L.mesh(
        "Ceiling", *box(-r["half_x"], r["half_x"], r["y0"], r["y1"], r["height"], r["height"] + 0.05), (0.8, 0.8, 0.8)
    )
    for name, b in {
        "Wall_left": (-r["half_x"] - 0.05, -r["half_x"], r["y0"], r["y1"]),
        "Wall_right": (r["half_x"], r["half_x"] + 0.05, r["y0"], r["y1"]),
        "Wall_back": (-r["half_x"], r["half_x"], r["y0"] - 0.05, r["y0"]),
        "Wall_front": (-r["half_x"], r["half_x"], r["y1"], r["y1"] + 0.05),
    }.items():
        L.mesh(name, *box(*b, 0.0, r["height"]), (0.75, 0.75, 0.73), 0.9)
    # the bundle's box is thin along torso +X; built thin along scene +Y, then turned onto that axis
    centre = to_scene(fit, chassis(BACKDROP["centre"]))
    n = np.array([math.cos(TORSO_YAW), math.sin(TORSO_YAW)])
    yaw = math.atan2(np.dot(n, fit["v"]), np.dot(n, fit["u"])) - math.pi / 2
    half = (BACKDROP["half"][1], BACKDROP["half"][0], BACKDROP["half"][2])
    L.mesh("Backdrop", *box_at(centre, half, yaw), BACKDROP_RGB, 0.9)
    return centre, yaw


def add_lights(L):
    lt = LIGHTS
    for i, (x, y) in enumerate(lt["panels"]):
        rl = UsdLux.RectLight.Define(L.stage, f"{L.root}/CeilingPanel_{i}")
        rl.CreateWidthAttr(lt["size"][0])
        rl.CreateHeightAttr(lt["size"][1])
        rl.CreateIntensityAttr(lt["intensity"])
        rl.CreateEnableColorTemperatureAttr(True)
        rl.CreateColorTemperatureAttr(lt["color_temperature"])
        UsdGeom.Xformable(rl).AddTranslateOp().Set(Gf.Vec3d(x, y, ROOM["height"] - 0.02))
    dome = UsdLux.DomeLight.Define(L.stage, f"{L.root}/AmbientDome")
    dome.CreateIntensityAttr(lt["dome"])
    dome.CreateColorAttr(Gf.Vec3f(0.95, 0.95, 1.0))


def robot_stand(fit):
    """Robot root (AGV_link) in the scene frame, its heading (rad from scene +X) and table-edge distance."""
    root = to_scene(fit, np.array([0.0, 0.0, 0.0]))
    heading = math.atan2(np.dot([1, 0], fit["v"]), np.dot([1, 0], fit["u"]))
    return root, heading, -fit["width"] / 2 - root[1]


def main():
    fit = fit_table()
    assert fit["resid"] < 0.03, f"fitted corners are {fit['resid']:.3f} m off a rectangle"
    if OUT.exists():
        shutil.rmtree(OUT)
    OUT.mkdir(parents=True)
    layers = {name: Layer(OUT / f"{name}.usda", name.capitalize()) for name in ("background", "table", "lights")}
    backdrop = add_background(layers["background"], fit)
    add_table(layers["table"], fit)
    add_lights(layers["lights"])
    for layer in layers.values():
        layer.save()

    st = Usd.Stage.CreateNew(str(OUT / "scene.usda"))
    UsdGeom.SetStageUpAxis(st, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(st, 1.0)
    UsdPhysics.SetStageKilogramsPerUnit(st, 1.0)
    st.SetDefaultPrim(UsdGeom.Xform.Define(st, "/World").GetPrim())
    UsdPhysics.Scene.Define(st, "/World/PhysicsScene").CreateGravityDirectionAttr(Gf.Vec3f(0, 0, -1))
    for nm in ("Background", "Table", "Lights"):
        st.DefinePrim(f"/World/{nm}").GetReferences().AddReference(f"./{nm.lower()}.usda")
    st.GetRootLayer().Save()
    # RoboDojo Room entry (layout_manager.select_room loads the only .usd in Assets/Room/<name>/): no PhysicsScene
    room = Usd.Stage.CreateNew(str(HERE / "real_scene4.usd"))
    UsdGeom.SetStageUpAxis(room, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(room, 1.0)
    room.SetDefaultPrim(UsdGeom.Xform.Define(room, "/Scene").GetPrim())
    for nm in ("Background", "Table", "Lights"):
        room.DefinePrim(f"/Scene/{nm}").GetReferences().AddReference(f"./usd/{nm.lower()}.usda")
    room.GetRootLayer().Save()

    root, heading, edge = robot_stand(fit)
    params = dict(
        frame="Z-up, meters; origin on the floor under the tabletop centre; +X along the table; +Y away from the robot",
        table=dict(
            top_z=r4(fit["top_z"]),
            length=r4(fit["length"]),
            width=r4(fit["width"]),
            front_y=r4(-fit["width"] / 2),
            thick=r4(fit["thick"]),
            rectangle_fit_residual_m=r4(fit["resid"]),
        ),
        robot=dict(
            root_xy=[r4(root[0]), r4(root[1])],
            heading_deg=round(math.degrees(heading), 2),
            table_edge_from_root=r4(edge),
            torso_yaw=TORSO_YAW,
        ),
        backdrop=dict(centre=[r4(x) for x in backdrop[0]], yaw_deg=round(math.degrees(backdrop[1]), 2)),
        layers=dict(scene="scene.usda", background="background.usda", table="table.usda", lights="lights.usda"),
    )
    (OUT / "build_params.json").write_text(json.dumps(params, indent=1), encoding="utf-8")
    for f in OUT.glob("*.usda"):  # USD ends text layers with a blank line, which `git diff --check` rejects
        f.write_text(f.read_text().rstrip("\n") + "\n")
    print(json.dumps(params["table"]), json.dumps(params["robot"]), json.dumps(params["backdrop"]))


if __name__ == "__main__":
    main()
