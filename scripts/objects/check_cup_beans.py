"""Fill the plastic cups with the g1d tasks' granular red beans, let them settle, and render them.

Cups (make_plastic_cups.py) are kinematic and loaded with RigidObject; beans are FluidObject PBD solid particles with
RED_BEAN_PHYSICS (4 mm spheres), as in g1d_pour_beans_cup_to_cup. Writes under --out:
    spheres.png   the beans as simulated (particle spheres, RedBean.mdl)
    beans.png     the same settled positions drawn with the red_bean mesh (make_red_bean.py), random orientation,
                  display only; top.png the same from above
    closeup.png   the 500 ml cup from the side, bean mesh
    bean.png      three single beans close up
    metrics.json  particle counts and the fraction still inside each cup

PYTHONPATH=. python scripts/objects/check_cup_beans.py --out output/plastic_cups/beans
The CPU state pipeline allows particle USD readback; PBD physics still requires a GPU (see check_pitcher_fluid.py).
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--rigid-dir", default="Assets/Object/RoboDojo/Rigid")
parser.add_argument("--fluid-dir", default="Assets/Object/RoboDojo/Fluid/wuliangye/00000")
parser.add_argument("--out", default="output/plastic_cups/beans")
parser.add_argument("--seconds", type=float, default=2.0)
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(device="cpu")
args = parser.parse_args()
if args.device != "cpu":
    parser.error("Use --device cpu for particle USD readback; PBD physics still runs on the GPU.")
args.headless = True
args.enable_cameras = True
app = AppLauncher(args).app

import json  # noqa: E402
import os  # noqa: E402
from pathlib import Path  # noqa: E402
import sys  # noqa: E402
import tempfile  # noqa: E402
import traceback  # noqa: E402

import carb  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
import numpy as np  # noqa: E402
from omni.physx import get_physx_interface  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics, Vt  # noqa: E402
from scipy.spatial.transform import Rotation  # noqa: E402
import torch  # noqa: E402

from env.scene_manager.objects.fluid import FluidObject  # noqa: E402
from env.scene_manager.objects.rigid import RigidObject  # noqa: E402
from scripts.g1d.build_asset import RED_BEAN_MDL, RED_BEAN_PHYSICS  # noqa: E402
from scripts.objects import (
    make_plastic_cups as pc,  # noqa: E402
    make_red_bean as rb,  # noqa: E402
)

DT = 0.004
GPU_OVERRIDE = get_physx_interface().get_overwrite_gpu_setting()
MARGIN = 0.003  # fill region inset from the inner wall/bottom: particle radius + contact offset
BEAN_SCALE = 0.85  # bean mesh (5.8 mm) drawn near the 4 mm particle spacing
# category/index, fill height above the outside bottom (m), y position
CUPS = [("plastic_mug/00000", 0.055, -0.16), ("measuring_cup/00000", 0.085, 0.0), ("measuring_cup/00001", 0.105, 0.16)]


def fill_region(category, top):
    """Convex point cloud inside the cup cavity, in the cup frame with origin at the outside bottom centre."""
    if category == "plastic_mug":
        sec, r, tb = pc.unit_section(2.0), pc.MUG["R0"] - pc.MUG["T_WALL"], pc.MUG["T_BOTTOM"]
    else:
        sec, r, tb = pc.unit_section(pc.CUP["SQ"]), pc.CUP["A0"] - pc.CUP["T_WALL"], pc.CUP["T_BOTTOM"]
    ring = (r - MARGIN) * sec[::4]
    return np.array([(x, y, z) for z in (tb + MARGIN, top) for x, y in ring])


def write_fill_usd(points):
    path = Path(tempfile.mkdtemp()) / "fill.usda"
    st = Usd.Stage.CreateNew(str(path))
    st.SetDefaultPrim(UsdGeom.Xform.Define(st, "/root").GetPrim())
    UsdGeom.Mesh.Define(st, "/root/model").CreatePointsAttr(Vt.Vec3fArray.FromNumpy(points.astype(np.float32)))
    st.Save()
    return str(path)


def add_camera(path, eye, target, focal=None):
    cam = Camera(prim_path=path, resolution=(1280, 720))
    set_camera_view(eye, target, camera_prim_path=path)
    usd_cam = UsdGeom.Camera(get_current_stage().GetPrimAtPath(path))
    usd_cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.002, 100.0))
    if focal:
        usd_cam.GetFocalLengthAttr().Set(focal)
    return cam


def add_bean(stage, path, pos, quat_wxyz):
    x = UsdGeom.Xform.Define(stage, path)
    x.AddTranslateOp().Set(Gf.Vec3d(*pos))
    x.AddOrientOp().Set(Gf.Quatf(*map(float, quat_wxyz)))
    rb.define_visual(stage, path)


def main():
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rigid_dir, fluid_dir = Path(args.rigid_dir).resolve(), Path(args.fluid_dir).resolve()
    fluid_meta = json.loads((fluid_dir / "metadata.json").read_text())
    asset_root = fluid_dir.parents[4]
    bean_cfg = {
        "physics": dict(fluid_meta["physics"], **RED_BEAN_PHYSICS),
        "visual": {"visual_mdl_path": RED_BEAN_MDL.replace("$Robodojo_ASSETS", str(asset_root)), "scale": [1, 1, 1]},
    }

    carb.settings.get_settings().set_float("/rtx/translucency/worldEps", 0.0)
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(
            dt=DT,
            device=args.device,
            use_fabric=False,
            render=sim_utils.RenderCfg(
                enable_translucency=True,
                enable_reflections=True,
                enable_global_illumination=True,
                rendering_mode="quality",
            ),
        )
    )
    stage = get_current_stage()
    PhysxSchema.PhysxSceneAPI(stage.GetPrimAtPath(sim.cfg.physics_prim_path)).CreateEnableGPUDynamicsAttr().Set(True)
    get_physx_interface().overwrite_gpu_setting(1)
    floor = sim_utils.CuboidCfg(
        size=(2, 2, 0.02),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.36, 0.38)),
    )
    floor.func("/World/floor", floor, translation=(0, 0, -0.01))
    light = sim_utils.DomeLightCfg(intensity=300)
    light.func("/World/light", light)
    sun = sim_utils.DistantLightCfg(intensity=2500, angle=5)
    sun.func("/World/sun", sun, orientation=(0.9397, 0.2418, 0.2418, 0))

    cups = []
    for k, (name, top, y) in enumerate(CUPS):
        d = rigid_dir / name
        meta = json.loads((d / "metadata.json").read_text())
        half_h = meta["physics"]["size"][2] / 2
        axis = np.array([0.0, y, 0.0]) + [*meta["active"]["place"]["up"]["contact_circle"]["center"][:2], 0]
        root = f"/World/envs/env_{k}"
        path = f"{root}/Rigid/{d.parent.name}/{d.parent.name}_{int(d.name)}_0"
        obj = RigidObject(
            prim_path=path,
            usd_path=str(d / "object.usdz"),
            inst_config={"physics": {"mass": meta["physics"]["mass"]}},
            env_origin=[0, 0, 0],
            default_pos=(0.0, y, half_h),
            default_ori=(1, 0, 0, 0),
            scale=(1, 1, 1),
        )
        UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).CreateKinematicEnabledAttr().Set(True)
        fluid = FluidObject(
            prim_path=f"{root}/Fluid/red_bean/red_bean_0_0",
            usd_path=write_fill_usd(fill_region(d.parent.name, top)),
            inst_config=json.loads(json.dumps(bean_cfg)),
            env_origin=torch.zeros(3),
            default_pos=tuple(axis),
            default_ori=(1, 0, 0, 0),
        )
        fluid.particle_system.CreateSimulationOwnerRel().SetTargets([sim.cfg.physics_prim_path])
        count = len(fluid.init_particle_positions)
        assert count >= 100, f"{name}: undersampled fill ({count})"
        cups.append(dict(name=name, obj=obj, fluid=fluid, axis=axis, count=count, size=meta["physics"]["size"]))

    beans_at = np.array([0.5, 0.5, 0.0])
    cams = {
        "overview": add_camera("/World/cam_overview", (0.78, -0.62, 0.62), (0.0, 0.0, 0.05)),
        "top": add_camera("/World/cam_top", (0.42, 0.0, 0.92), (0.0, 0.0, 0.03)),
        "closeup": add_camera("/World/cam_closeup", (0.20, 0.06, 0.15), (0.0, 0.16, 0.07)),
        "bean": add_camera("/World/cam_bean", tuple(beans_at + [0.0, -0.05, 0.033]), tuple(beans_at), focal=60.0),
    }
    poses = [  # flat with the hilum towards the camera, on its side with the hilum up, flat and turned
        ((-0.0068, 0.0, rb.C), (0.0, 0.0, 0.0, 1.0)),
        ((0.0, 0.0015, rb.B), tuple(Rotation.from_euler("xz", [90, 15], degrees=True).as_quat()[[3, 0, 1, 2]])),
        ((0.0068, 0.0, rb.C), tuple(Rotation.from_euler("z", 130, degrees=True).as_quat()[[3, 0, 1, 2]])),
    ]
    for i, (p, q) in enumerate(poses):
        add_bean(stage, f"/World/single_beans/bean_{i}", beans_at + p, q)

    sim.reset()
    carb.settings.get_settings().set_bool("/physics/suppressReadback", False)
    assert sim.get_physics_context().is_gpu_dynamics_enabled(), "PBD requires GPU dynamics"
    for c in cups:
        c["obj"].initialize()
        c["fluid"].initialize()
    for cam in cams.values():
        cam.initialize()

    def snapshot(cam, name):
        for _ in range(40):
            sim.render()
        Image.fromarray(cams[cam].get_rgba()[:, :, :3]).save(out / f"{name}.png")

    for _ in range(round(args.seconds / DT)):
        sim.step(render=False)
    get_physx_interface().update_transformations(False, True)

    report = {}
    rng = np.random.default_rng(0)
    for c in cups:
        local = c["fluid"].get_particle_positions()[0]
        assert local.shape == (c["count"], 3) and np.isfinite(local).all(), "particle state broken"
        tf = np.array(UsdGeom.XformCache().GetLocalToWorldTransform(c["fluid"].point_instancer.GetPrim()))
        world = local @ tf[:3, :3] + tf[3, :3]
        rel = world - c["axis"]
        half = min(c["size"][:2]) / 2  # rim half-width; the larger extent includes the handle or tab
        inside = (np.abs(rel[:, :2]) <= half + 0.002).all(1) & (rel[:, 2] >= 0) & (rel[:, 2] <= c["size"][2])
        initial = np.array([[p[0], p[1], p[2]] for p in c["fluid"].init_particle_positions])
        report[c["name"]] = dict(
            count=c["count"],
            inside=float(inside.mean()),
            settle_drop_mm=float((initial[:, 2].mean() - local[:, 2].mean()) * 1000),
            top_mm=float(np.percentile(rel[:, 2], 98) * 1000),
        )
        c["world"] = world
    snapshot("overview", "spheres")

    for k, c in enumerate(cups):
        UsdGeom.Imageable(c["fluid"].point_instancer.GetPrim()).MakeInvisible()
        inst = UsdGeom.PointInstancer.Define(stage, f"/World/bean_vis/cup_{k}")
        proto = f"/World/bean_vis/cup_{k}/proto"
        UsdGeom.Xform.Define(stage, proto)
        rb.define_visual(stage, proto)
        inst.CreatePrototypesRel().AddTarget(proto)
        n = len(c["world"])
        q = Rotation.random(n, random_state=rng.integers(1 << 31)).as_quat()
        inst.CreateProtoIndicesAttr(Vt.IntArray([0] * n))
        inst.CreatePositionsAttr(Vt.Vec3fArray.FromNumpy(c["world"].astype(np.float32)))
        inst.CreateOrientationsAttr(Vt.QuathArray([Gf.Quath(*map(float, (w, x, y, z))) for x, y, z, w in q]))
        inst.CreateScalesAttr(Vt.Vec3fArray.FromNumpy(np.full((n, 3), BEAN_SCALE, np.float32)))
    snapshot("overview", "beans")
    snapshot("top", "top")
    snapshot("closeup", "closeup")
    snapshot("bean", "bean")

    report = dict(cups=report, seconds=args.seconds, bean_physics=RED_BEAN_PHYSICS)
    (out / "metrics.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report["cups"], indent=2), flush=True)
    return not all(r["inside"] >= 0.99 and r["settle_drop_mm"] > 0.5 for r in report["cups"].values())


try:
    failed = main()
except Exception:
    traceback.print_exc()
    failed = True
finally:
    get_physx_interface().overwrite_gpu_setting(GPU_OVERRIDE)
sys.stdout.flush()
sys.stderr.flush()
os._exit(1 if failed else 0)
