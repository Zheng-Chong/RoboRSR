"""Check pitcher fluid containment and pouring with RoboDojo's actual object loaders.

An open pitcher and an otherwise identical pitcher with a blocked pour opening follow
the same prescribed motion. This isolates vessel/particle collisions from robot control.
Uses the benchmark's wuliangye fluid asset and particle/material parameters unchanged.

PYTHONPATH=. python scripts/objects/check_pitcher_fluid.py --device cpu
The CPU state pipeline allows particle USD readback; PBD physics still requires a GPU.
Writes metrics.json and upright/pouring/final.png under --out.
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--pitcher-dir", default="Assets/Object/RoboDojo/Rigid/pitcher/00002")
parser.add_argument("--fluid-dir", default="Assets/Object/RoboDojo/Fluid/wuliangye/00000")
parser.add_argument("--out", default="output/pitcher/fluid")
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(device="cpu")
args = parser.parse_args()
if args.device != "cpu":
    parser.error("Use --device cpu for particle USD readback; PBD physics still runs on the GPU.")
args.headless = True
args.enable_cameras = True
app = AppLauncher(args).app

import copy  # noqa: E402
import json  # noqa: E402
import os  # noqa: E402
from pathlib import Path  # noqa: E402
import sys  # noqa: E402
import traceback  # noqa: E402

import carb  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaacsim.core.utils.stage import get_current_stage  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
import numpy as np  # noqa: E402
from omni.physx import get_physx_interface  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, PhysxSchema, UsdGeom, UsdPhysics  # noqa: E402
import torch  # noqa: E402

from env.scene_manager.objects.fluid import FluidObject  # noqa: E402
from env.scene_manager.objects.rigid import RigidObject  # noqa: E402
from scripts.objects import make_pitcher as shape  # noqa: E402

DT = 0.004
SETTLE, TILT, HOLD = 2.0, 4.0, 3.0
ANGLE = np.radians(130)
BUFFER = 0.004  # particle radius/contact offsets near the outer envelope
GPU_OVERRIDE = get_physx_interface().get_overwrite_gpu_setting()


def rotation(angle):
    c, s = np.cos(angle), np.sin(angle)
    return np.array([[c, 0, -s], [0, 1, 0], [s, 0, c]])


def contained(points):
    """Outer glass/collar envelope in the generator frame, including contact tolerance."""
    z = points[:, 2]
    t = np.clip((z - shape.Z_SH0) / shape.SHOULDER, 0, 1)
    s = t * t * (3 - 2 * t)
    r = shape.A + (shape.R_NECK - shape.A) * s
    theta = np.arctan2(points[:, 1], points[:, 0])
    square_rho = (abs(np.cos(theta)) ** shape.SQ + abs(np.sin(theta)) ** shape.SQ) ** (-1 / shape.SQ)
    radius = r * ((1 - s) * square_rho + s)
    radius = np.where(z >= shape.Z_C0, shape.R_C, radius)
    return (np.linalg.norm(points[:, :2], axis=1) <= radius + BUFFER) & (z >= -BUFFER) & (z <= shape.Z_TOP + BUFFER)


def world_particles(case):
    points = case["fluid"].get_particle_positions()[0]
    assert points.shape == (case["count"], 3), "particle count changed"
    assert np.isfinite(points).all(), "nonfinite particle positions"
    tf = case["fluid_tf"]
    return points @ tf[:3, :3] + tf[3, :3]


def main():
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    pitcher_dir, fluid_dir = Path(args.pitcher_dir).resolve(), Path(args.fluid_dir).resolve()
    meta = json.loads((pitcher_dir / "metadata.json").read_text())
    fluid_cfg = json.loads((fluid_dir / "metadata.json").read_text())
    # Resolve the same asset-root placeholder as RoboDojo's metadata loader.
    asset_root = fluid_dir.parents[4]
    fluid_cfg["visual"]["visual_mdl_path"] = fluid_cfg["visual"]["visual_mdl_path"].replace(
        "$Robodojo_ASSETS", str(asset_root)
    )
    visual, _, _ = shape.build()
    points = np.concatenate([p for p, _, _ in visual])
    shift = -(points.min(0) + points.max(0)) / 2
    assert np.allclose(points.max(0) - points.min(0), meta["physics"]["size"], atol=1e-4)
    assert shape.SPOUT_OPEN > 0
    assert contained(np.array([[0, 0, 0.05]])).item()
    assert not contained(np.array([[0.15, 0, 0.05]])).item()
    assert np.allclose(rotation(ANGLE).T @ rotation(ANGLE), np.eye(3))

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
    # Isaac Sim 5.1's GPU tensor pipeline does not publish PBD positions to USD.
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

    cases = []
    for index, name in enumerate(("open", "sealed")):
        centre = np.array([0, (index - 0.5) * 0.4, 0.40])
        root = f"/World/envs/env_{index}"
        path = f"{root}/Rigid/pitcher/pitcher_2_0"
        obj = RigidObject(
            prim_path=path,
            usd_path=str(pitcher_dir / "object.usdz"),
            inst_config={"physics": {"mass": meta["physics"]["mass"]}},
            env_origin=[0, 0, 0],
            default_pos=centre,
            default_ori=(1, 0, 0, 0),
            scale=(1, 1, 1),
        )
        UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).CreateKinematicEnabledAttr().Set(True)
        if name == "sealed":
            # Only seal the lid's pour opening; leave every original collider unchanged.
            cap = sim_utils.CuboidCfg(
                size=(0.035, 0.09, shape.T_CAP),
                collision_props=sim_utils.CollisionPropertiesCfg(),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.6, 0.12, 0.08)),
            )
            cap.func(
                f"{path}/collision/seal",
                cap,
                translation=tuple(np.array([-shape.R_C, 0, shape.Z_LID + shape.T_CAP / 2]) + shift),
            )
        fluid = FluidObject(
            prim_path=f"{root}/Fluid/wuliangye/wuliangye_0_0",
            usd_path=str(fluid_dir / "object.usdz"),
            inst_config=copy.deepcopy(fluid_cfg),
            env_origin=torch.zeros(3),
            default_pos=centre + shift + np.array([0, 0, shape.T_BOTTOM + 0.045]),
            default_ori=(1, 0, 0, 0),
        )
        fluid.particle_system.CreateSimulationOwnerRel().SetTargets([sim.cfg.physics_prim_path])
        count = len(fluid.init_particle_positions)
        assert count >= 100, "empty/undersampled fluid"
        tf = np.array(UsdGeom.XformCache().GetLocalToWorldTransform(fluid.point_instancer.GetPrim()))
        orient_attr = stage.GetPrimAtPath(path).GetAttribute("xformOp:orient")
        assert orient_attr.IsValid()
        case = dict(name=name, obj=obj, orient=orient_attr, centre=centre, fluid=fluid, fluid_tf=tf, count=count)
        initial = world_particles(case)
        local = initial - centre - shift
        assert contained(local).all(), "initial fluid is outside pitcher"
        case.update(initial=initial, previous=local, crossed=np.zeros(count, bool), opening=np.zeros(count, bool))
        cases.append(case)

    camera = Camera(prim_path="/World/check_camera", resolution=(1280, 720))
    set_camera_view((-1.6, -1.5, 1.0), (-0.05, 0, 0.27), camera_prim_path=camera.prim_path)
    sim.reset()
    settings = carb.settings.get_settings()
    settings.set_bool("/physics/suppressReadback", False)
    assert sim.get_physics_context().is_gpu_dynamics_enabled(), "PBD requires GPU dynamics"
    for case in cases:
        case["obj"].initialize()
        case["fluid"].initialize()
    camera.initialize()

    def snapshot(name):
        for _ in range(30):
            sim.render()
        Image.fromarray(camera.get_rgba()[:, :, :3]).save(out / f"{name}.png")

    report = {"dt": DT, "settle_s": SETTLE, "tilt_s": TILT, "hold_s": HOLD, "angle_deg": 130, "cases": {}}
    for step in range(round((SETTLE + TILT + HOLD) / DT)):
        t = (step + 1) * DT
        fraction = np.clip((t - SETTLE) / TILT, 0, 1)
        angle = ANGLE * (fraction * fraction * (3 - 2 * fraction))
        rot = rotation(angle)
        quat = Gf.Quatf(float(np.cos(angle / 2)), Gf.Vec3f(0, float(-np.sin(angle / 2)), 0))
        for case in cases:
            case["orient"].Set(quat)
        sim.step(render=False)
        # FluidObject reads USD points; physics-only steps otherwise leave these stale.
        get_physx_interface().update_transformations(False, True)
        for case in cases:
            world = world_particles(case)
            local = (world - case["centre"]) @ rot - shift
            # Track first crossing of the outer lid plane; positions are interpolated across the step.
            plane = shape.Z_TOP + BUFFER
            prev = case["previous"]
            crossed = (~case["crossed"]) & (prev[:, 2] <= plane) & (local[:, 2] > plane)
            if crossed.any():
                a, b = prev[crossed], local[crossed]
                u = (plane - a[:, 2]) / (b[:, 2] - a[:, 2])
                hit = a + u[:, None] * (b - a)
                case["opening"][crossed] = hit[:, 0] <= -shape.R_CI + shape.SPOUT_OPEN + BUFFER
                case["crossed"] |= crossed
            case["previous"] = local
            if step + 1 == round(SETTLE / DT):
                retained = float(contained(local).mean())
                drop = float(case["initial"][:, 2].mean() - world[:, 2].mean())
                report["cases"][case["name"]] = dict(
                    count=case["count"], upright_retained=retained, settling_drop_m=drop
                )
                print(case["name"], "upright", report["cases"][case["name"]], flush=True)
        if step + 1 == round(SETTLE / DT):
            snapshot("upright")
        if step + 1 == round((SETTLE + TILT * 0.80) / DT):
            snapshot("pouring")
    snapshot("final")
    np.savez_compressed(out / "particles.npz", **{c["name"]: world_particles(c) for c in cases})
    for case in cases:
        data = report["cases"][case["name"]]
        data.update(
            final_retained=float(contained(case["previous"]).mean()),
            crossed_lid=int(case["crossed"].sum()),
            crossed_opening=int(case["opening"].sum()),
            ground_fraction=float((world_particles(case)[:, 2] < 0.025).mean()),
        )
    opened, sealed = report["cases"]["open"], report["cases"]["sealed"]
    checks = {
        "upright_containment": all(c["upright_retained"] >= 0.99 for c in report["cases"].values()),
        "particles_simulated": all(c["settling_drop_m"] > 0.003 for c in report["cases"].values()),
        "sealed_containment": sealed["final_retained"] >= 0.99,
        "open_pours": opened["final_retained"] <= 0.50,
        "exit_through_opening": opened["crossed_opening"] >= opened["count"] * 0.50,
    }
    report.update(
        checks=checks,
        passed=all(checks.values()),
        state_pipeline="cpu",
        gpu_dynamics=True,
        fluid_physics=fluid_cfg["physics"],
    )
    (out / "metrics.json").write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)
    return not report["passed"]


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
