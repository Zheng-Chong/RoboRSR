"""Isaac Lab smoke check: production G1-D import, driven gripper poses and four HD views."""

import argparse
from pathlib import Path

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--asset-dir", type=Path, required=True)
parser.add_argument("--runtime-file", type=Path, required=True)
parser.add_argument("--out", type=Path, required=True)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.asset_dir = args.asset_dir.resolve()
args.out = args.out.resolve()
if args.out.exists():
    parser.error("--out must be a new private directory")
args.headless = True
args.enable_cameras = True
app = AppLauncher(args).app

from hashlib import sha256
from importlib import import_module
import json
import os
import sys
import traceback

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import Articulation, ArticulationCfg
import isaaclab.sim as sim_utils
from isaacsim.sensors.camera import Camera
import numpy as np
from PIL import Image
from pxr import Usd, UsdGeom, UsdPhysics
from scipy.spatial.transform import Rotation
import torch


def main():
    out = args.out
    out.mkdir(parents=True, exist_ok=False)
    before = {str(p): sha256(p.read_bytes()).hexdigest() for p in args.asset_dir.rglob("*") if p.is_file()}
    os.environ["ROBORSR_G1D_ASSET_DIR"] = str(args.asset_dir)
    runtime = import_module("env.robot_manager.robot_config.g1d")
    if Path(runtime.__file__).resolve() != args.runtime_file.resolve():
        raise ValueError("Use the delivery's runtime/launch.py so the private config is selected")
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(dt=1 / 300, device=args.device, log_dir=str(out / "logs"))
    )
    full_cfg = runtime.get_robot_config()
    full_cfg.prim_path = "/World/G1D"
    full_cfg.spawn.usd_dir = str(out / "full_robot_usd")
    full_cfg.spawn.force_usd_conversion = True
    full = Articulation(full_cfg)
    robot = Articulation(
        ArticulationCfg(
            prim_path="/World/Gripper",
            spawn=sim_utils.UrdfFileCfg(
                asset_path=str(args.asset_dir / "left_gripper.urdf"),
                usd_dir=str(out / "gripper_usd"),
                force_usd_conversion=True,
                fix_base=True,
                merge_fixed_joints=False,
                joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                    gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)
                ),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(disable_gravity=True),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=True, solver_position_iteration_count=8, solver_velocity_iteration_count=0
                ),
            ),
            init_state=ArticulationCfg.InitialStateCfg(pos=(3, 0, 0.2)),
            actuators={
                "fingers": ImplicitActuatorCfg(
                    joint_names_expr=[".*finger_joint.*"], stiffness=1e4, damping=100.0, effort_limit_sim=20.0
                )
            },
        )
    )
    prim = sim.stage.GetPrimAtPath("/World/Gripper")
    black = sim_utils.PreviewSurfaceCfg(diffuse_color=(0.02, 0.02, 0.02), roughness=0.8)
    black.func("/World/Gripper/Looks/Black", black)
    for link in prim.GetChildren():
        visuals = link.GetChild("visuals")
        if visuals:
            visuals.SetInstanceable(False)
            sim_utils.bind_visual_material(str(visuals.GetPath()), "/World/Gripper/Looks/Black")
    contract = json.loads((args.asset_dir / "physics_contract.json").read_text())
    pairs = [(a, b) for a, others in contract["filtered_pairs"].items() for b in others]
    filtered = []
    for a, b in pairs + contract["additional_internal_slider_carrier_pairs"]:
        pa, pb = prim.GetChild(a), prim.GetChild(b)
        if pa and pb:
            UsdPhysics.FilteredPairsAPI.Apply(pa).CreateFilteredPairsRel().AddTarget(pb.GetPath())
            filtered.append([a, b])
    dome = sim_utils.DomeLightCfg(intensity=700)
    dome.func("/World/Fill", dome)
    for name, intensity, quat in (("Key", 1800, (0.92388, 0.38268, 0, 0)), ("Rim", 1300, (0.5, -0.5, 0.5, 0.5))):
        light = sim_utils.DistantLightCfg(intensity=intensity, angle=10)
        light.func(f"/World/{name}", light, orientation=quat)
    camera = Camera(prim_path="/World/Review", resolution=(2560, 1920))
    camera.set_clipping_range(0.001, 1.0)  # exclude the separate full robot from the front-view background
    sim.reset()
    camera.initialize()
    robot.write_joint_state_to_sim(
        torch.full_like(robot.data.default_joint_pos, -0.02), torch.zeros_like(robot.data.default_joint_pos)
    )
    native_world = Rotation.from_euler("z", -90, degrees=True).as_matrix()

    def view(eye, target=(0, 0.075, 0.025), up=(0, 0, 1), span=0.21):
        eye = np.array(eye)
        forward = np.array(target) - eye
        forward /= np.linalg.norm(forward)
        right = np.cross(forward, up)
        right /= np.linalg.norm(right)
        rotation = native_world @ np.array([right, np.cross(forward, right), forward]).T @ np.diag([1, -1, -1])
        quat = Rotation.from_matrix(rotation).as_quat()
        camera.set_world_pose(
            position=native_world @ eye + [3, 0, 0.2], orientation=quat[[3, 0, 1, 2]], camera_axes="usd"
        )
        usd = UsdGeom.Camera(sim.stage.GetPrimAtPath(camera.prim_path))
        usd.GetProjectionAttr().Set(UsdGeom.Tokens.orthographic)
        usd.GetHorizontalApertureAttr().Set(span * 10)
        usd.GetVerticalApertureAttr().Set(span * 10 * 3 / 4)

    def capture(name):
        for _ in range(24):
            sim.render()
        rgb = np.asarray(camera.get_rgba())[:, :, :3]
        if np.issubdtype(rgb.dtype, np.floating) and rgb.max() <= 1:
            rgb *= 255
        if not np.isfinite(rgb).all() or rgb.std() < 1:
            raise ValueError("Invalid render")
        Image.fromarray(np.clip(rgb, 0, 255).astype(np.uint8)).save(out / f"{name}.png")

    samples = []
    for label, target in (("open", -0.02), ("middle", 0.0), ("closed", 0.0245)):
        start = robot.data.joint_pos.clone()
        for step in range(240):
            t = min((step + 1) / 180, 1.0)
            desired = start + (target - start) * (t * t * (3 - 2 * t))
            robot.set_joint_position_target(desired)
            robot.write_data_to_sim()
            full.set_joint_position_target(full.data.default_joint_pos)
            full.write_data_to_sim()
            sim.step(render=False)
            robot.update(sim.get_physics_dt())
            full.update(sim.get_physics_dt())
        actual = robot.data.joint_pos.cpu().numpy()
        error = float(np.max(np.abs(actual - target)))
        if not np.isfinite(actual).all() or error > 0.001:
            raise ValueError(f"Failed driven {label}: {actual}, error={error}")
        samples.append({"label": label, "target_m": target, "actual_m": actual.tolist(), "max_error_m": error})
        view((0.36, 0.42, 0.30), span=0.24)  # include the entire camera in the external review image
        capture(label)
        if label == "open":
            for name, eye, up in (
                ("side", (-0.6, 0.075, 0.025), (0, 0, 1)),
                ("front", (0, 0.6, 0.025), (0, 0, 1)),
                ("top", (0, 0.075, 0.6), (1, 0, 0)),
                ("oblique", (0.36, 0.42, 0.30), (0, 0, 1)),
            ):
                view(eye, up=up, span=0.24 if name == "oblique" else 0.21)
                capture(name)
    assert torch.isfinite(full.data.joint_pos).all()
    assert "d435_link" in full.body_names
    assert all("Wheel" not in n for n in full.joint_names)
    assert all(sha256(Path(p).read_bytes()).hexdigest() == h for p, h in before.items())
    colliders = [
        str(p.GetPath())
        for p in sim.stage.Traverse(Usd.TraverseInstanceProxies())
        if p.HasAPI(UsdPhysics.CollisionAPI) and str(p.GetPath()).startswith("/World/Gripper/")
    ]
    assert len(colliders) >= 39
    grid = Image.new("RGB", (2560 * 3, 1920), "white")
    for i, label in enumerate(("open", "middle", "closed")):
        grid.paste(Image.open(out / f"{label}.png"), (2560 * i, 0))
    grid.save(out / "open_middle_closed.png")
    report = {
        "passed": True,
        "approximate_geometry_not_calibrated": True,
        "production_full_robot_import_passed": True,
        "full_robot_joint_count": len(full.joint_names),
        "head_frame_and_locked_wheels_verified": True,
        "self_collisions_enabled": True,
        "existing_filter_contract_applied": True,
        "additional_filters_added": False,
        "gripper_colliders": len(colliders),
        "drive_samples": samples,
        "input_asset_files_unchanged": True,
        "calibration_files_modified": [],
        "scope": "Empty-jaw driven smoke check and full-robot import; not a grasp or precision calibration test",
    }
    (out / "result.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2), flush=True)


try:
    main()
    code = 0
except Exception:
    traceback.print_exc()
    code = 1
sys.stdout.flush()
sys.stderr.flush()
os._exit(code)
