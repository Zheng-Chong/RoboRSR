"""Render each robot body alone at its default joint pose, for the embodiment cards of the Real2Sim platform.

Loads env/robot_manager/robot_config/<robot_name>.py (the config the env uses) on a floor and writes
<image-dir>/<robot_name>/preview/sim.png, the layout src/real2sim_platform reads (preview/ is not in git).

Usage (GPU server, from a run dir whose Assets/Robots/ holds the robot assets):
    PYTHONPATH=. python scripts/embodiments/render_embodiments.py g1d g1d_mobile x5 franka
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Render robot bodies at their default pose.")
parser.add_argument("robots", nargs="+", help="robot_name, as in env/robot_manager/robot_config/<robot_name>.py")
parser.add_argument("--image-dir", default="real2sim/embodiments")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True
app = AppLauncher(args).app

from importlib import import_module  # noqa: E402
import os  # noqa: E402
from pathlib import Path  # noqa: E402
import sys  # noqa: E402
import traceback  # noqa: E402

import carb  # noqa: E402
from isaaclab.assets import Articulation  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

DT = 1 / 60
SPACING = 4.0  # m between robots along Y, so each camera sees only its own robot
VIEW = np.array([0.8, -0.5, 0.45]) / np.linalg.norm([0.8, -0.5, 0.45])  # front-right, above; robots face +X


def look_at(eye, target):
    """(w, x, y, z) of a camera at `eye` looking at `target`, Isaac "world" camera axes (+X forward, +Z up)."""
    forward = (target - eye) / np.linalg.norm(target - eye)
    left = np.cross([0.0, 0.0, 1.0], forward)
    left /= np.linalg.norm(left)
    up = np.cross(forward, left)
    m = np.stack([forward, left, up], axis=1)  # columns: the camera's axes in the world
    w = np.sqrt(max(1e-12, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    return np.array([w, (m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w)])


def main():
    render = sim_utils.RenderCfg(enable_reflections=True, enable_global_illumination=True, rendering_mode="quality")
    carb.settings.get_settings().set_float("/rtx/translucency/worldEps", 0.0)
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device=args.device, render=render))
    row = (len(args.robots) - 1) * SPACING
    floor = sim_utils.CuboidCfg(  # static box instead of GroundPlaneCfg, which fetches a USD from Nucleus
        size=(60.0, row + 60.0, 0.02),  # past the horizon of every camera
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.55, 0.56, 0.58)),
    )
    floor.func("/World/floor", floor, translation=(0.0, row / 2, -0.01))
    dome = sim_utils.DomeLightCfg(intensity=600.0)
    dome.func("/World/dome", dome)
    sun = sim_utils.DistantLightCfg(intensity=2500.0, angle=5.0)
    sun.func("/World/sun", sun, orientation=(0.9397, 0.2418, 0.2418, 0.0))  # 40 deg off vertical, for shadows

    robots, cameras = [], []
    for k, name in enumerate(args.robots):
        cfg = import_module(f"env.robot_manager.robot_config.{name}").get_robot_config()
        pose = cfg.init_state.replace(pos=(0.0, k * SPACING, 0.0), rot=(1.0, 0.0, 0.0, 0.0))
        robot = Articulation(cfg.replace(prim_path=f"/World/robot_{k}", init_state=pose))
        # stand the robot on the floor: its lowest mesh point (a fixed base, so independent of the arm pose) at z=0;
        # the init state is applied at sim.reset()
        prim = sim.stage.GetPrimAtPath(f"/World/robot_{k}")
        bottom = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"]).ComputeWorldBound(prim)
        robot.cfg.init_state = pose.replace(pos=(0.0, k * SPACING, -bottom.ComputeAlignedRange().GetMin()[2]))
        robots.append(robot)
        cam = Camera(prim_path=f"/World/camera_{k}", resolution=(960, 720))
        set_camera_view((3.0, k * SPACING - 2.0, 2.0), (0.0, k * SPACING, 0.5), camera_prim_path=cam.prim_path)
        cameras.append((cam, Path(args.image_dir) / name / "preview" / "sim.png"))

    sim.reset()
    for robot in robots:
        robot.write_joint_state_to_sim(robot.data.default_joint_pos, robot.data.default_joint_vel)
    for _ in range(60):  # hold the default pose while the scene settles
        for robot in robots:
            robot.set_joint_position_target(robot.data.default_joint_pos)
            robot.write_data_to_sim()
        sim.step(render=False)
        for robot in robots:
            robot.update(DT)
    for robot, (cam, _), name in zip(robots, cameras, args.robots):
        # frame the settled pose: link origins plus a margin for the meshes around them
        links = robot.data.body_pos_w[0].cpu().numpy()
        lo, hi = links.min(axis=0) - 0.12, links.max(axis=0) + 0.12
        lo[2] = max(lo[2], 0.0)
        centre = (lo + hi) / 2
        # USD default lens sees 18 deg vertically at 4:3; the bounding sphere fills ~85% of the height
        eye = centre + VIEW * 3.5 * float(np.linalg.norm(hi - lo))
        cam.initialize()
        cam.set_world_pose(eye, look_at(eye, centre), camera_axes="world")
        print(f"{name}: links span {np.round(hi - lo, 3).tolist()} m")
    for _ in range(40):  # let RTX accumulate
        sim.render()
    for cam, path in cameras:
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(cam.get_rgba()[:, :, :3]).save(path)
        print(f"image: {path}")


try:
    main()
    failed = False
except Exception:
    traceback.print_exc()
    failed = True
sys.stdout.flush()
sys.stderr.flush()
os._exit(1 if failed else 0)  # app.close() hangs once a camera render product exists, and Kit eats exit codes
