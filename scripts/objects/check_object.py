"""Drop RoboDojo rigid objects on a floor in Isaac Sim and check that they settle upright.

Each object dir (object.usdz + metadata.json) is loaded through env RigidObject, the loader tasks use.
Fails if an object tips over, sinks into / floats above the floor, or is still moving after --seconds.

Usage:
    python scripts/objects/check_object.py DIR [DIR ...] [--image out.png]
    e.g. python scripts/objects/check_object.py Assets/Object/RoboDojo/Rigid/pitcher/0000{0,1,2}
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Drop rigid objects on a floor and check they settle upright.")
parser.add_argument("dirs", nargs="+", help="object dirs containing object.usdz and metadata.json")
parser.add_argument("--seconds", type=float, default=3.0)
parser.add_argument("--image", help="also render the settled objects to this PNG")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = args.image is not None
app = AppLauncher(args).app

import json  # noqa: E402
import os  # noqa: E402
from pathlib import Path  # noqa: E402
import sys  # noqa: E402
import traceback  # noqa: E402

import carb  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

from env.scene_manager.objects.rigid import RigidObject  # noqa: E402

DT = 0.004  # env_cfg/sim/sim_config.yml
SPACING = 0.2  # m between objects along Y


def as_np(x):
    return np.asarray(x.cpu() if hasattr(x, "cpu") else x, dtype=float).reshape(-1)


def main():
    """Returns True if any object failed."""
    # Match RoboDojo's render settings: IsaacLab defaults hide transmissive glass.
    render = sim_utils.RenderCfg(
        enable_translucency=True,
        enable_reflections=True,
        enable_global_illumination=True,
        rendering_mode="quality",
    )
    carb.settings.get_settings().set_float("/rtx/translucency/worldEps", 0.0)
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device=args.device, render=render))
    floor = sim_utils.CuboidCfg(  # static box instead of GroundPlaneCfg, which fetches a USD from Nucleus
        size=(2.0, 2.0, 0.02),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.36, 0.38)),
    )
    floor.func("/World/floor", floor, translation=(0.0, 0.0, -0.01))
    dome = sim_utils.DomeLightCfg(intensity=300.0)  # brighter washes out glass
    dome.func("/World/dome", dome)
    sun = sim_utils.DistantLightCfg(intensity=2500.0, angle=5.0)
    sun.func("/World/sun", sun, orientation=(0.9397, 0.2418, 0.2418, 0.0))  # 40 deg off vertical, for shadows

    objs = []
    for k, d in enumerate(map(Path, args.dirs)):
        meta = json.loads((d / "metadata.json").read_text())
        half_h = meta["physics"]["size"][2] / 2  # asset origin is the bbox centre
        category, idx = d.parent.name, int(d.name)
        obj = RigidObject(
            prim_path=f"/World/rigid/{category}/{category}_{idx}_{k}",
            usd_path=str(d / "object.usdz"),
            inst_config={"physics": {"mass": meta["physics"]["mass"]}},
            env_origin=[0.0, 0.0, 0.0],
            default_pos=(0.0, k * SPACING, half_h + 0.01),
            default_ori=(1.0, 0.0, 0.0, 0.0),
            scale=(1.0, 1.0, 1.0),
        )
        objs.append((d, obj, half_h))

    camera = None
    if args.image:
        camera = Camera(prim_path="/World/check_camera", resolution=(1280, 720))
        mid_y = (len(objs) - 1) * SPACING / 2
        # pose before sim.reset(): later USD xform edits do not reach the renderer with Fabric on
        set_camera_view((1.1, mid_y - 0.9, 0.5), (0.0, mid_y, 0.11), camera_prim_path=camera.prim_path)
    sim.reset()
    for _, obj, _ in objs:
        obj.initialize()
    for _ in range(round(args.seconds / DT)):
        sim.step(render=False)

    failed = False
    for d, obj, half_h in objs:
        pos, quat = map(as_np, obj.get_world_pose())
        _, x, y, _ = quat
        up = 1 - 2 * (x * x + y * y)  # world-Z component of the object's Z axis
        speed = np.linalg.norm(as_np(obj.get_linear_velocity()))
        ok = up > np.cos(np.radians(5)) and abs(pos[2] - half_h) < 0.005 and speed < 0.01
        failed |= not ok
        tilt = np.degrees(np.arccos(min(up, 1.0)))
        print(
            f"{'OK  ' if ok else 'FAIL'} {d}: z={pos[2]:.4f} (expect {half_h:.4f}) "
            f"tilt={tilt:.2f} deg speed={speed:.4f} m/s"
        )

    if camera is not None:
        camera.initialize()
        for _ in range(30):  # let RTX accumulate
            sim.render()
        Image.fromarray(camera.get_rgba()[:, :, :3]).save(args.image)
        print(f"image: {args.image}")
    return failed


try:
    failed = main()
except Exception:
    traceback.print_exc()
    failed = True
sys.stdout.flush()
sys.stderr.flush()
os._exit(1 if failed else 0)  # app.close() hangs once a camera render product exists, and Kit eats exit codes
