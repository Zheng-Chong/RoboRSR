"""Render the Real2Sim platform's preview image of rigid objects without simulating them.

Same camera, lights and output layout as check_object.py --image-dir (<image-dir>/<type>/<category>/<index>/preview/
sim.png), but every object is kinematic and placed standing on the floor at its half-height: no settling, no
check.json. For assets whose physics is not validated yet.

    PYTHONPATH=. python scripts/objects/render_preview.py DIR [DIR ...] --image-dir real2sim/objects
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Render platform preview images of rigid objects, no physics.")
parser.add_argument("dirs", nargs="+", help="object dirs containing object.usdz and metadata.json")
parser.add_argument("--image-dir", required=True, help="writes <dir>/<type>/<category>/<index>/preview/sim.png")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True
app = AppLauncher(args).app

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
from PIL import Image  # noqa: E402
from pxr import UsdPhysics  # noqa: E402

from env.scene_manager.objects.rigid import RigidObject  # noqa: E402

SOLO_SPACING = 0.6  # check_object.py: neighbours stay outside each object's own view
SOLO_VIEW = np.array([0.82, -0.25, 0.5]) / np.linalg.norm([0.82, -0.25, 0.5])


def main():
    render = sim_utils.RenderCfg(
        enable_translucency=True,
        enable_reflections=True,
        enable_global_illumination=True,
        rendering_mode="quality",
    )
    carb.settings.get_settings().set_float("/rtx/translucency/worldEps", 0.0)
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=0.004, device=args.device, render=render))
    stage = get_current_stage()
    row = (len(args.dirs) - 1) * SOLO_SPACING
    floor = sim_utils.CuboidCfg(
        size=(2.0, row + 2.0, 0.02),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.36, 0.38)),
    )
    floor.func("/World/floor", floor, translation=(0.0, row / 2, -0.01))
    dome = sim_utils.DomeLightCfg(intensity=300.0)
    dome.func("/World/dome", dome)
    sun = sim_utils.DistantLightCfg(intensity=2500.0, angle=5.0)
    sun.func("/World/sun", sun, orientation=(0.9397, 0.2418, 0.2418, 0.0))

    objs, cams = [], []
    for k, d in enumerate(map(Path, args.dirs)):
        meta = json.loads((d / "metadata.json").read_text())
        size = meta["physics"].get("size") or meta["geometry"]["aligned_bbox"]["extents"]
        category = d.parent.name
        path = f"/World/rigid/{category}/{category}_{int(d.name)}_{k}"
        objs.append(
            RigidObject(
                prim_path=path,
                usd_path=str(d / "object.usdz"),
                inst_config={"physics": {"mass": meta["physics"]["mass"]}},
                env_origin=[0.0, 0.0, 0.0],
                default_pos=(0.0, k * SOLO_SPACING, size[2] / 2),
                default_ori=(1.0, 0.0, 0.0, 0.0),
                scale=(1.0, 1.0, 1.0),
            )
        )
        UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).CreateKinematicEnabledAttr().Set(True)
        cam = Camera(prim_path=f"/World/solo_camera_{k}", resolution=(640, 480))
        centre = np.array([0.0, k * SOLO_SPACING, size[2] / 2])
        eye = centre + SOLO_VIEW * max(3.7 * float(np.linalg.norm(size)), 0.03)
        set_camera_view(tuple(eye), tuple(centre), camera_prim_path=cam.prim_path)
        cams.append((cam, Path(args.image_dir) / d.parent.parent.name / category / d.name / "preview" / "sim.png"))

    sim.reset()
    for obj in objs:
        obj.initialize()
    for cam, _ in cams:
        cam.initialize()
        cam.set_clipping_range(0.001, 100.0)
    for _ in range(30):  # let RTX accumulate
        sim.render()
    for cam, path in cams:
        path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(cam.get_rgba()[:, :, :3]).save(path)
        print(f"image: {path}", flush=True)


try:
    main()
    failed = False
except Exception:
    traceback.print_exc()
    failed = True
sys.stdout.flush()
sys.stderr.flush()
os._exit(1 if failed else 0)  # app.close() hangs once a camera render product exists
