"""Render rigid objects under three lighting presets (the platform's 物理检验「光照渲染」, reviewed by eye).

Objects stand in a row on a grey floor in front of a striped backdrop, so see-through parts show their refraction.
Presets: daylight (bright dome + sun), warm_indoor (dim warm dome + one overhead lamp), backlit (dim cool dome + low sun
from behind the objects). Writes <out>/<preset>.png (whole row) and <preset>_closeup.png (first object). Look for
light leaking through closed surfaces, black faces (inverted normals) and wrong refraction.

    PYTHONPATH=. python scripts/objects/render_lights.py DIR [DIR ...] --out output/render_lights
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Render rigid objects under three lighting presets.")
parser.add_argument("dirs", nargs="+", help="object dirs containing object.usdz and metadata.json")
parser.add_argument("--out", default="output/render_lights")
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
from pxr import Gf, UsdGeom, UsdPhysics  # noqa: E402

from env.scene_manager.objects.rigid import RigidObject  # noqa: E402

SPACING = 0.15  # m between objects along Y
STRIPES = [(0.75, 0.12, 0.10), (0.95, 0.85, 0.15), (0.10, 0.45, 0.20), (0.10, 0.25, 0.70), (0.9, 0.9, 0.9)]
PRESETS = {  # light prim -> (intensity, colour); lights not listed are off
    "daylight": {"dome": (600.0, (1.0, 1.0, 1.0)), "sun": (3000.0, (1.0, 0.98, 0.95))},
    "warm_indoor": {"dome": (60.0, (1.0, 0.85, 0.7)), "lamp": (60000.0, (1.0, 0.78, 0.55))},
    "backlit": {"dome": (40.0, (0.8, 0.9, 1.0)), "back": (3500.0, (1.0, 1.0, 1.0))},
}


def main():
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    carb.settings.get_settings().set_float("/rtx/translucency/worldEps", 0.0)
    sim = sim_utils.SimulationContext(
        sim_utils.SimulationCfg(
            dt=0.004,
            device=args.device,
            use_fabric=False,  # light edits after reset must reach the renderer
            render=sim_utils.RenderCfg(
                enable_translucency=True,
                enable_reflections=True,
                enable_global_illumination=True,
                rendering_mode="quality",
            ),
        )
    )
    stage = get_current_stage()
    row = (len(args.dirs) - 1) * SPACING
    floor = sim_utils.CuboidCfg(
        size=(2.0, row + 2.0, 0.02),
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.45, 0.45, 0.47)),
    )
    floor.func("/World/floor", floor, translation=(0.0, row / 2, -0.01))
    width = (row + 0.6) / len(STRIPES)
    for i, rgb in enumerate(STRIPES):  # backdrop behind the row (-X), vertical stripes along Y
        stripe = sim_utils.CuboidCfg(
            size=(0.01, width, 0.5), visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=rgb)
        )
        stripe.func(f"/World/backdrop/s{i}", stripe, translation=(-0.25, -0.3 + (i + 0.5) * width, 0.25))

    lights = {
        "dome": sim_utils.DomeLightCfg(intensity=1.0),
        "sun": sim_utils.DistantLightCfg(intensity=1.0, angle=5.0),
        "lamp": sim_utils.SphereLightCfg(intensity=1.0, radius=0.05),
        "back": sim_utils.DistantLightCfg(intensity=1.0, angle=3.0),
    }
    poses = {
        "sun": ((0.0, 0.0, 0.0), (0.9397, 0.2418, 0.2418, 0.0)),
        "lamp": ((0.15, row / 2, 0.9), (1.0, 0.0, 0.0, 0.0)),
        "back": ((0.0, 0.0, 0.0), (0.7934, 0.0, -0.6088, 0.0)),  # -75 deg about Y: shines toward +X, 15 deg down
    }
    for name, cfg in lights.items():
        pos, ori = poses.get(name, ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)))
        cfg.func(f"/World/lights/{name}", cfg, translation=pos, orientation=ori)

    objs, top = [], 0.0
    for k, d in enumerate(map(Path, args.dirs)):
        meta = json.loads((d / "metadata.json").read_text())
        size = meta["physics"].get("size") or meta["geometry"]["aligned_bbox"]["extents"]
        top = max(top, size[2])
        path = f"/World/rigid/{d.parent.name}_{int(d.name)}_{k}"
        objs.append(
            RigidObject(
                prim_path=path,
                usd_path=str(d / "object.usdz"),
                inst_config={"physics": {"mass": meta["physics"]["mass"]}},
                env_origin=[0.0, 0.0, 0.0],
                default_pos=(0.0, k * SPACING, size[2] / 2),
                default_ori=(1.0, 0.0, 0.0, 0.0),
                scale=(1.0, 1.0, 1.0),
            )
        )
        UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(path)).CreateKinematicEnabledAttr().Set(True)
        if k == 0:
            first = size

    cams = {
        "": Camera(prim_path="/World/cam_row", resolution=(1600, 900)),
        "_closeup": Camera(prim_path="/World/cam_close", resolution=(900, 900)),
    }
    dist = 2.5 * (row + 0.3)  # default 50 mm lens sees ~24 deg across: the whole row plus a margin
    set_camera_view((dist, row / 2, 0.15 + dist * 0.12), (0.0, row / 2, top * 0.45), camera_prim_path="/World/cam_row")
    d0 = 2.6 * float(np.linalg.norm(first))
    set_camera_view(
        (d0 * 0.9, -d0 * 0.25, first[2] * 0.75), (0.0, 0.0, first[2] * 0.5), camera_prim_path="/World/cam_close"
    )
    for cam in cams.values():
        UsdGeom.Camera(stage.GetPrimAtPath(cam.prim_path)).GetClippingRangeAttr().Set(Gf.Vec2f(0.005, 100.0))

    sim.reset()
    for obj in objs:
        obj.initialize()
    for cam in cams.values():
        cam.initialize()
    for preset, on in PRESETS.items():
        for name in lights:
            prim = stage.GetPrimAtPath(f"/World/lights/{name}")
            intensity, color = on.get(name, (0.0, (1.0, 1.0, 1.0)))
            prim.GetAttribute("inputs:intensity").Set(intensity)
            prim.GetAttribute("inputs:color").Set(Gf.Vec3f(*color))
        for _ in range(60):  # let RTX accumulate
            sim.render()
        for suffix, cam in cams.items():
            path = out / f"{preset}{suffix}.png"
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
