"""Drop RoboDojo rigid objects on a floor in Isaac Sim and check that they settle upright.

Each object dir (object.usdz + metadata.json) is loaded through env RigidObject, the loader tasks use.
Fails if an object tips over, sinks into / floats above the floor, or is still moving after --seconds.

Usage:
    python scripts/objects/check_object.py DIR [DIR ...] [--image out.png] [--image-dir real2sim/objects]
    e.g. python scripts/objects/check_object.py Assets/Object/RoboDojo/Rigid/pitcher/0000{0,1,2}
--image-dir renders every object on its own, to <image-dir>/<type>/<category>/<index>/preview/sim.png (the layout the
Real2Sim platform reads, src/real2sim_platform).
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Drop rigid objects on a floor and check they settle upright.")
parser.add_argument("dirs", nargs="+", help="object dirs containing object.usdz and metadata.json")
parser.add_argument("--seconds", type=float, default=3.0)
parser.add_argument("--image", help="also render the settled objects to this PNG")
parser.add_argument(
    "--image-dir",
    help="also render each object alone and write its check result, to <dir>/<type>/<category>/<index>/preview/",
)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = bool(args.image or args.image_dir)
app = AppLauncher(args).app

import json  # noqa: E402
import os  # noqa: E402
from pathlib import Path  # noqa: E402
import sys  # noqa: E402
import time  # noqa: E402
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
SOLO_SPACING = 0.6  # with --image-dir: neighbours stay outside each object's own view
SOLO_VIEW = np.array([0.82, -0.25, 0.5]) / np.linalg.norm([0.82, -0.25, 0.5])  # eye direction from the object


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
    spacing = SOLO_SPACING if args.image_dir else SPACING
    row = (len(args.dirs) - 1) * spacing
    floor = sim_utils.CuboidCfg(  # static box instead of GroundPlaneCfg, which fetches a USD from Nucleus
        size=(2.0, row + 2.0, 0.02),  # under the whole row of objects
        collision_props=sim_utils.CollisionPropertiesCfg(),
        visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.35, 0.36, 0.38)),
    )
    floor.func("/World/floor", floor, translation=(0.0, row / 2, -0.01))
    dome = sim_utils.DomeLightCfg(intensity=300.0)  # brighter washes out glass
    dome.func("/World/dome", dome)
    sun = sim_utils.DistantLightCfg(intensity=2500.0, angle=5.0)
    sun.func("/World/sun", sun, orientation=(0.9397, 0.2418, 0.2418, 0.0))  # 40 deg off vertical, for shadows

    objs, solo = [], []
    for k, d in enumerate(map(Path, args.dirs)):
        meta = json.loads((d / "metadata.json").read_text())
        size = meta["physics"].get("size") or meta["geometry"]["aligned_bbox"]["extents"]  # official assets: bbox only
        half_h = size[2] / 2  # asset origin is the bbox centre
        category, idx = d.parent.name, int(d.name)
        obj = RigidObject(
            prim_path=f"/World/rigid/{category}/{category}_{idx}_{k}",
            usd_path=str(d / "object.usdz"),
            inst_config={"physics": {"mass": meta["physics"]["mass"]}},
            env_origin=[0.0, 0.0, 0.0],
            default_pos=(0.0, k * spacing, half_h + 0.01),
            default_ori=(1.0, 0.0, 0.0, 0.0),
            scale=(1.0, 1.0, 1.0),
        )
        objs.append((d, obj, half_h, meta, size))
        if args.image_dir:
            cam = Camera(prim_path=f"/World/solo_camera_{k}", resolution=(640, 480))
            centre = np.array([0.0, k * spacing, half_h])
            # USD default lens (50 mm on a 20.955 mm aperture) sees 18 deg vertically at 4:3; the bounding sphere fills
            # ~85% of the height
            eye = centre + SOLO_VIEW * max(3.7 * float(np.linalg.norm(size)), 0.03)
            set_camera_view(tuple(eye), tuple(centre), camera_prim_path=cam.prim_path)
            solo.append((cam, Path(args.image_dir) / d.parent.parent.name / category / d.name / "preview" / "sim.png"))

    camera = None
    if args.image:
        camera = Camera(prim_path="/World/check_camera", resolution=(1280, 720))
        mid_y = (len(objs) - 1) * spacing / 2
        # pose before sim.reset(): later USD xform edits do not reach the renderer with Fabric on
        set_camera_view((1.1, mid_y - 0.9, 0.5), (0.0, mid_y, 0.11), camera_prim_path=camera.prim_path)
    sim.reset()
    for _, obj, *_ in objs:
        obj.initialize()
    for _ in range(round(args.seconds / DT)):
        sim.step(render=False)

    failed = False
    for d, obj, half_h, meta, size in objs:
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
        if args.image_dir:  # read by the Real2Sim platform next to sim.png: the built asset before it is in the library
            report = {
                "settle": "pass" if ok else "fail",
                "z_m": round(float(pos[2]), 4),
                "expect_z_m": round(half_h, 4),
                "tilt_deg": round(float(tilt), 2),
                "speed_m_s": round(float(speed), 4),
                "asset_dir": str(d.resolve()),
                "mass_kg": meta["physics"].get("mass"),
                "size_m": size,
                "measurements_mm": meta.get("measurements_mm") or {},
                "variant_of": meta.get("variant_of"),  # a generalization variant: the platform shows it on that card
                "checked_at": time.strftime("%Y-%m-%d %H:%M:%S"),
            }
            out = Path(args.image_dir) / d.parent.parent.name / d.parent.name / d.name / "preview" / "check.json"
            out.parent.mkdir(parents=True, exist_ok=True)
            out.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")

    cameras = [(camera, Path(args.image))] if camera is not None else []
    cameras += solo
    if cameras:
        for cam, _ in cameras:
            cam.initialize()
            cam.set_clipping_range(0.001, 100.0)  # a 6 mm bean is framed from 3 cm away
        for _ in range(30):  # let RTX accumulate
            sim.render()
        for cam, path in cameras:
            path.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(cam.get_rgba()[:, :, :3]).save(path)
            print(f"image: {path}")
    return failed


try:
    failed = main()
except Exception:
    traceback.print_exc()
    failed = True
sys.stdout.flush()
sys.stderr.flush()
os._exit(1 if failed else 0)  # app.close() hangs once a camera render product exists, and Kit eats exit codes
