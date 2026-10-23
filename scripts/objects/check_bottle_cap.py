"""Check twistable-cap bottles (make_plastic_bottle.py Articulation twins) in Isaac Sim through env ArticulationObject.

Per bottle, two copies: a free one dropped on the floor must settle upright with the cap closed (cap_twist and
cap_lift at 0); a fixed-base one gets a velocity target on cap_twist: it must unscrew to the open limit with cap_lift
following one thread pitch per turn, then screw back closed.

Usage:
    python scripts/objects/check_bottle_cap.py DIR [DIR ...] [--image out.png]
    e.g. python scripts/objects/check_bottle_cap.py Assets/Object/RoboDojo/Articulation/plastic_bottle/0000{0,1,2,3,4,5}
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Drop and twist-test bottles with articulated caps.")
parser.add_argument("dirs", nargs="+", help="object dirs containing object.usdz and metadata.json")
parser.add_argument("--speed", type=float, default=3.1416, help="cap_twist velocity target, rad/s")
parser.add_argument("--image", help="render the bottles with their caps unscrewed to this PNG")
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(device="cpu")  # as TaskEnv (sim_config device); ArticulationObject reads DOF limits as numpy
args = parser.parse_args()
args.headless = True
args.enable_cameras = bool(args.image)
app = AppLauncher(args).app

import json  # noqa: E402
import os  # noqa: E402
from pathlib import Path  # noqa: E402
import sys  # noqa: E402
import traceback  # noqa: E402

import carb  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
import torch  # noqa: E402

from env.scene_manager.objects.articulation import ArticulationObject  # noqa: E402

DT = 0.004  # env_cfg/sim/sim_config.yml
SPACING = 0.2  # m between bottles along Y; free copies at x = 0, fixed ones at x = FIXED_X
FIXED_X = 0.3


def as_np(x):
    return np.asarray(x.cpu() if hasattr(x, "cpu") else x, dtype=float).reshape(-1)


def step(sim, seconds):
    for _ in range(round(seconds / DT)):
        sim.step(render=False)


def joints(obj):
    q = as_np(obj.get_joint_positions())
    return {n: q[i] for i, n in enumerate(obj.dof_names)}


def main():
    """Returns True if any bottle failed."""
    render = sim_utils.RenderCfg(enable_translucency=True, enable_reflections=True, rendering_mode="quality")
    carb.settings.get_settings().set_float("/rtx/translucency/worldEps", 0.0)
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=DT, device=args.device, render=render))
    row = (len(args.dirs) - 1) * SPACING
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

    bottles = []
    for k, d in enumerate(map(Path, args.dirs)):
        meta = json.loads((d / "metadata.json").read_text())
        half_h = meta["physics"]["size"][2] / 2
        pair = {}
        for env, fixed, x in ((0, False, 0.0), (1, True, FIXED_X)):
            pair["fixed" if fixed else "free"] = ArticulationObject(
                prim_path=f"/World/envs/env_{env}/articulation/{d.parent.name}/{d.parent.name}_{int(d.name)}_{k}",
                usd_path=str(d / "object.usdz"),
                inst_config={"physics": {"fixed_base": fixed}},
                env_origin=np.zeros(3),
                default_pos=(x, k * SPACING, half_h + (0.002 if fixed else 0.01)),
                default_ori=(1.0, 0.0, 0.0, 0.0),
                scale=(1.0, 1.0, 1.0),
            )
        bottles.append((d, pair, half_h, meta["articulation"]))

    camera = None
    if args.image:
        camera = Camera(prim_path="/World/check_camera", resolution=(1280, 720))
        eye, target = (FIXED_X + 0.9, row / 2 - 0.7, 0.45), (FIXED_X, row / 2, 0.12)
        set_camera_view(eye, target, camera_prim_path=camera.prim_path)
    sim.reset()
    for _, pair, *_ in bottles:
        for obj in pair.values():
            obj.initialize()
    step(sim, 2.0)

    failed = False
    for d, pair, half_h, spec in bottles:
        obj = pair["free"]
        pos, quat = map(as_np, obj.get_world_pose())
        up = 1 - 2 * (quat[1] ** 2 + quat[2] ** 2)
        q = joints(obj)
        ok = up > np.cos(np.radians(5)) and abs(pos[2] - half_h) < 0.005 and max(map(abs, q.values())) < 0.02
        failed |= not ok
        print(
            f"{'OK  ' if ok else 'FAIL'} drop  {d}: z={pos[2]:.4f} (expect {half_h:.4f}) "
            f"tilt={np.degrees(np.arccos(min(up, 1.0))):.2f} deg joints={ {n: round(v, 4) for n, v in q.items()} }"
        )

    def twist(speed, seconds):
        for _, pair, *_ in bottles:
            obj = pair["fixed"]
            i = torch.tensor([obj.dof_names.index("cap_twist")])
            obj.apply_action(ArticulationAction(joint_velocities=torch.tensor([speed]), joint_indices=i))
        step(sim, seconds)

    open_s = max(s["cap_twist"]["open"] for *_, s in bottles) / args.speed + 1.5
    twist(args.speed, open_s)
    opened = [joints(pair["fixed"]) for _, pair, *_ in bottles]
    if camera is not None:
        camera.initialize()
        camera.set_clipping_range(0.001, 100.0)
        for _ in range(30):
            sim.render()
        Path(args.image).parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(camera.get_rgba()[:, :, :3]).save(args.image)
        print(f"image: {args.image}")
    twist(-args.speed, open_s)
    for (d, pair, _, spec), q_open in zip(bottles, opened):
        q_closed = joints(pair["fixed"])
        t_open, l_open = spec["cap_twist"]["open"], spec["cap_lift"]["open"]
        ok = (
            abs(q_open["cap_twist"] - t_open) < 0.03 * t_open
            and abs(q_open["cap_lift"] - l_open) < 0.05 * l_open
            and abs(q_closed["cap_twist"]) < 0.03 * t_open
            and abs(q_closed["cap_lift"]) < 0.05 * l_open
        )
        failed |= not ok
        print(
            f"{'OK  ' if ok else 'FAIL'} twist {d}: open twist={q_open['cap_twist']:.3f}/{t_open} rad "
            f"lift={q_open['cap_lift'] * 1000:.2f}/{l_open * 1000:.2f} mm; closed twist={q_closed['cap_twist']:.3f} "
            f"lift={q_closed['cap_lift'] * 1000:.2f} mm"
        )
    return failed


try:
    failed = main()
except Exception:
    traceback.print_exc()
    failed = True
sys.stdout.flush()
sys.stderr.flush()
os._exit(1 if failed else 0)  # app.close() hangs once a camera render product exists, and Kit eats exit codes
