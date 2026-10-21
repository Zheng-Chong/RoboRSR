"""Render a scene twin with Isaac RTX and its own USD lights, for the scene cards of the Real2Sim platform.

Loads real2sim/scenes/<dir>/usd/scene.usda alone (no robot) and frames everything outside /Background and /Lights
(the table and what stands on it) from the robot's side, front-right and above, the same view the platform's 3D
preview opens with; the camera is kept inside the room. Writes <dir>/preview/render/sim.jpg (preview/ is not in git;
sync it to the platform machine's checkout). One scene per process: each room brings its own dome light.

Usage (GPU server):
    OMNI_KIT_ACCEPT_EULA=YES python scripts/scenes/render_scenes.py real2sim/scenes/real_scene5_shenzhen_lab
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description="Render a scene twin for its platform card.")
parser.add_argument("scene", help="scene directory, e.g. real2sim/scenes/real_scene3_lab")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.headless = True
args.enable_cameras = True
app = AppLauncher(args).app

import os  # noqa: E402
from pathlib import Path  # noqa: E402
import sys  # noqa: E402
import traceback  # noqa: E402

import carb  # noqa: E402
import isaaclab.sim as sim_utils  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

VIEW = np.array([0.55, -1.0, 0.6]) / np.linalg.norm([0.55, -1.0, 0.6])  # front-right, above; the robot stands at -y
MARGIN = 0.15  # m the camera keeps from the walls and the ceiling


def look_at(eye, target):
    """(w, x, y, z) of a camera at `eye` looking at `target`, Isaac "world" camera axes (+X forward, +Z up)."""
    forward = (target - eye) / np.linalg.norm(target - eye)
    left = np.cross([0.0, 0.0, 1.0], forward)
    left /= np.linalg.norm(left)
    up = np.cross(forward, left)
    m = np.stack([forward, left, up], axis=1)
    w = np.sqrt(max(1e-12, 1 + m[0, 0] + m[1, 1] + m[2, 2])) / 2
    return np.array([w, (m[2, 1] - m[1, 2]) / (4 * w), (m[0, 2] - m[2, 0]) / (4 * w), (m[1, 0] - m[0, 1]) / (4 * w)])


def bounds(prims):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render"])
    box = Gf.Range3d()
    for prim in prims:
        box.UnionWith(cache.ComputeWorldBound(prim).ComputeAlignedRange())
    return np.array(box.GetMin()), np.array(box.GetMax())


def main():
    scene_dir = Path(args.scene).resolve()
    render = sim_utils.RenderCfg(enable_reflections=True, enable_global_illumination=True, rendering_mode="quality")
    carb.settings.get_settings().set_float("/rtx/translucency/worldEps", 0.0)
    sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 60, device=args.device, render=render))
    room = sim.stage.DefinePrim("/World/Scene", "Xform")
    room.GetReferences().AddReference(str(scene_dir / "usd" / "scene.usda"))
    parts = {p.GetName(): p for p in room.GetChildren()}
    if "PhysicsScene" in parts:
        parts["PhysicsScene"].SetActive(False)  # the SimulationContext brings its own
    focus = [p for name, p in parts.items() if name not in ("Background", "Lights", "PhysicsScene")]
    lo, hi = bounds(focus)
    centre, size = (lo + hi) / 2, float(np.linalg.norm(hi - lo))
    eye = centre + VIEW * (1.1 * size + 0.8)
    floor = sim.stage.GetPrimAtPath("/World/Scene/Background/Floor")
    if floor:  # keep the camera inside the room: within the floor's footprint and under the ceiling
        room_lo, room_hi = bounds([floor])
        room_top = bounds([parts["Background"]])[1][2]
        eye[:2] = np.clip(eye[:2], room_lo[:2] + MARGIN, room_hi[:2] - MARGIN)
        eye[2] = min(eye[2], room_top - MARGIN)
    cam = Camera(prim_path="/World/camera", resolution=(1280, 720))
    UsdGeom.Camera(sim.stage.GetPrimAtPath("/World/camera")).GetFocalLengthAttr().Set(16.0)  # ~66 deg horizontal
    sim.reset()
    cam.initialize()
    cam.set_world_pose(eye, look_at(eye, centre), camera_axes="world")
    print(f"focus {np.round(lo, 2).tolist()} .. {np.round(hi, 2).tolist()}, camera at {np.round(eye, 2).tolist()}")
    for _ in range(60):  # let RTX accumulate
        sim.render()
    out = scene_dir / "preview" / "render" / "sim.jpg"
    out.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(cam.get_rgba()[:, :, :3]).save(out, quality=90)
    print(f"image: {out}")


try:
    main()
    failed = False
except Exception:
    traceback.print_exc()
    failed = True
sys.stdout.flush()
sys.stderr.flush()
os._exit(1 if failed else 0)  # app.close() hangs once a camera render product exists, and Kit eats exit codes
