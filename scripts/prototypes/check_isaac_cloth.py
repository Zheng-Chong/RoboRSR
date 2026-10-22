"""Isolated headless GarmentObject drop test; no task/robot/shared-asset changes."""

import argparse
import hashlib
import json
from pathlib import Path
import sys
import traceback

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--asset", type=Path, required=True, help="Generated textile_fixture directory")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=180)
    parser.add_argument("--capture", type=Path, help="Optional final Isaac-rendered PNG")
    parser.add_argument("--aurora-reference", type=Path, help="Optional static visual USD; not a mounted gripper")
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("steps must be positive")
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "width": 640, "height": 480})
    report = {
        "passed": False,
        "real_material_calibrated": False,
        "robot_integrated": False,
        "tactile_simulated": False,
        "steps": args.steps,
        "backend": "Isaac Sim 5.1 particle cloth",
    }
    try:
        from isaacsim.core.api import World
        import numpy as np
        from omegaconf import OmegaConf
        import torch

        from env.scene_manager.objects.garment import GarmentObject

        world = World(physics_dt=1 / 120, rendering_dt=1 / 30, backend="torch", device="cuda:0")
        world.get_physics_context().enable_gpu_dynamics(True)
        world.get_physics_context().set_broadphase_type("GPU")
        world.scene.add_default_ground_plane()
        if args.aurora_reference:
            from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
            from pxr import Gf, UsdGeom

            prim = add_reference_to_stage(str(args.aurora_reference.resolve()), "/World/AuroraVisualReference")
            UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(0.25, 0, 0.1))
            report["aurora_static_reference_loaded"] = True
            report["aurora_robot_mount_verified"] = False

        cfg = json.loads((args.asset / "isaac_instance.json").read_text())
        cloth = GarmentObject(
            "/World/env_0/Object/Garment/cloth_patch",
            str((args.asset / "geometry.usda").resolve()),
            OmegaConf.create(cfg),
            torch.zeros(3, device="cuda:0"),
            [0.0, 0.0, 0.1],
            [1.0, 0.0, 0.0, 0.0],
            [1.0, 1.0, 1.0],
        )
        cloth.particle_system.set_simulation_owner(world.get_physics_context().prim_path)
        camera = None
        if args.capture:
            from isaacsim.core.utils.stage import get_current_stage
            from isaacsim.core.utils.viewports import set_camera_view
            from isaacsim.sensors.camera import Camera
            from pxr import UsdLux

            UsdLux.DomeLight.Define(get_current_stage(), "/World/ReviewLight").CreateIntensityAttr(450.0)
            camera = Camera(prim_path="/World/ReviewCamera", resolution=(640, 480))
            # With Fabric enabled, create and position the camera before reset.
            set_camera_view((0.9, -1.25, 0.75), (0.12, 0, 0.06), camera_prim_path=camera.prim_path)
            camera.set_clipping_range(0.001, 10.0)
        world.reset()
        cloth.initialize()

        def positions():
            return cloth._cloth_prim_view.get_world_positions().detach().cpu().numpy().reshape(-1, 3)

        initial = positions().copy()
        finite = True
        minimum_z = float(initial[:, 2].min())
        peak_strain = 0.0
        grid = json.loads((args.asset / "asset.json").read_text())["config"]
        nx, ny = grid["nx"], grid["ny"]
        edges = []
        for j in range(ny):
            for i in range(nx):
                if i + 1 < nx:
                    edges.append((j * nx + i, j * nx + i + 1))
                if j + 1 < ny:
                    edges.append((j * nx + i, (j + 1) * nx + i))
        edges = np.array(edges)
        rest = np.linalg.norm(initial[edges[:, 0]] - initial[edges[:, 1]], axis=1)
        snapshots = [initial.tolist()]
        for i in range(args.steps):
            world.step(render=i % 4 == 0)
            p = positions()
            finite = finite and bool(np.isfinite(p).all())
            if not finite:
                break
            minimum_z = min(minimum_z, float(p[:, 2].min()))
            peak_strain = max(
                peak_strain, float(np.max(abs(np.linalg.norm(p[edges[:, 0]] - p[edges[:, 1]], axis=1) / rest - 1)))
            )
            if i % 30 == 29:
                snapshots.append(p.tolist())

        if args.capture:
            from PIL import Image

            camera.initialize()
            # Render the final saved physical state without advancing the simulation.
            for _ in range(16):
                world.render()
            rgba = camera.get_rgba()
            if rgba is None or rgba.shape != (480, 640, 4):
                raise RuntimeError("Isaac camera image unavailable")
            args.capture.parent.mkdir(parents=True, exist_ok=True)
            Image.fromarray(rgba).save(args.capture)
            report["capture"] = args.capture.name
        masses = cloth._cloth_prim_view.get_particle_masses().detach().cpu().numpy()
        metadata = json.loads((args.asset / "asset.json").read_text())
        expected = metadata["mass_kg"]
        drop = float(initial[:, 2].mean() - p[:, 2].mean())
        mass = float(masses.sum())
        report.update(
            finite=finite,
            vertices=len(p),
            simulated_seconds=args.steps / 120,
            mean_drop_m=drop,
            minimum_vertex_z_m=minimum_z,
            max_grid_edge_strain=peak_strain,
            total_particle_mass_kg=mass,
            expected_mass_kg=expected,
            snapshots=snapshots,
            geometry_sha256=hashlib.sha256((args.asset / "geometry.usda").read_bytes()).hexdigest(),
            instance_config=cfg,
            passed=bool(
                finite and abs(mass - expected) < 1e-7 and drop > 0.05 and minimum_z > -0.002 and peak_strain < 0.05
            ),
        )
    except Exception as exc:
        report.update(error=str(exc), traceback=traceback.format_exc())
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2) + "\n")
        app.close()
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
