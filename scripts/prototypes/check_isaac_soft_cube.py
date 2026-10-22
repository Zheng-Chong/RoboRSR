"""Standalone IsaacLab FEM candidate; require report.json passed=true for acceptance.

Kit fast shutdown can mask the process exit code. Missing reports and error.txt
always mean failure, even if the process exits zero.
"""

import argparse
import json
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--steps", type=int, default=180)
    a = p.parse_args()
    a.output.mkdir(parents=True, exist_ok=False)
    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True, "fast_shutdown": True})
    try:
        from isaaclab.assets import DeformableObject, DeformableObjectCfg
        import isaaclab.sim as sim_utils
        import torch

        sim = sim_utils.SimulationContext(sim_utils.SimulationCfg(dt=1 / 120, device="cuda:0"))
        # Local static collider: avoid an external Nucleus ground USD dependency.
        ground = sim_utils.CuboidCfg(
            size=(0.8, 0.8, 0.02),
            collision_props=sim_utils.CollisionPropertiesCfg(),
        )
        ground.func("/World/Ground", ground, translation=(0, 0, -0.01))
        cfg = DeformableObjectCfg(
            prim_path="/World/Cube",
            spawn=sim_utils.MeshCuboidCfg(
                size=(0.1, 0.1, 0.1),
                mass_props=sim_utils.MassPropertiesCfg(mass=0.03),
                deformable_props=sim_utils.DeformableBodyPropertiesCfg(rest_offset=0.0, contact_offset=0.001),
                physics_material=sim_utils.DeformableBodyMaterialCfg(youngs_modulus=20000.0, poissons_ratio=0.3),
                visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.7, 0.7)),
            ),
            init_state=DeformableObjectCfg.InitialStateCfg(pos=(0, 0, 0.15)),
        )
        cube = DeformableObject(cfg)
        sim.reset()
        cube.reset()
        initial = cube.data.nodal_pos_w.clone()
        finite = True
        for _ in range(a.steps):
            cube.write_data_to_sim()
            sim.step()
            cube.update(sim.get_physics_dt())
            finite = finite and bool(torch.isfinite(cube.data.nodal_pos_w).all().item())
        final = cube.data.nodal_pos_w
        initial_center = initial.mean(dim=-2, keepdim=True)
        final_center = final.mean(dim=-2, keepdim=True)
        deformation = (final - final_center) - (initial - initial_center)
        minimum_z = float(final[..., 2].min().item())
        center_drop = float((initial_center[..., 2] - final_center[..., 2]).item())
        settled_height = float(final_center[..., 2].item())
        passed = finite and center_drop > 0.05 and 0.04 < settled_height < 0.06 and minimum_z > -0.003
        report = {
            "backend": "IsaacLab PhysX FEM MeshCuboidCfg",
            "dimensions_m": [0.1, 0.1, 0.1],
            "dimensions_source": "user specified",
            "mass_kg_candidate": 0.03,
            "real_material_calibrated": False,
            "finite": finite,
            "passed": passed,
            "scope": "gravity drop and floor contact; not material calibration or robot grasp",
            "acceptance": {
                "minimum_center_drop_m": 0.05,
                "center_height_range_m": [0.04, 0.06],
                "minimum_node_z_limit_m": -0.003,
            },
            "center_drop_m": center_drop,
            "maximum_centered_node_displacement_m": float(torch.linalg.vector_norm(deformation, dim=-1).max().item()),
            "initial_extent_m": (initial.amax(dim=-2) - initial.amin(dim=-2)).flatten().tolist(),
            "final_extent_m": (final.amax(dim=-2) - final.amin(dim=-2)).flatten().tolist(),
            "steps": a.steps,
            "sim_nodes": int(final.shape[-2]),
            "initial_center_z_m": float(initial[..., 2].mean().item()),
            "final_center_z_m": float(final[..., 2].mean().item()),
            "minimum_z_m": float(final[..., 2].min().item()),
            "engine_equivalence_to_mujoco_verified": False,
        }
        from isaacsim.core.utils.stage import get_current_stage

        get_current_stage().GetRootLayer().Export(str(a.output / "cube_candidate.usda"))
        (a.output / "report.json").write_text(json.dumps(report, indent=2) + "\n")
        print(json.dumps(report))
        return 0 if passed else 1
    except Exception:
        import traceback

        (a.output / "error.txt").write_text(traceback.format_exc())
        traceback.print_exc()
        return 1
    finally:
        # All artifacts are already flushed. This standalone smoke process owns
        # no renderer writers; skip stage cleanup that can block with live FEM views.
        app.close(skip_cleanup=True)


if __name__ == "__main__":
    raise SystemExit(main())
