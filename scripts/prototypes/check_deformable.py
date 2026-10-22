"""Headless drop/contact smoke check, not textile calibration or grasp validation."""

import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np

ROOT = Path(__file__).resolve().parents[2]


def verify(path, steps=2500):
    if steps < 1:
        raise ValueError("steps must be positive")
    model = mujoco.MjModel.from_xml_path(str(path.resolve()))
    if model.nflex != 1 or model.flex_dim[0] not in (2, 3):
        raise ValueError("Expected one 2D or 3D flex fixture")
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    initial = data.flexvert_xpos.copy()
    edges = model.flex_edge
    rest = np.linalg.norm(initial[edges[:, 0]] - initial[edges[:, 1]], axis=1)
    finite, max_strain, penetration, contact_seen = True, 0.0, 0.0, False
    for _ in range(steps):
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        finite = finite and all(np.isfinite(x).all() for x in (data.qpos, data.qvel, data.flexvert_xpos))
        if not finite:
            break
        length = np.linalg.norm(data.flexvert_xpos[edges[:, 0]] - data.flexvert_xpos[edges[:, 1]], axis=1)
        max_strain = max(max_strain, float(np.max(np.abs(length / rest - 1))))
        penetration = max(penetration, float(max(0, model.flex_radius[0] - data.flexvert_xpos[:, 2].min())))
        contact_seen = contact_seen or data.ncon > 0
    warnings = [int(w.number) for w in data.warning]
    drop = float(initial[:, 2].mean() - data.flexvert_xpos[:, 2].mean())
    passed = bool(
        finite
        and not any(warnings)
        and contact_seen
        and drop > 0.001
        and (model.flex_dim[0] == 3 or max_strain < 0.05)
        and penetration < 0.002
        and np.isclose(data.time, steps * model.opt.timestep)
    )
    return dict(
        engine=mujoco.__version__,
        model_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        steps=steps,
        simulated_seconds=float(data.time),
        passed_numerical_smoke=passed,
        finite=bool(finite),
        warnings=warnings,
        floor_contact_seen=bool(contact_seen),
        mean_drop_m=drop,
        max_absolute_edge_strain=max_strain,
        max_floor_penetration_m=penetration,
        thresholds={
            "max_absolute_edge_strain": 0.05 if model.flex_dim[0] == 2 else None,
            "max_floor_penetration_m": 0.002,
        },
        real_material_calibrated=False,
        grasp_validated=False,
        self_collision_validated=False,
        tactile_sensor_integrated=False,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, default=ROOT / "real2sim/prototypes/cloth_patch/v1/model.xml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=2500)
    args = parser.parse_args()
    report = verify(args.model, args.steps)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    return 0 if report["passed_numerical_smoke"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
