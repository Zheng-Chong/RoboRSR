"""Qualitative compression/recovery test of an uncalibrated MuJoCo soft block.

The force is distributed over the top grid vertices. No plate, gripper,
measured force curve or tactile sensor is inferred from this numerical check.
"""

import argparse
import hashlib
import json
from pathlib import Path

import mujoco
import numpy as np


def verify(path, force_n=0.3):
    if not np.isfinite(force_n) or force_n <= 0:
        raise ValueError("force_n must be positive and finite")
    m = mujoco.MjModel.from_xml_path(str(path))
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    if m.nflex != 1 or m.flex_dim[0] != 3:
        raise ValueError("Expected one volume flex")
    z = d.flexvert_xpos[:, 2].copy()
    top = np.flatnonzero(np.isclose(z, z.max()))
    bottom = np.flatnonzero(np.isclose(z, z.min()))
    bodies = m.flex_vertbodyid[top]

    def height():
        return float(d.flexvert_xpos[top, 2].mean() - d.flexvert_xpos[bottom, 2].mean())

    finite = True
    penetration = 0.0
    history = []
    for phase, seconds in [("settle", 0.8), ("load", 0.4), ("release", 0.4)]:
        d.xfrc_applied[:] = 0
        if phase == "load":
            d.xfrc_applied[bodies, 2] = -force_n / len(top)
        for i in range(round(seconds / m.opt.timestep)):
            mujoco.mj_step(m, d)
            mujoco.mj_forward(m, d)
            finite = finite and bool(np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all())
            penetration = max(penetration, float(max(0, m.flex_radius[0] - d.flexvert_xpos[:, 2].min())))
            if i % 20 == 0:
                history.append(dict(t_s=float(d.time), phase=phase, height_m=height()))
        if phase == "settle":
            baseline = height()
        elif phase == "load":
            loaded = height()
        else:
            released = height()
    compression = baseline - loaded
    recovery_error = abs(released - baseline)
    warnings = [int(w.number) for w in d.warning]
    return dict(
        passed=bool(
            finite
            and not any(warnings)
            and compression > 1e-5
            and recovery_error < compression * 0.5
            and penetration < 0.002
        ),
        force_n=force_n,
        top_vertices=len(top),
        baseline_height_m=baseline,
        loaded_height_m=loaded,
        released_height_m=released,
        compression_m=compression,
        recovery_error_m=recovery_error,
        max_floor_penetration_m=penetration,
        finite=finite,
        warnings=warnings,
        history=history,
        engine=mujoco.__version__,
        model_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        real_material_calibrated=False,
        tactile_simulated=False,
        scope="qualitative candidate compression and recovery only",
    )


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--model", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--force-n", type=float, default=0.3)
    a = p.parse_args()
    report = verify(a.model, a.force_n)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "history"}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
