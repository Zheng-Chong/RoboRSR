"""Independent audit of an exported dataset against its raw episodes; the last step before release.

Run with h5py, numpy and scipy (the Isaac env interpreter works, nothing is launched):
    python scripts/datagen/audit.py --dataset output/datagen/office-beans-v3/lerobot/g1d_office_beans_v3
Re-reads every episode's HDF5 (frame counts across state, action and cameras, finite values), recomputes the task's
acceptance from the recorded privileged data (the plugin's audit()), checks that the layouts are distinct and reports
their coverage. Writes validation_raw.json, layouts.csv and coverage.png (with matplotlib), then final_validation.json
and sha256.json over the whole dataset folder; upload.py refuses a dataset without them.
"""

import argparse
import hashlib
import importlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.datagen.export import layout_key  # noqa: E402

BINS = 20


def check_episode(path, quality, plugin):
    import h5py

    n = quality["frames"]
    with h5py.File(path, "r") as f:
        for key in quality["state_keys"]:
            for group in ("state", "action"):
                rows = f[f"{group}/{key}s"][()]
                assert rows.shape[0] == n and np.isfinite(rows).all(), (path, group, key)
        for camera in quality["cameras"]:
            assert f[f"vision/{camera}/colors"].shape == (n,), (path, camera)
        privileged = {k: f[f"privileged/{k}"][()] for k in f["privileged"]}
    return plugin.audit(privileged, quality)


def coverage(qualities):
    """Per labelled object: planned x, y (m) and yaw (deg) ranges and how many of BINS equal bins they fill."""
    rows, report = [], {}
    for q in qualities:
        for label, (pos, wxyz) in sorted(q["planned_poses"].items()):
            yaw = np.degrees(2 * np.arctan2(wxyz[3], wxyz[0]))
            rows.append([q["layout_seed"], q["layout_id"], label, pos[0], pos[1], yaw])
    for label in sorted({r[2] for r in rows}):
        values = np.array([r[3:] for r in rows if r[2] == label])
        report[label] = {}
        for axis, column in zip(("x", "y", "yaw"), values.T):
            low, high = float(column.min()), float(column.max())
            bins = np.clip(((column - low) / (high - low or 1) * BINS).astype(int), 0, BINS - 1)
            report[label][axis] = dict(min=low, max=high, bins_filled=len(set(bins)), bins=BINS)
    return rows, report


def plot(rows, path):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False
    labels = sorted({r[2] for r in rows})
    fig, axes = plt.subplots(1, len(labels), figsize=(4 * len(labels), 4), layout="constrained", squeeze=False)
    for ax, label in zip(axes[0], labels):
        xy = np.array([r[3:5] for r in rows if r[2] == label]) * 100
        ax.scatter(xy[:, 0], xy[:, 1], s=18, alpha=0.8)
        ax.set(title=label, xlabel="x (cm)", ylabel="y (cm)")
        ax.set_aspect("equal")
        ax.grid(alpha=0.2)
    fig.savefig(path, dpi=160)
    return True


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dataset", type=Path, required=True)
    args = parser.parse_args()
    root = args.dataset
    verification = json.loads((root / "export_verification.json").read_text())
    if verification["status"] != "verified":
        raise SystemExit("export.py did not verify this dataset")
    qualities = json.loads((root / "collection_quality.json").read_text())
    plugin = importlib.import_module(qualities[0]["plugin"])
    metrics = []
    for i, q in enumerate(qualities):
        metrics.append(check_episode(Path(q["raw"]), q, plugin))
        if (i + 1) % 10 == 0:
            print(f"audit {i + 1}/{len(qualities)}: pass", flush=True)
    if len({layout_key(q) for q in qualities}) != len(qualities):
        raise SystemExit("Duplicate layouts in the dataset")
    rows, report = coverage(qualities)
    numeric = {k for m in metrics for k, v in m.items() if isinstance(v, int | float)}
    summary = {k: dict(min=min(m[k] for m in metrics), max=max(m[k] for m in metrics)) for k in sorted(numeric)}
    validation = dict(
        episodes=len(qualities), frames=sum(q["frames"] for q in qualities), metrics=summary, coverage=report
    )
    (root / "validation_raw.json").write_text(json.dumps(validation, indent=2) + "\n")
    header = "layout_seed,layout_id,label,x,y,yaw_deg\n"
    (root / "layouts.csv").write_text(header + "".join(",".join(map(str, r)) + "\n" for r in rows))
    plotted = plot(rows, root / "coverage.png")
    final = dict(
        status="validated",
        acceptance_version=qualities[0]["acceptance_version"],
        export=verification,
        audit=dict(episodes=len(qualities), recomputed_acceptance=True, distinct_layouts=True, coverage_plot=plotted),
    )
    (root / "final_validation.json").write_text(json.dumps(final, indent=2) + "\n")
    files = sorted(p for p in root.rglob("*") if p.is_file() and p.name != "sha256.json")
    sums = {str(p.relative_to(root)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files}
    (root / "sha256.json").write_text(json.dumps(sums, indent=2) + "\n")
    print(json.dumps(dict(final, metrics=summary)), flush=True)


if __name__ == "__main__":
    main()
