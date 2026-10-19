"""Release accepted episodes as a LeRobot v3.0 dataset through XPolicyLab's official converter
(XPolicyLab/scripts/transform_lerobot_v30_format.py), verify it with the LeRobot loader and write the dataset card.

Run in the LeRobot env (lerobot==0.4.4 and h5py), repo root; nothing is uploaded:
    python scripts/datagen/export.py --run output/datagen/office-beans-v3 --episodes 100 --name g1d_office_beans_v3
Takes the first --episodes accepted episodes with distinct layouts (planned object poses), stages them as
data/<name>/<task>/<env_cfg>/data/*.hdf5 (the converter's input), converts into <run>/lerobot/<name>, then checks
every episode's length, instruction, and first and last frame against its HDF5. Fails instead of shipping fewer or
unverified episodes. Run scripts/datagen/audit.py on the result next.
"""

import argparse
import importlib.util
import json
import os
from pathlib import Path
import shutil
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
CONVERTER = ROOT / "XPolicyLab/scripts/transform_lerobot_v30_format.py"
CAMERAS = {"cam_high": "cam_head", "cam_left_wrist": "cam_left_wrist", "cam_right_wrist": "cam_right_wrist"}
SAME = ("task", "plugin", "acceptance_version", "env_cfg", "physics_dt", "fps", "cameras", "state_keys")


def layout_key(quality):
    return json.dumps({k: np.round(np.r_[p, q], 5).tolist() for k, (p, q) in sorted(quality["planned_poses"].items())})


def select(raw, count):
    """[(hdf5, quality)]: the first `count` accepted episodes (seed folder, then layout order) with distinct layouts."""
    chosen, keys = [], set()
    for sidecar in sorted(raw.glob("seed*/episode_*.json")):
        quality = json.loads(sidecar.read_text())
        if not quality["accepted"]:
            continue
        if not quality["hold_frames"] <= quality["stable_frames"] <= quality["frames"]:
            raise ValueError(f"Inconsistent acceptance evidence: {sidecar}")
        key = layout_key(quality)
        if key in keys:
            continue
        keys.add(key)
        chosen.append((sidecar.with_suffix(".hdf5"), quality))
        if len(chosen) == count:
            break
    else:
        raise ValueError(f"Only {len(chosen)} accepted episodes with distinct layouts in {raw}; need {count}")
    mixed = [k for k in SAME if len({json.dumps(q[k]) for _, q in chosen}) > 1]
    if mixed:
        raise ValueError(f"Episodes disagree on {mixed}")
    return chosen


def convert(pattern, name, episodes, output):
    """Run the official converter on data/<pattern>, writing <output>/<name>."""
    os.environ["HF_LEROBOT_HOME"] = str(output)  # read by lerobot at import
    spec = importlib.util.spec_from_file_location("transform_lerobot_v30_format", CONVERTER)
    converter = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(converter)
    # It roots data/ and env_cfg/ at its own resolved path, which is another checkout when XPolicyLab is a symlink
    # (as in server run dirs): pin both to this repo.
    converter.DATA_ROOT, converter.ENV_CFG_ROOT = ROOT / "data", ROOT / "env_cfg"
    converter.ROBOT_INFO_PATH = ROOT / "env_cfg/robot/_robot_info.json"
    sys.argv = [str(CONVERTER), pattern, "--repo_id", name, "--max_episode", str(episodes)]
    converter.main()


def verify(dataset_dir, name, chosen):
    import h5py
    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    dataset = LeRobotDataset(name, root=dataset_dir)
    frames = sum(q["frames"] for _, q in chosen)
    if (dataset.num_episodes, dataset.num_frames) != (len(chosen), frames):
        raise ValueError(f"Converted {dataset.num_episodes} episodes / {dataset.num_frames} frames, expected {frames}")
    for index, (path, quality) in enumerate(chosen):
        meta = dataset.meta.episodes[index]
        start, end = int(meta["dataset_from_index"]), int(meta["dataset_to_index"])
        if end - start != quality["frames"] or list(meta["tasks"]) != [quality["instruction"]]:
            raise ValueError(f"Episode {index} ({path}) length or instruction differs")
        with h5py.File(path, "r") as f:
            state = np.concatenate([f[f"state/{k}s"][()] for k in quality["state_keys"]], axis=1)
            action = np.concatenate([f[f"action/{k}s"][()] for k in quality["state_keys"]], axis=1)
        for row, relative in ((start, 0), (end - 1, quality["frames"] - 1)):
            frame = dataset[row]
            assert int(frame["episode_index"]) == index and int(frame["frame_index"]) == relative
            np.testing.assert_allclose(frame["observation.state"].numpy(), state[relative], atol=1e-6)
            np.testing.assert_allclose(frame["action"].numpy(), action[relative], atol=1e-6)
            for camera in CAMERAS:
                assert frame[f"observation.images.{camera}"].shape[0] == 3, camera
    return dict(status="verified", episodes=dataset.num_episodes, frames=dataset.num_frames, fps=dataset.fps)


def card(name, config, chosen, verification):
    q = chosen[0][1]
    revision = config.get("revision") or {}
    seeds = sorted({c["layout_seed"] for _, c in chosen})
    return f"""# {name}

Unitree G1-D (Dex1-1 grippers) `{q["task"]}` demonstrations generated in simulation by RoboDojo's scripted expert
(`scripts/datagen`), converted with XPolicyLab's official LeRobot v3.0 converter.

- Episodes / frames / fps: {verification["episodes"]} / {verification["frames"]} / {verification["fps"]}
- Instruction: "{q["instruction"]}"
- Simulation: env config `{q["env_cfg"]}` (layouts and results under `{q["config_name"]}`), physics step
  {q["physics_dt"]:.6f} s. Evaluate policies trained on this data with `--env-cfg {q["env_cfg"]}`.
- Acceptance: `{q["acceptance_version"]}` (`{q["plugin"]}`), held for {q["hold_frames"]} frames.
- Layouts: seed folders {seeds} of `Eval_Layout/RoboDojo/{q["config_name"]}`, marked as training layouts; do not
  evaluate on them.
- Code: {revision.get("commit", "unknown")} (uncommitted changes: {revision.get("dirty", "unknown")}).

## Features

- `observation.state`, `action` (16): left arm joints 1-7 (rad), left gripper, right arm joints 1-7, right gripper
  (converter motor names `left_joint_0..7`, `right_joint_0..7`). Gripper state is the measured finger opening and the
  gripper action the command, both 0 = closed .. 1 = open; arm actions are joint position targets.
- `observation.images.cam_high` (head, left eye), `cam_left_wrist`, `cam_right_wrist`: 640 x 480 RGB video.

The full data contract, including the mapping to the real robot, is `scripts/datagen/CONTRACT.md` in the RoboDojo repo.
"""


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run", type=Path, required=True, help="run_batch.py output (raw/ and collection_config.json)")
    parser.add_argument("--episodes", type=int, required=True)
    parser.add_argument("--name", required=True, help="dataset name (LeRobot repo id)")
    parser.add_argument("--output", type=Path, help="default <run>/lerobot; the dataset goes to <output>/<name>")
    args = parser.parse_args()
    output = (args.output or args.run / "lerobot").resolve()
    dataset_dir = output / args.name
    staging = ROOT / "data" / args.name
    if args.episodes < 1 or dataset_dir.exists() or staging.exists():
        parser.error(f"--episodes must be positive and {dataset_dir}, {staging} must not exist")
    chosen = select(args.run / "raw", args.episodes)
    task, env_cfg = chosen[0][1]["task"], chosen[0][1]["env_cfg"]
    inputs = staging / task / env_cfg / "data"
    inputs.mkdir(parents=True)
    for index, (path, _) in enumerate(chosen):
        (inputs / f"episode_{index:06d}.hdf5").symlink_to(path.resolve())
    convert(f"{args.name}.{task}.{env_cfg}", args.name, args.episodes, output)
    verification = verify(dataset_dir, args.name, chosen)
    shutil.rmtree(staging)
    config = json.loads((args.run / "collection_config.json").read_text())
    (dataset_dir / "collection_config.json").write_text(json.dumps(config, indent=2) + "\n")
    provenance = [dict(raw=str(path.resolve()), **quality) for path, quality in chosen]
    (dataset_dir / "collection_quality.json").write_text(json.dumps(provenance, indent=2) + "\n")
    (dataset_dir / "export_verification.json").write_text(json.dumps(verification, indent=2) + "\n")
    (dataset_dir / "README.md").write_text(card(args.name, config, chosen, verification))
    print(f"Verified {verification['episodes']} episodes / {verification['frames']} frames: {dataset_dir}")


if __name__ == "__main__":
    main()
