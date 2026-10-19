"""Record expert demonstrations as XPolicyLab trajectories: one HDF5 per episode, in the layout XPolicyLab's
scripts/transform_lerobot_v30_format.py reads, plus a quality.json sidecar with the acceptance result.

Episodes run through src/eval_client/main.py like an eval, with this module as the in-process policy
(--policy_module), so the recorded state is the observation a policy gets at eval time (scripts/datagen/CONTRACT.md).
The task's expert, acceptance check and audit are scripts/datagen/plugins/<task>.py.

Usage (Isaac env, repo root; run_batch.py runs one per GPU):
    PYTHONPATH=.:XPolicyLab python scripts/datagen/collect.py --task g1d_pour_beans_measuring_cups \
        --env-cfg g1d_beans_collect --seed 9 --output output/datagen/probe [--layout-ids 0,1] [--stop-file PATH] \
        [--device cuda:1 --device_id 1]
Layouts already recorded in --output are skipped, so a rerun resumes.
"""

import argparse
import hashlib
import importlib
import itertools
import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
ENV_KEY = "ROBODOJO_DATAGEN"  # JSON config for the in-process policy; survives main.py's os.execv restarts
CAMERAS = ("cam_head", "cam_left_wrist", "cam_right_wrist")
SECTIONS = ("Rigid", "Articulation", "Fluid", "Geometry", "Clutter", "Dynamic", "Garment")


def plugin_name(task):
    return f"scripts.datagen.plugins.{task}"


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def code_revision():
    """{commit, dirty} of the checkout; None outside a git checkout (an uploaded run dir)."""
    quiet = dict(cwd=ROOT, stderr=subprocess.DEVNULL)
    try:
        commit = subprocess.check_output(["git", "rev-parse", "HEAD"], text=True, **quiet).strip()
        dirty = bool(subprocess.check_output(["git", "status", "--porcelain", "--untracked-files=no"], **quiet))
    except (OSError, subprocess.CalledProcessError):
        return None
    return dict(commit=commit, dirty=dirty)


def load_yaml(path):
    import yaml

    return yaml.safe_load(Path(path).read_text())


def layout_dir(env_cfg, seed):
    config_name = load_yaml(ROOT / "env_cfg" / f"{env_cfg}.yml")["config_name"]
    return ROOT / "Assets" / "Eval_Layout" / "RoboDojo" / config_name / str(seed)


def layout_files(env_cfg, seed, task):
    """The seed folder's layouts in the eval client's order (SeedManager): index = layout id."""
    files = layout_dir(env_cfg, seed).glob(f"{task}_*.json")
    return sorted(
        (p for p in files if p.stem.rpartition("_")[2].isdigit()), key=lambda p: int(p.stem.rpartition("_")[2])
    )


def planned_poses(layout):
    """{label: [position, wxyz]} of every labelled object as laid out (before physics settles it)."""
    poses = {}
    for section in SECTIONS:
        for entries in (layout.get(section) or {}).values():
            for entry in entries:
                if entry.get("label") and "default_pos" in entry:
                    poses[entry["label"]] = [list(entry["default_pos"]), list(entry.get("default_ori", [1, 0, 0, 0]))]
    return poses


def state_keys(env):
    """obs['state'] / action keys of the target arms, in the converters' order: arm, gripper per arm, left first."""
    rm = env.robot_manager
    robots = sorted((r for r in rm.robot_list if r.type == "target"), key=lambda r: r.arm_name)
    return [rm.process_name(name) for r in robots for name in (r.arm_name, r.gripper_name)]


def write_trajectory(path, instruction, info, states, actions, images, privileged):
    """XPolicyLab trajectory layout (README "Trajectory Data Format"): state/ and action/ as <key>s (T, D) float32,
    vision/<camera>/colors as encode_image_bit JPEGs in a fixed-width byte column; privileged/ is ours (audit only)."""
    import h5py

    tmp = path.with_suffix(".tmp")
    try:
        with h5py.File(tmp, "w") as f:
            f["data_format_version"] = "v1.0"
            f["instruction"] = instruction
            for key, value in info.items():
                f[f"additional_info/{key}"] = json.dumps(value) if isinstance(value, dict | list) else value
            for key, rows in states.items():
                f[f"state/{key}s"] = np.asarray(rows, dtype=np.float32)
            for key, rows in actions.items():
                f[f"action/{key}s"] = np.asarray(rows, dtype=np.float32)
            for camera, frames in images.items():
                f[f"vision/{camera}/colors"] = np.asarray(frames, dtype=f"S{max(map(len, frames))}")
            for key, value in privileged.items():
                value = np.asarray(value)
                f.create_dataset(f"privileged/{key}", data=value, compression="gzip" if value.ndim else None)
        tmp.replace(path)
    finally:
        tmp.unlink(missing_ok=True)


def record_episode(env, plugin, output, cfg):
    from XPolicyLab.utils.process_data import encode_image_bit

    if env.num_envs != 1 or env.task_name != plugin.TASK:
        raise ValueError(f"Collector needs one {plugin.TASK} environment")
    layout_id = env.current_env_seed_map[0]
    path = output / f"episode_{layout_id:06d}.hdf5"
    sidecar = path.with_suffix(".json")
    if sidecar.exists():
        raise FileExistsError(sidecar)
    env.end_on_success = False  # acceptance is the plugin's, not the task's success check
    env.step_lim = plugin.STEP_LIM
    layout_file = Path(env.seed_manager.seed_info[layout_id]["scene_layout"])
    quality = dict(
        task=plugin.TASK,
        plugin=plugin.__name__,
        acceptance_version=plugin.ACCEPTANCE,
        hold_frames=plugin.HOLD,
        env_cfg=cfg["env_cfg"],
        config_name=env.config_name,
        physics_dt=env.dt,
        fps=cfg["fps"],
        layout_seed=int(env.eval_seed),
        layout_id=layout_id,
        layout_file=layout_file.name,
        layout_sha256=sha256(layout_file),
        planned_poses=planned_poses(env.seed_manager.get_seed_scene_info(layout_id)),
        revision=cfg.get("revision"),
    )
    episode = plugin.Episode(env)
    obs = env.get_obs()
    quality["instruction"] = instruction = str(obs["instruction"])
    if episode.reject:
        quality.update(accepted=False, reason=episode.reject, **episode.summary())
        sidecar.write_text(json.dumps(quality, indent=2) + "\n")
        env.success[0], env.end_flag[0] = False, True
        return
    keys = state_keys(env)
    rm = env.robot_manager
    joints = {rm.process_name(r.arm_name): list(r.arm_joints_name) for r in rm.robot_list if r.type == "target"}
    states, actions = {k: [] for k in keys}, {k: [] for k in keys}
    images = {camera: [] for camera in CAMERAS}
    stable, last = 0, None
    for action in itertools.chain(episode.actions(), itertools.repeat(None, plugin.TAIL)):
        if action is None and last is None:
            raise RuntimeError("Expert produced no actions")
        action = last = action if action is not None else last
        for key in keys:
            states[key].append(np.asarray(obs["state"][key], dtype=np.float32).reshape(-1))
            actions[key].append(np.asarray(action[key], dtype=np.float32).reshape(-1))
        for camera in CAMERAS:
            images[camera].append(encode_image_bit(np.asarray(obs["vision"][camera]["color"], dtype=np.uint8)))
        env.take_action(action)
        obs = env.get_obs()
        stable = stable + 1 if episode.step() else 0
        frames = len(states[keys[0]])
        if stable >= plugin.HOLD or not env.success[0] or frames >= env.step_lim:
            break
    if not all(np.isfinite(np.concatenate(rows)).all() for rows in (*states.values(), *actions.values())):
        raise ValueError("Non-finite state or action")
    accepted = stable >= plugin.HOLD and bool(env.success[0])
    quality.update(
        accepted=accepted,
        frames=frames,
        stable_frames=stable,
        task_success=bool(env.task_success[0]),
        cameras=list(CAMERAS),
        state_keys=keys,
        joint_names=joints,
        **episode.summary(),
    )
    info = dict(frequency=cfg["fps"], env_cfg=cfg["env_cfg"], physics_dt=env.dt, layout=layout_file.name)
    write_trajectory(path, instruction, info, states, actions, images, episode.privileged())
    sidecar.write_text(json.dumps(quality, indent=2) + "\n")
    env.success[0], env.end_flag[0] = accepted, True
    print(f"[datagen] {path.name}: accepted={accepted} frames={frames} stable={stable}", flush=True)


def eval_one_episode(TASK_ENV, model_client):
    """Policy entry for main.py --policy_module scripts.datagen.collect."""
    cfg = json.loads(os.environ[ENV_KEY])
    record_episode(TASK_ENV, importlib.import_module(cfg["plugin"]), Path(cfg["output"]), cfg)
    if cfg.get("stop_file") and Path(cfg["stop_file"]).exists():
        TASK_ENV.seed_manager.idx = TASK_ENV.seed_manager.ed_idx


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--task", required=True)
    parser.add_argument("--env-cfg", required=True)
    parser.add_argument("--seed", type=int, required=True, help="layout seed folder")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--layout-ids", default="", help="comma-separated; default: every layout in the folder")
    parser.add_argument("--stop-file", type=Path, help="finish the current episode, then stop, once this exists")
    args, remaining = parser.parse_known_args()
    files = layout_files(args.env_cfg, args.seed, args.task)
    wanted = [int(i) for i in args.layout_ids.split(",") if i.strip()] or range(len(files))
    args.output.mkdir(parents=True, exist_ok=True)
    todo = [i for i in wanted if not (args.output / f"episode_{i:06d}.json").exists()]
    if not todo:
        print(f"[datagen] nothing left to record in {args.output}")
        return
    plugin = importlib.import_module(plugin_name(args.task))
    env_cfg = load_yaml(ROOT / "env_cfg" / f"{args.env_cfg}.yml")
    cfg = dict(
        plugin=plugin.__name__,
        output=str(args.output.resolve()),
        env_cfg=args.env_cfg,
        fps=env_cfg["observation"]["collect_freq"],
        stop_file=str(args.stop_file.resolve()) if args.stop_file else None,
        revision=json.loads(os.environ.get("ROBODOJO_REVISION", "null")) or code_revision(),
    )
    os.environ[ENV_KEY] = json.dumps(cfg)
    os.environ["EVAL_NUM"] = str(len(todo))
    from scripts.g1d import scripted_demo

    sys.argv = [sys.argv[0], "--env_cfg_type", args.env_cfg, "--task_name", args.task, "--seed", str(args.seed)]
    sys.argv += ["--layout_ids", ",".join(map(str, todo)), "--allow_train_layouts", *remaining]
    scripted_demo.main(extra_kit_args="--/renderer/multiGpu/enabled=false", policy_module="scripts.datagen.collect")


if __name__ == "__main__":
    main()
