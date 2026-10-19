"""Replay recorded episodes open-loop: the consistency check between the data and the evaluation environment.

Each episode's recorded actions (float32, as a policy trained on the dataset would output them) go to the same layout
in --env-cfg, and the result is compared with the recording: the largest deviation of the observed state from the
recorded one per key (arm joints in rad, gripper opening), the task's own success check and the plugin's acceptance.
In the collection env config a faithful environment repeats the recording; a different physics step, observation or
asset shows up here before anyone trains on the data.

Usage (Isaac env, repo root; one seed folder per run):
    PYTHONPATH=.:XPolicyLab python scripts/datagen/replay.py --raw output/datagen/office-beans-v3/raw/seed100 \
        --report output/datagen/office-beans-v3/replay_seed100.json [--env-cfg g1d] [--layout-ids 0,3] \
        [--device cuda:1 --device_id 1]
--env-cfg defaults to the one the episodes were recorded in. Only accepted episodes are replayed.
"""

import argparse
import importlib
import json
import os
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
ENV_KEY = "ROBODOJO_REPLAY"


def replay_episode(env, path, quality):
    import h5py

    plugin = importlib.import_module(quality["plugin"])
    keys, n = quality["state_keys"], quality["frames"]
    with h5py.File(path, "r") as f:
        actions = {k: f[f"action/{k}s"][()] for k in keys}
        states = {k: f[f"state/{k}s"][()] for k in keys}
    env.end_on_success = False
    env.step_lim = max(env.step_lim, n)
    episode = plugin.Episode(env)
    deviation = dict.fromkeys(keys, 0.0)

    def compare(obs, t):
        for k in keys:
            diff = np.abs(np.asarray(obs["state"][k], dtype=np.float64).reshape(-1) - states[k][t])
            deviation[k] = max(deviation[k], float(diff.max()))

    compare(env.get_obs(), 0)
    stable = steps = 0
    for t in range(n):
        env.take_action({k: actions[k][t] for k in keys})
        obs = env.get_obs()
        stable, steps = stable + 1 if episode.step() else 0, t + 1
        if t + 1 < n:
            compare(obs, t + 1)
        if env.end_flag[0]:
            break
    return dict(
        layout_id=quality["layout_id"],
        raw=str(path),
        frames=n,
        replayed_frames=steps,
        recorded_accepted=quality["accepted"],
        accepted=stable >= plugin.HOLD and bool(env.success[0]),
        stable_frames=stable,
        recorded_task_success=quality.get("task_success"),
        task_success=bool(env.task_success[0]),
        max_state_deviation=deviation,
        **episode.summary(),
    )


def eval_one_episode(TASK_ENV, model_client):
    """Policy entry for main.py --policy_module scripts.datagen.replay."""
    cfg = json.loads(os.environ[ENV_KEY])
    path = Path(cfg["episodes"][str(TASK_ENV.current_env_seed_map[0])])
    result = replay_episode(TASK_ENV, path, json.loads(path.with_suffix(".json").read_text()))
    report = Path(cfg["report"])
    data = json.loads(report.read_text()) if report.exists() else dict(env_cfg=cfg["env_cfg"], episodes=[])
    data["episodes"].append(result)
    report.write_text(json.dumps(data, indent=2) + "\n")
    TASK_ENV.success[0], TASK_ENV.end_flag[0] = result["accepted"], True
    print(f"[replay] layout {result['layout_id']}: accepted={result['accepted']} {result['max_state_deviation']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--raw", type=Path, required=True, help="one seed folder of recorded episodes")
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--env-cfg", help="default: the episodes' own")
    parser.add_argument("--layout-ids", default="")
    args, remaining = parser.parse_known_args()
    wanted = {int(i) for i in args.layout_ids.split(",") if i.strip()}
    episodes = {}
    for sidecar in sorted(args.raw.glob("episode_*.json")):
        quality = json.loads(sidecar.read_text())
        if quality["accepted"] and (not wanted or quality["layout_id"] in wanted):
            episodes[quality["layout_id"]] = quality
    if not episodes:
        sys.exit(f"No accepted episodes to replay in {args.raw}")
    first = next(iter(episodes.values()))
    if len({(q["task"], q["layout_seed"], q["env_cfg"]) for q in episodes.values()}) > 1:
        sys.exit("Replay one task, seed folder and env config at a time")
    if args.report.exists():
        sys.exit(f"{args.report} exists")
    env_cfg = args.env_cfg or first["env_cfg"]
    paths = {str(i): str((args.raw / f"episode_{i:06d}.hdf5").resolve()) for i in episodes}
    os.environ[ENV_KEY] = json.dumps(dict(episodes=paths, report=str(args.report.resolve()), env_cfg=env_cfg))
    os.environ["EVAL_NUM"] = str(len(episodes))
    from scripts.g1d import scripted_demo

    ids = ",".join(map(str, sorted(episodes)))
    sys.argv = [
        sys.argv[0],
        "--env_cfg_type",
        env_cfg,
        "--task_name",
        first["task"],
        "--seed",
        str(first["layout_seed"]),
    ]
    sys.argv += ["--layout_ids", ids, "--allow_train_layouts", *remaining]
    scripted_demo.main(extra_kit_args="--/renderer/multiGpu/enabled=false", policy_module="scripts.datagen.replay")


if __name__ == "__main__":
    main()
