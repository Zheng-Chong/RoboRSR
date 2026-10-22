"""Collect a batch of accepted expert episodes on several GPUs.

Shards a seed folder of candidate layouts into new seed folders marked as training layouts (split.json; the eval
client then refuses them without --allow_train_layouts), runs scripts/datagen/collect.py on each shard, one process per
GPU (--per-gpu N for N per GPU), and stops once --target episodes are accepted. A shard whose process exits before
every layout is recorded is queued again (a process records at most the task's eval_nums layouts, 50 by default).

Usage (Isaac env python, repo root):
    python scripts/datagen/run_batch.py --task g1d_pour_beans_measuring_cups --env-cfg g1d_beans_collect \
        --candidates 9 --shards 100-107 --gpus 0-7 --target 100 --output output/datagen/office-beans-v3
    python scripts/datagen/run_batch.py --output output/datagen/office-beans-v3 --resume
Writes <output>/collection_config.json (code revision, source and asset hashes, layouts), raw/seed<N>/ episodes,
worker_<N>.log and status.json. Then export with scripts/datagen/export.py and audit with scripts/datagen/audit.py.
"""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.datagen.collect import SECTIONS, code_revision, layout_dir, layout_files, load_yaml, sha256  # noqa: E402

SOURCES = ("scripts/datagen", "scripts/g1d", "scripts/objects", "src/eval_client", "env", "task/RoboDojo", "utils")
THREADS = {k: "4" for k in ("OMP_NUM_THREADS", "MKL_NUM_THREADS", "OPENBLAS_NUM_THREADS")}  # 8 Kit processes per box


def span(text):
    """'0-3,6' -> [0, 1, 2, 3, 6]"""
    out = []
    for part in text.split(","):
        low, _, high = part.partition("-")
        out += range(int(low), int(high or low) + 1)
    return out


def write_json(path, data):
    tmp = path.with_suffix(".tmp")
    tmp.write_text(json.dumps(data, indent=2) + "\n")
    tmp.replace(path)


def source_hashes():
    files = [p for d in SOURCES for p in sorted((ROOT / d).rglob("*")) if p.suffix in (".py", ".yml", ".yaml", ".json")]
    files += sorted((ROOT / "env_cfg").rglob("*.yml"))
    return {str(p.relative_to(ROOT)): sha256(p) for p in files if p.is_file() and "__pycache__" not in p.parts}


def asset_hashes(layouts, env_cfg):
    """Objects, rooms and the robot the candidate layouts load."""
    assets = ROOT / "Assets"
    files = set()
    for path in layouts:
        layout = json.loads(path.read_text())
        for section in SECTIONS:
            for key, entries in (layout.get(section) or {}).items():
                for e in entries:
                    kind = "Clutter" if e.get("type") == "cluttered" else section
                    folder = (
                        assets / "Object/RoboDojo" / kind / e.get("category", key) / f"{e.get('category_idx', 0):05d}"
                    )
                    files |= {folder / "object.usdz", folder / "metadata.json"}
        room = (layout.get("Room") or {}).get("default")
        if room:
            files.add(assets / "Room" / room / f"{room}.usd")
    for robot in load_yaml(ROOT / "env_cfg/robot" / f"{env_cfg['config']['robot']}.yml")["robots"]:
        files |= set((assets / "Robots" / robot["robot_name"]).glob("*.urdf"))
    return {str(p.relative_to(assets)): sha256(p) for p in sorted(files) if p.is_file()}


def make_shards(args):
    files = layout_files(args.env_cfg, args.candidates, args.task)
    shards, per = span(args.shards), args.per_shard
    if len(files) < len(shards) * per:
        raise SystemExit(f"{len(files)} candidate layouts, {len(shards)} shards x {per} needs more")
    for i, seed in enumerate(shards):
        folder = layout_dir(args.env_cfg, seed)
        folder.mkdir(parents=True, exist_ok=False)
        chosen = files[i * per : (i + 1) * per]
        for j, path in enumerate(chosen):
            shutil.copy2(path, folder / f"{args.task}_{j}.json")
        write_json(
            folder / "split.json",
            dict(split="train", run=args.output.name, candidates=args.candidates, files=[p.name for p in chosen]),
        )
    candidates = layout_dir(args.env_cfg, args.candidates) / "split.json"
    if not candidates.exists():
        write_json(candidates, dict(split="train", run=args.output.name, note="sharded into seeds " + args.shards))
    return files[: len(shards) * per]


def assign(slots, active, queue):
    """[(slot, seed)] for free (gpu, k) slots in order, popping seeds off the queue until it runs out."""
    return [(slot, queue.pop(0)) for slot in slots if slot not in active and queue]


def accepted_count(raw):
    return sum(json.loads(p.read_text()).get("accepted", False) for p in raw.glob("seed*/episode_*.json"))


def remaining(args, seed):
    done = {p.stem for p in (args.output / "raw" / f"seed{seed}").glob("episode_*.json")}
    return [i for i in range(len(layout_files(args.env_cfg, seed, args.task))) if f"episode_{i:06d}" not in done]


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--resume", action="store_true", help="relaunch on an existing --output with its config")
    parser.add_argument("--task")
    parser.add_argument("--env-cfg")
    parser.add_argument("--candidates", type=int, help="seed folder of candidate layouts")
    parser.add_argument("--shards", help="new seed folders for the shards, e.g. 100-107")
    parser.add_argument("--per-shard", type=int, default=50)
    parser.add_argument("--gpus", default="0", help="e.g. 0-7")
    parser.add_argument("--per-gpu", type=int, default=1, help="worker processes per GPU")
    parser.add_argument("--target", type=int, help="accepted episodes to collect")
    parser.add_argument("--python", default=sys.executable, help="Isaac env interpreter for the workers")
    parser.add_argument("--revision", help="code commit to record when this run dir is not a git checkout")
    args = parser.parse_args()
    config_path = args.output / "collection_config.json"
    if args.resume:
        config = json.loads(config_path.read_text())
        for key in ("task", "env_cfg", "candidates", "shards", "per_shard", "target"):
            setattr(args, key, config[key])
        config.setdefault("resumed_unix", []).append(time.time())
    else:
        if None in (args.task, args.env_cfg, args.candidates, args.shards, args.target):
            parser.error("--task, --env-cfg, --candidates, --shards and --target are required without --resume")
        args.output.mkdir(parents=True, exist_ok=False)
        env_cfg = load_yaml(ROOT / "env_cfg" / f"{args.env_cfg}.yml")
        config = dict(
            task=args.task,
            env_cfg=args.env_cfg,
            config_name=env_cfg["config_name"],
            sim=load_yaml(ROOT / "env_cfg/sim" / f"{env_cfg['config']['sim']}.yml"),
            fps=env_cfg["observation"]["collect_freq"],
            candidates=args.candidates,
            shards=args.shards,
            per_shard=args.per_shard,
            target=args.target,
            revision=code_revision() or (dict(commit=args.revision, dirty=None) if args.revision else None),
            started_unix=time.time(),
        )
        layouts = make_shards(args)
        config.update(source_sha256=source_hashes(), asset_sha256=asset_hashes(layouts, env_cfg))
    write_json(config_path, config)
    raw, stop = args.output / "raw", args.output / "STOP"
    if stop.exists() and accepted_count(raw) < args.target:
        stop.unlink()
    queue = [s for s in span(args.shards) if remaining(args, s)]
    slots = [(gpu, k) for k in range(args.per_gpu) for gpu in span(args.gpus)]  # fill every GPU once, then again
    active, exits, status = {}, {}, {}
    env = dict(os.environ, PYTHONPATH=".:XPolicyLab", OMNI_KIT_ACCEPT_EULA="YES", **THREADS)
    env["ROBODOJO_REVISION"] = json.dumps(config["revision"])
    while queue or active:
        accepted = accepted_count(raw)
        if accepted >= args.target:
            stop.touch()
            queue.clear()
        for slot, seed in assign(slots, active, queue):
            gpu = slot[0]
            log = (args.output / f"worker_{seed}.log").open("a")
            cmd = [args.python, "-u", "scripts/datagen/collect.py", "--task", args.task, "--env-cfg", args.env_cfg]
            cmd += ["--seed", str(seed), "--output", str(raw / f"seed{seed}"), "--stop-file", str(stop)]
            cmd += ["--device", f"cuda:{gpu}", "--device_id", str(gpu)]
            run_env = dict(env, ROBODOJO_RUN_ID=f"{args.output.name}-seed{seed}-{int(time.time())}")
            proc = subprocess.Popen(
                cmd, cwd=ROOT, env=run_env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True
            )
            active[slot] = (seed, proc, log)
            print(f"[run_batch] seed {seed} on GPU {gpu}, pid {proc.pid}", flush=True)
        for slot, (seed, proc, log) in list(active.items()):
            if proc.poll() is not None:
                log.close()
                exits.setdefault(str(seed), []).append(proc.returncode)
                del active[slot]
                if proc.returncode == 0 and not stop.exists() and remaining(args, seed):
                    queue.append(seed)  # stopped at eval_nums layouts: run the rest
        status = dict(
            accepted=accepted,
            attempted=len(list(raw.glob("seed*/episode_*.json"))),
            target=args.target,
            active={str(s): p.pid for s, p, _ in active.values()},
            queued=queue,
            exit_codes=exits,
            state="collecting" if accepted < args.target else "finishing",
            updated_unix=time.time(),
        )
        write_json(args.output / "status.json", status)
        if queue or active:
            time.sleep(5)
    accepted = accepted_count(raw)
    write_json(args.output / "status.json", dict(status, accepted=accepted, state="done", updated_unix=time.time()))
    print(json.dumps(dict(accepted=accepted, target=args.target, exit_codes=exits)), flush=True)
    if accepted < args.target:
        sys.exit(f"Only {accepted}/{args.target} accepted: add candidate layouts and shards, then --resume")


if __name__ == "__main__":
    main()
