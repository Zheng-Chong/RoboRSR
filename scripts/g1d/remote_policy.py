"""Evaluate a G1-D policy served over the openpi-style msgpack WebSocket protocol of the real robot (OpenWAM's
scripts/serve_g1d.py): one metadata frame on connect, then observation/state (joint16: left arm 7, right arm 7, left
and right gripper in raw Dex1 units 0 closed .. 5.4 open) plus observation/images/{head_left,wrist_left,wrist_right}
in, actions [horizon, 16] absolute joint targets in the same order out.

Episodes run through src/eval_client/main.py with this module as the in-process policy (--policy_module), under the
datagen plugin's protocol: no end on the task's own success check, the plugin's step limit, and the episode stops
once the plugin's strict acceptance has held for its HOLD frames. A per-layout JSON records both the task check
(also the success counted in _result.json) and the strict acceptance.

Gripper: the sim training labels are the next frame's measured opening, which stalls at ~0.75 on a held cup while
the expert commanded 0. A predicted opening below --close-below is therefore sent as 0 (close), else as predicted.

Usage (Isaac env, repo root; policy server already listening):
    PYTHONPATH=.:XPolicyLab python scripts/g1d/remote_policy.py --task g1d_pour_beans_measuring_cups \
        --env-cfg g1d_beans_collect --seed 900 --output output/policy_eval/probe --server ws://127.0.0.1:8001 \
        [--layout-ids 0,1] [--device cuda:1 --device_id 1]
"""

import argparse
import importlib
import json
import os
from pathlib import Path
import runpy
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
ENV_KEY = "ROBODOJO_REMOTE_POLICY"  # JSON config for the in-process policy; survives main.py's os.execv restarts
RAW_OPEN = 5.4  # Dex1 reading when fully open; sim gripper state and command are 0 closed .. 1 open
SETTLE = 10  # steps the --init-joints start is held before the episode's frames count
CAMERAS = {"cam_head": "head_left", "cam_left_wrist": "wrist_left", "cam_right_wrist": "wrist_right"}


def codec():
    """msgpack with numpy arrays as {__ndarray__, data, dtype, shape} maps (openpi msgpack_numpy)."""
    import msgpack

    def pack(obj):
        if isinstance(obj, np.ndarray):
            return {b"__ndarray__": True, b"data": obj.tobytes(), b"dtype": obj.dtype.str, b"shape": obj.shape}
        if isinstance(obj, np.generic):
            return {b"__npgeneric__": True, b"data": obj.item(), b"dtype": obj.dtype.str}
        return obj

    def unpack(obj):
        if b"__ndarray__" in obj:
            return np.ndarray(buffer=obj[b"data"], dtype=np.dtype(obj[b"dtype"]), shape=obj[b"shape"])
        if b"__npgeneric__" in obj:
            return np.dtype(obj[b"dtype"]).type(obj[b"data"])
        return obj

    return lambda o: msgpack.packb(o, default=pack), lambda b: msgpack.unpackb(b, object_hook=unpack)


def arm_keys(env):
    """{side: (arm key, gripper key)} of the target arms."""
    rm = env.robot_manager
    return {
        r.arm_name.split("_")[0]: (rm.process_name(r.arm_name), rm.process_name(r.gripper_name))
        for r in rm.robot_list
        if r.type == "target"
    }


def joint16(obs, keys):
    state = [np.asarray(obs["state"][keys[s][0]], np.float32).reshape(7) for s in ("left", "right")]
    grips = [float(np.asarray(obs["state"][keys[s][1]]).reshape(-1)[0]) * RAW_OPEN for s in ("left", "right")]
    return np.concatenate([*state, np.float32(grips)])


def finger_joints(env):
    """[[left finger 1, finger 2], [right finger 1, finger 2]] joint positions (m) of env 0."""
    rm = env.robot_manager
    rows = {}
    for idx, robot in enumerate(rm.robot_list):
        if robot.type == "target":
            q = rm.robot_key[idx].data.joint_pos[0, robot.gripper_joint_indices]
            rows[robot.arm_name.split("_")[0]] = q.cpu().numpy().astype(np.float32)
    return np.stack([rows["left"], rows["right"]])


def request(env, obs, keys, prompt=None):
    msg = {"observation/state": joint16(obs, keys)}
    if prompt is not None:
        msg["prompt"] = prompt
    for camera, name in CAMERAS.items():
        msg[f"observation/images/{name}"] = np.ascontiguousarray(obs["vision"][camera]["color"], dtype=np.uint8)
    return msg


def smooth_chunk(chunk, last, sigma, blend):
    """Arm joints only: Gaussian low-pass over the chunk, then a linear blend of its first `blend` rows from the
    previous command `last`. The chunks carry step-to-step noise ~10x the expert's and jump at chunk boundaries."""
    from scipy.ndimage import gaussian_filter1d

    chunk = chunk.copy()
    if sigma > 0:
        chunk[:, :14] = gaussian_filter1d(chunk[:, :14], sigma, axis=0, mode="nearest")
    if blend > 0 and last is not None:
        n = min(blend, len(chunk))
        w = (1 - np.arange(1, n + 1) / (blend + 1))[:, None]
        chunk[:n, :14] = w * last[:14] + (1 - w) * chunk[:n, :14]
    return chunk


def to_action(row, keys, close_below):
    action = {}
    for side, (arm, grip) in keys.items():
        sl, g = (slice(0, 7), 14) if side == "left" else (slice(7, 14), 15)
        opening = float(np.clip(row[g] / RAW_OPEN, 0, 1))
        action[arm] = np.asarray(row[sl], np.float64)
        action[grip] = [0.0 if opening < close_below else opening]
    return action


def place_arms(env, keys, q14, close_below, settle=SETTLE):
    """Put the target arms at q14 [L7, R7] (rad), grippers open, and hold them there for `settle` steps."""
    import torch

    rm = env.robot_manager
    for idx, robot in enumerate(rm.robot_list):
        if robot.type != "target":
            continue
        key = rm.robot_key[idx]
        side = robot.arm_name.split("_")[0]
        q = torch.tensor([q14[:7] if side == "left" else q14[7:]], dtype=torch.float32, device=key.device)
        key.write_joint_state_to_sim(q, torch.zeros_like(q), joint_ids=robot.arm_joint_indices)
        key.set_joint_position_target(q, joint_ids=robot.arm_joint_indices)
    hold = to_action(np.r_[q14, RAW_OPEN, RAW_OPEN], keys, close_below)
    for _ in range(settle):
        env.take_action(hold)


def run_episode(env, plugin, output, cfg):
    from websockets.sync.client import connect

    layout_id = env.current_env_seed_map[0]
    sidecar = output / f"episode_{layout_id:06d}.json"
    env.end_on_success = False  # strict acceptance needs the beans settled after the task check fires
    env.step_lim = plugin.STEP_LIM
    start = (cfg.get("init_joints") or {}).get(str(layout_id))
    if start is not None:
        settle = cfg.get("settle_steps", SETTLE)
        env.step_lim += settle
        place_arms(env, arm_keys(env), start, cfg["close_below"], settle)
    episode = plugin.Episode(env)
    obs = env.get_obs()
    result = dict(task=plugin.TASK, acceptance_version=plugin.ACCEPTANCE, layout_seed=int(env.eval_seed))
    result.update(layout_id=layout_id, server=cfg["server"], close_below=cfg["close_below"])
    prompt = (cfg.get("prompts") or {}).get(str(layout_id))
    result.update(smooth_sigma=cfg["smooth_sigma"], blend_steps=cfg["blend_steps"], start_joints=start, prompt=prompt)
    if episode.reject:
        result.update(accepted=False, reason=episode.reject, **episode.summary())
        sidecar.write_text(json.dumps(result, indent=2) + "\n")
        env.success[0], env.end_flag[0] = False, True
        return
    keys = arm_keys(env)
    packb, unpackb = codec()
    frames, stable, infer = 0, 0, []
    # joint16 (raw gripper units): state, server row, sent row; fingers: both finger joints of each gripper (m)
    trace = dict(state=[], raw=[], command=[], chunk=[], fingers=[])
    last = None
    with connect(cfg["server"], max_size=None, compression=None, open_timeout=60) as ws:
        meta = unpackb(ws.recv())
        while frames < env.step_lim and stable < plugin.HOLD and not env.end_flag[0]:
            t0 = time.monotonic()
            ws.send(packb(request(env, obs, keys, prompt)))
            reply = ws.recv()
            if isinstance(reply, str):
                raise RuntimeError(f"policy server error:\n{reply}")
            infer.append(time.monotonic() - t0)
            raw = np.asarray(unpackb(reply)["actions"], np.float64)
            for raw_row, row in zip(raw, smooth_chunk(raw, last, cfg["smooth_sigma"], cfg["blend_steps"])):
                trace["state"].append(joint16(obs, keys))
                trace["raw"].append(raw_row)
                trace["command"].append(row)
                trace["chunk"].append(len(infer) - 1)
                trace["fingers"].append(finger_joints(env))
                last = row
                env.take_action(to_action(row, keys, cfg["close_below"]))
                obs = env.get_obs()
                frames += 1
                stable = stable + 1 if episode.step() else 0
                if frames >= env.step_lim or stable >= plugin.HOLD or env.end_flag[0]:
                    break
    accepted = stable >= plugin.HOLD
    result.update(
        accepted=accepted,
        task_success=bool(env.task_success[0]),
        frames=frames,
        stable_frames=stable,
        chunks=len(infer),
        mean_chunk_seconds=float(np.mean(infer)),
        policy=meta.get("policy"),
        action_horizon=meta.get("action_horizon"),
        **episode.summary(),
    )
    sidecar.write_text(json.dumps(result, indent=2) + "\n")
    np.savez_compressed(sidecar.with_suffix(".npz"), **{k: np.asarray(v) for k, v in trace.items()})
    env.success[0], env.end_flag[0] = bool(env.task_success[0]), True
    print(f"[remote_policy] layout {layout_id}: task={result['task_success']} strict={accepted} frames={frames}")


def eval_one_episode(TASK_ENV, model_client):
    """Policy entry for main.py --policy_module scripts.g1d.remote_policy."""
    cfg = json.loads(os.environ[ENV_KEY])
    run_episode(TASK_ENV, importlib.import_module(cfg["plugin"]), Path(cfg["output"]), cfg)


def main():
    from scripts.datagen.collect import layout_files, plugin_name

    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--task", required=True)
    parser.add_argument("--env-cfg", required=True)
    parser.add_argument("--seed", type=int, required=True, help="layout seed folder (not marked train)")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--server", required=True, help="ws://host:port of the policy server")
    parser.add_argument("--policy-name", default="remote_policy", help="eval_result/<task>/<policy-name>/...")
    parser.add_argument("--layout-ids", default="", help="comma-separated; default: every layout in the folder")
    parser.add_argument("--close-below", type=float, default=0.87, help="predicted opening below this closes")
    parser.add_argument("--smooth-sigma", type=float, default=0.0, help="Gaussian sigma (actions) over each chunk")
    parser.add_argument("--blend-steps", type=int, default=0, help="rows blended from the previous command")
    parser.add_argument(
        "--init-joints", type=Path, help="JSON {layout id: [L7, R7] arm joints}: each listed episode starts there"
    )
    parser.add_argument(
        "--settle-steps", type=int, default=SETTLE, help="steps an --init-joints start is held (beans settle too)"
    )
    parser.add_argument(
        "--prompts", type=Path, help="JSON {layout id: instruction} sent as the request's prompt (server default else)"
    )
    args, remaining = parser.parse_known_args()
    files = layout_files(args.env_cfg, args.seed, args.task)
    wanted = [int(i) for i in args.layout_ids.split(",") if i.strip()] or range(len(files))
    args.output.mkdir(parents=True, exist_ok=True)
    todo = [i for i in wanted if not (args.output / f"episode_{i:06d}.json").exists()]
    if not todo:
        print(f"[remote_policy] nothing left to evaluate in {args.output}")
        return
    cfg = dict(plugin=plugin_name(args.task), output=str(args.output.resolve()), server=args.server)
    cfg.update(close_below=args.close_below, smooth_sigma=args.smooth_sigma, blend_steps=args.blend_steps)
    cfg.update(init_joints=json.loads(args.init_joints.read_text()) if args.init_joints else None)
    cfg.update(settle_steps=args.settle_steps)
    cfg.update(prompts=json.loads(args.prompts.read_text()) if args.prompts else None)
    os.environ[ENV_KEY] = json.dumps(cfg)
    os.environ["EVAL_NUM"] = str(len(todo))
    sys.argv = [str(ROOT / "src/eval_client/main.py"), "--env_cfg_type", args.env_cfg, "--task_name", args.task]
    sys.argv += ["--seed", str(args.seed), "--layout_ids", ",".join(map(str, todo)), *remaining]
    if "--device_id" not in remaining:
        sys.argv += ["--device_id", "0"]
    sys.argv += ["--policy_name", args.policy_name, "--port", "0", "--policy_module", "scripts.g1d.remote_policy"]
    sys.argv += ["--additional_info", Path(args.output).name, "--headless", "--enable_cameras"]
    sys.argv += [
        "--kit_args",
        "--enable isaacsim.replicator.behavior --enable isaacsim.sensors.camera --/renderer/multiGpu/enabled=false",
    ]
    runpy.run_path(sys.argv[0], run_name="__main__")


if __name__ == "__main__":
    main()
