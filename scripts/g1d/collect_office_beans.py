"""Record strict, all-bean measuring-cup demonstrations in the office.

Run in the Isaac environment, then export accepted raw episodes in a separate
LeRobot environment. Each frame is observation(t), command(t), before physics.
"""

import argparse
import itertools
import json
import os
from pathlib import Path
import sys

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from scripts.g1d import scripted_demo as demo  # noqa: E402
from scripts.objects import measuring_cup_shape as cup_shape  # noqa: E402

CAMERAS = ("cam_head", "cam_left_wrist", "cam_right_wrist")
TASK = "Pour all red beans from the tall measuring cup into the short measuring cup."
FPS = 30
UPRIGHT_DEG = 30  # side grasps leave the cup leaning; match the task's return-to-upright tolerance


def in_cavity(points, position, quaternion, height):
    """Bean centres inside the cup's actual inner wall (measuring_cup_shape.py, the
    same rings make_plastic_cups.py lofts), excluding the wall and hanging tab.

    A centre must be above the inner bottom and below the rim; no scattered beans ignored.
    """
    local = Rotation.from_quat(np.asarray(quaternion)[[1, 2, 3, 0]]).inv().apply(points - position)
    return cup_shape.in_cavity(local - cup_shape.origin_shift(height), height)


def counts(points, poses):
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError("Invalid or missing bean positions")
    (short, *_), (tall, *_) = cup_shape.MEASURING
    source = in_cavity(points, *poses["source_cup"], tall)
    target = in_cavity(points, *poses["target_cup"], short)
    return dict(
        total=len(points),
        source=int(source.sum()),
        target=int(target.sum()),
        outside=int((~(source | target)).sum()),
        overlap=int((source & target).sum()),
    )


def all_transferred(result, initial_total):
    return initial_total > 0 and result == dict(
        total=initial_total, source=0, target=initial_total, outside=0, overlap=0
    )


def record_episode(env, output):
    from utils.save_file import VideoStreamWriter

    if env.num_envs != 1 or env.task_name != "g1d_pour_beans_measuring_cups" or env.config_name != "g1d":
        raise ValueError("Collector requires one office measuring-cup environment")
    env.step_lim = 1200
    lm = env.scene_manager.layout_manager
    fluid = lm.get_scene_object(0, lm.get_instance_name(0, "beans"))
    arms = sorted(
        [demo.Arm(env, r) for r in env.robot_manager.robot_list if r.type == "target"], key=lambda a: a.arm_key
    )
    layout_id = env.current_env_seed_map[0]
    layout = env.seed_manager.get_seed_scene_info(layout_id)
    planned_poses = {c["label"]: [c["default_pos"], c["default_ori"]] for c in layout["Rigid"]["measuring_cup"]}
    folder = output / f"episode_{layout_id:06d}"
    folder.mkdir(parents=True, exist_ok=False)

    def poses():
        return {
            label: [demo.as_np(v).tolist() for v in lm.get_instance_pose(0, label=label)]
            for label in ("source_cup", "target_cup")
        }

    def particles():
        points = np.asarray(fluid.get_particle_positions()[0], dtype=float)
        return demo.from_wxyz(fluid.init_ori).apply(points) + fluid.init_pos

    initial = particles()
    initial_poses = poses()
    initial_counts = counts(initial, initial_poses)
    if initial_counts["source"] != len(initial):
        (folder / "quality.json").write_text(
            json.dumps(
                dict(
                    accepted=False,
                    reason="Beans not all initially inside source",
                    initial_counts=initial_counts,
                    initial_poses=initial_poses,
                    layout_id=layout_id,
                ),
                indent=2,
            )
            + "\n"
        )
        env.success[0], env.end_flag[0] = False, True
        return
    writers, states, commands, metrics = {}, [], [], []
    particle_history, cup_history = [], []
    stable = 0
    previous = initial
    final = initial_counts
    last_action = None
    obs = env.get_obs()
    original_get_reward = env.reward_manager.get_reward
    # take_action checks the legacy reward internally; keep recording until strict acceptance or timeout.
    env.reward_manager.get_reward = lambda final_check=True: [0.0]
    try:
        for action in itertools.chain(demo.cup_to_cup_actions(env), itertools.repeat(None, 60)):
            if action is None:
                if last_action is None:
                    raise RuntimeError("Planner produced no actions")
                action = last_action
            last_action = action
            state, command = [], []
            for arm in arms:
                fingers = demo.as_np(arm.key.data.joint_pos[0, list(arm.robot.gripper_joint_indices)])
                state.extend(arm.q())
                # Measured aperture in the same 0=closed, 1=open convention as commands.
                state.append(float((0.0245 - fingers.mean()) / 0.0445))
                command.extend(demo.as_np(action[arm.arm_key]))
                command.extend(demo.as_np(action[arm.grip_key]))
            if not np.isfinite(state + command).all():
                raise ValueError("Non-finite state/action")
            states.append(state)
            commands.append(command)
            for camera in CAMERAS:
                rgb = np.asarray(obs["vision"][camera]["color"], dtype=np.uint8)
                if camera not in writers:
                    h, w, c = rgb.shape
                    writers[camera] = VideoStreamWriter(str(folder / f"{camera}.mp4"), h, w, c, fps=FPS)
                writers[camera].append(rgb)
            env.take_action(action)
            obs = env.get_obs()
            points = particles()
            current_poses = poses()
            final = counts(points, current_poses)
            particle_history.append(points.astype(np.float32))
            cup_history.append([sum(current_poses[label], []) for label in ("source_cup", "target_cup")])
            previous = points
            upright = all(
                demo.from_wxyz(p[1]).as_matrix()[2, 2] > np.cos(np.deg2rad(UPRIGHT_DEG)) for p in current_poses.values()
            )
            valid = all_transferred(final, len(initial)) and upright
            stable = stable + 1 if valid else 0
            metrics.append([final[k] for k in ("total", "source", "target", "outside", "overlap")])
            if stable >= FPS or not env.success[0] or len(states) >= env.step_lim:
                break
        for writer in writers.values():
            writer.close(announce=False)
        accepted = stable >= FPS and bool(env.success[0])
        np.savez_compressed(
            folder / "trajectory.npz",
            state=np.asarray(states, dtype=np.float32),
            action=np.asarray(commands, dtype=np.float32),
            counts=np.asarray(metrics),
            particles_post_action=np.asarray(particle_history, dtype=np.float32),
            cup_poses_post_action=np.asarray(cup_history, dtype=np.float32),
            initial_particles=initial,
            final_particles=previous,
        )
        metadata = dict(
            accepted=accepted,
            acceptance_version="all_beans_retained_v2",
            layout_id=layout_id,
            layout_seed=int(env.eval_seed),
            planned_poses=planned_poses,
            task=TASK,
            fps=FPS,
            physics_dt=env.dt,
            frames=len(states),
            initial_poses=initial_poses,
            final_poses=poses(),
            upright_limit_degrees=UPRIGHT_DEG,
            initial_counts=initial_counts,
            final_counts=final,
            stable_frames=stable,
            cameras=CAMERAS,
            state_order=[a.arm_key + "[7]," + a.grip_key + "[1]" for a in arms],
        )
        (folder / "quality.json").write_text(json.dumps(metadata, indent=2) + "\n")
        env.success[0], env.end_flag[0] = accepted, True
        print(f"[strict_collection] {folder.name}: accepted={accepted} counts={final}", flush=True)
    except BaseException:
        for writer in writers.values():
            writer.abort()
        raise
    finally:
        env.reward_manager.get_reward = original_get_reward


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--attempts", type=int, default=1)
    parser.add_argument("--stop-file", type=Path, help="Finish the current episode, then stop if this file exists")
    args, remaining = parser.parse_known_args()
    if args.attempts < 1:
        parser.error("attempts must be positive")
    args.output.mkdir(parents=True, exist_ok=True)
    demo.C2C_POUR = 1.0

    def collect(TASK_ENV, model_client):
        record_episode(TASK_ENV, args.output)
        if args.stop_file is not None and args.stop_file.exists():
            TASK_ENV.seed_manager.idx = TASK_ENV.seed_manager.ed_idx

    demo.eval_one_episode = collect
    os.environ["EVAL_NUM"] = str(args.attempts)
    sys.argv = [
        sys.argv[0],
        "--env_cfg_type",
        "g1d_beans_collect",
        "--task_name",
        "g1d_pour_beans_measuring_cups",
        *remaining,
    ]
    demo.main(extra_kit_args="--/renderer/multiGpu/enabled=false")


if __name__ == "__main__":
    main()
