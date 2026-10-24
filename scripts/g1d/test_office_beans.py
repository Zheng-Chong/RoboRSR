"""CPU checks for the strict acceptance boundary (no Isaac imports)."""

import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
from scipy.spatial.transform import Rotation

from scripts.g1d import collect_office_beans as collector
from scripts.g1d.collect_office_beans import all_transferred, counts, in_cavity
from scripts.g1d.export_office_beans import select_episodes, validated_trajectory
from scripts.g1d.prepare_office_beans_layouts import RANGES, sample_layouts
from scripts.objects.measuring_cup_shape import origin_shift


class AcceptanceTests(unittest.TestCase):
    def test_round_target_cavity(self):
        cup = dict(height=0.089, radius=0.031, bottom=0.004)
        at = np.array([0.1, -0.5, 0.88])  # bbox centre
        upright, tilted = [1.0, 0.0, 0.0, 0.0], Rotation.from_euler("x", 90, degrees=True).as_quat()[[3, 0, 1, 2]]
        points = at + np.array([[0, 0, 0], [0.025, 0, -0.03], [0, 0, 0.05], [0.035, 0, 0], [0, 0, -0.043]])
        self.assertEqual(collector.in_round_cavity(points, at, upright, cup).tolist(), [1, 1, 0, 0, 0])
        low = at + [[0, 0, -0.04]]  # just above the bottom when upright, past the wall once the cup lies down
        self.assertTrue(collector.in_round_cavity(low, at, upright, cup)[0])
        self.assertFalse(collector.in_round_cavity(low, at, tilted, cup)[0])
        poses = {"source_cup": [at + [1, 0, 0], upright], "target_cup": [at, upright]}
        self.assertEqual(counts(points, poses, cup)["target"], 2)
        tapered = dict(height=0.099, radius=0.0345, radius_bottom=0.0225, bottom=0.003)
        side = at + np.array([[0.03, 0, 0.045], [0.03, 0, -0.045]])  # 3 cm out: in near the rim, out near the bottom
        self.assertEqual(collector.in_round_cavity(side, at, upright, tapered).tolist(), [1, 0])

    def test_export_checks_trajectory_evidence_not_only_acceptance_flag(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory)
            quality = dict(frames=30, fps=30, stable_frames=30, initial_counts={"total": 2})
            counts = np.tile([2, 0, 2, 0, 0], (30, 1))
            for spill in (False, True):
                if spill:
                    counts[-5] = [2, 0, 1, 1, 0]
                np.savez(folder / "trajectory.npz", state=np.zeros((30, 16)), action=np.zeros((30, 16)), counts=counts)
                if spill:
                    with self.assertRaisesRegex(ValueError, "retain every bean"):
                        validated_trajectory(folder, quality)
                else:
                    state, _ = validated_trajectory(folder, quality)
                    self.assertEqual(state.shape, (30, 16))

    def test_recording_survives_legacy_success_and_saves_aligned_frames(self):
        fluid = SimpleNamespace(init_ori=[1, 0, 0, 0], init_pos=np.zeros(3))
        fluid.points = np.array([[-0.153, 0, 0]])
        fluid.get_particle_positions = lambda: (fluid.points, None, None)
        poses = {"source_cup": ([-0.15, 0, 0], [1, 0, 0, 0]), "target_cup": ([0.15, 0, 0], [1, 0, 0, 0])}
        layout = SimpleNamespace(
            get_instance_name=lambda *args: "beans",
            get_scene_object=lambda *args: fluid,
            get_instance_pose=lambda _, label: poses[label],
        )

        def legacy_reward(final_check=True):
            return [1.0]

        env = SimpleNamespace(
            num_envs=1,
            task_name="g1d_pour_beans_measuring_cups",
            config_name="g1d",
            dt=1 / 600,
            scene_manager=SimpleNamespace(layout_manager=layout),
            robot_manager=SimpleNamespace(
                robot_list=[SimpleNamespace(type="target", side=s) for s in ("left", "right")]
            ),
            current_env_seed_map={0: 0},
            eval_seed=0,
            seed_manager=SimpleNamespace(
                get_seed_scene_info=lambda _: {
                    "Rigid": {
                        "measuring_cup": [
                            dict(label=label, default_pos=pose[0], default_ori=pose[1]) for label, pose in poses.items()
                        ]
                    }
                }
            ),
            reward_manager=SimpleNamespace(get_reward=legacy_reward),
            success=[True],
            end_flag=[False],
        )
        env.get_obs = lambda: {
            "vision": {cam: {"color": np.zeros((4, 4, 3), dtype=np.uint8)} for cam in collector.CAMERAS}
        }

        def take_action(action):
            self.assertEqual(env.reward_manager.get_reward(), [0.0])
            fluid.points = np.array([[0.147, 0, 0]])

        env.take_action = take_action

        def arm(_, robot):
            return SimpleNamespace(
                arm_key=robot.side,
                grip_key=robot.side + "_gripper",
                q=lambda: np.zeros(7),
                key=SimpleNamespace(data=SimpleNamespace(joint_pos=np.zeros((1, 2)))),
                robot=SimpleNamespace(gripper_joint_indices=(0, 1)),
            )

        action = {"left": np.zeros(7), "right": np.zeros(7), "left_gripper": [0.0], "right_gripper": [0.0]}

        class Writer:
            def __init__(self, *args, **kwargs):
                self.frames = 0

            def append(self, image):
                self.frames += 1

            def close(self, **kwargs):
                pass

            def abort(self):
                pass

        with (
            tempfile.TemporaryDirectory() as directory,
            patch.object(collector.demo, "Arm", arm),
            patch.object(collector.demo, "cup_to_cup_actions", lambda _: iter([action])),
            patch("utils.save_file.VideoStreamWriter", Writer),
        ):
            collector.record_episode(env, Path(directory))
            folder = Path(directory) / "episode_000000"
            quality = json.loads((folder / "quality.json").read_text())
            self.assertTrue(quality["accepted"])
            self.assertEqual(quality["stable_frames"], 30)
            with np.load(folder / "trajectory.npz") as data:
                self.assertEqual(data["state"].shape, (quality["frames"], 16))
                np.testing.assert_array_equal(data["state"][0, :7], np.zeros(7))
                self.assertEqual(data["action"].shape, data["state"].shape)
        self.assertIs(env.reward_manager.get_reward, legacy_reward)

    def test_layout_sampling_balances_hands_and_covers_each_axis(self):
        samples = np.asarray(sample_layouts(100, 20261019))
        self.assertEqual(len(set(map(tuple, samples))), 100)
        self.assertTrue(np.array_equal(samples, sample_layouts(100, 20261019)))
        side = samples[:, 0]
        self.assertEqual(sorted(side), [-1] * 50 + [1] * 50)
        np.testing.assert_array_equal(np.sign(samples[:, 1]), side)  # tall cup on the pouring hand's side
        np.testing.assert_array_equal(np.sign(samples[:, 3]), -side)
        unsigned = np.c_[np.abs(samples[:, 1]), samples[:, 2], np.abs(samples[:, 3]), samples[:, 4:]]
        for column, (low, high) in zip(unsigned.T, RANGES):
            bins = ((column - low) / (high - low) * 100).astype(int)
            self.assertEqual(sorted(bins), list(range(100)))

    def test_cavity_excludes_tab_wall_and_below_bottom(self):
        # cup frame (outside bottom centre): centre, wall, chamfered bottom corner, same xy under the rounded mouth,
        # tab, below the inner bottom
        bottom = [[0, 0, 0.07], [0.032, 0, 0.07], [0.027, 0.027, 0.01], [0.027, 0.027, 0.13], [0.041, 0, 0.128]]
        points = np.array(bottom + [[0, 0, 0.001]]) + origin_shift(0.14)
        expected = [True, False, False, True, False, False]
        self.assertEqual(in_cavity(points, [0, 0, 0], [1, 0, 0, 0], 0.14).tolist(), expected)
        rotation = Rotation.from_euler("xyz", [20, -80, 45], degrees=True)
        q = rotation.as_quat()[[3, 0, 1, 2]]
        shifted = rotation.apply(points) + [1, 2, 3]
        self.assertEqual(in_cavity(shifted, [1, 2, 3], q, 0.14).tolist(), expected)

    def test_one_spilled_or_residual_bean_fails(self):
        poses = {"source_cup": [[-0.15, 0, 0], [1, 0, 0, 0]], "target_cup": [[0.15, 0, 0], [1, 0, 0, 0]]}
        particles = np.array([[0.147, 0, 0], [0.147, 0.005, 0]])
        self.assertTrue(all_transferred(counts(particles, poses), 2))
        for bad in ([0, 0, 0], [-0.153, 0, 0]):
            particles[1] = bad
            self.assertFalse(all_transferred(counts(particles, poses), 2))
        self.assertFalse(all_transferred(counts(particles[:1], poses), 2))
        with self.assertRaises(ValueError):
            counts(np.array([[float("nan"), 0, 0]]), poses)

    def test_export_rejects_shortfall_and_duplicate_positions(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            quality = dict(
                accepted=True,
                initial_counts={"total": 2},
                final_counts=dict(total=2, source=0, target=2, outside=0, overlap=0),
                stable_frames=30,
                fps=30,
                planned_poses={
                    "source_cup": [[-0.1, -0.3, 0], [1, 0, 0, 0]],
                    "target_cup": [[0.1, -0.3, 0], [1, 0, 0, 0]],
                },
            )
            for i in range(2):
                folder = root / str(i)
                folder.mkdir()
                (folder / "quality.json").write_text(json.dumps(quality))
            self.assertEqual(len(select_episodes(root, 1)), 1)
            with self.assertRaisesRegex(ValueError, "Only 1 unique"):
                select_episodes(root, 2)
            quality["final_counts"]["target"] = 1
            (root / "0/quality.json").write_text(json.dumps(quality))
            with self.assertRaisesRegex(ValueError, "Invalid acceptance"):
                select_episodes(root, 1)


if __name__ == "__main__":
    unittest.main()
