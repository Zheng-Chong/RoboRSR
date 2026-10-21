"""CPU checks for the data pipeline and its contract (no Isaac): python -m pytest scripts/datagen"""

import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

sys.modules.setdefault("env.seeding", ModuleType("env.seeding"))  # torch + warp; only DescManager seeds with it
sys.modules["env.seeding"].seed_everywhere = getattr(sys.modules["env.seeding"], "seed_everywhere", lambda *a: None)

from env.observation_manager.obs_manager import ObsManager  # noqa: E402
from env.seed_manager import seed_manager  # noqa: E402
from scripts.datagen import collect, export, run_batch  # noqa: E402

try:
    from XPolicyLab.utils.data_loader import load as load_trajectory
    from XPolicyLab.utils.process_data import encode_image_bit  # noqa: F401
except ImportError:
    load_trajectory = None
needs_xpolicylab = unittest.skipIf(load_trajectory is None, "XPolicyLab (and h5py, cv2) not importable")
KEYS = ["left_arm_joint_state", "left_ee_joint_state", "right_arm_joint_state", "right_ee_joint_state"]
CLOSED, OPEN = 0.0245, -0.02  # Dex1 finger joint travel, closed .. open


def dex1(side):
    return SimpleNamespace(
        type="target",
        robot_type="arm",
        arm_name=f"{side}_arm",
        gripper_name=f"{side}_ee",
        ee_type="gripper",
        gripper_scale=[OPEN, CLOSED],
        gripper_move={"sign": -1.0, "base": f"{side}_1", "mimic": [f"{side}_2", 1.0, 0.0]},
        gripper_joints_name=[f"{side}_1", f"{side}_2"],
        arm_joints_name=[f"{side}_j{i}" for i in range(7)],
    )


class FakeEnv:
    """Just enough of EvalEnv for collect.record_episode: obs(t) holds t in every state entry."""

    def __init__(self, layout_file):
        robots = [dex1("right"), dex1("left")]
        self.num_envs, self.task_name, self.config_name, self.dt, self.eval_seed = 1, "t", "cfg", 1 / 600, 9
        self.current_env_seed_map = {0: 3}
        self.success, self.end_flag, self.task_success = [True], [False], [True]
        self.end_on_success, self.step_lim, self.t, self.actions = True, 600, 0, []
        self.robot_manager = SimpleNamespace(robot_list=robots, process_name=lambda n: n + "_joint_state")
        layout = {"Rigid": {"cup": [{"label": "cup", "default_pos": [0.1, -0.3, 0.7], "default_ori": [1, 0, 0, 0]}]}}
        self.seed_manager = SimpleNamespace(
            seed_info={3: {"scene_layout": str(layout_file)}}, get_seed_scene_info=lambda _: layout
        )

    def get_obs(self):
        state = {k: np.full(7 if "arm" in k else 1, float(self.t)) for k in KEYS}
        vision = {c: {"color": np.full((8, 8, 3), 10 * self.t, dtype=np.uint8)} for c in collect.CAMERAS}
        return dict(state=state, vision=vision, instruction="pour it")

    def take_action(self, action):
        self.actions.append(action)
        self.t += 1


class Plugin(ModuleType):
    """Accepts once three steps are in: the expert's two actions, then the held tail."""

    TASK, ACCEPTANCE, HOLD, STEP_LIM, TAIL = "t", "test_v1", 2, 50, 5

    def __init__(self):
        super().__init__("test_plugin")
        outer = self

        class Episode:
            def __init__(self, env):
                self.reject, self.steps = None, 0

            def actions(self):
                for i in (1, 2):
                    yield {k: np.full(7 if "arm" in k else 1, i * 0.1) for k in KEYS}

            def step(self):
                self.steps += 1
                return self.steps >= 3

            def privileged(self):
                return dict(steps=np.arange(self.steps))

            def summary(self):
                return dict(hold=outer.HOLD)

        self.Episode = Episode


class RecorderTests(unittest.TestCase):
    @needs_xpolicylab
    def test_record_episode_writes_aligned_standard_trajectory(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "layout.json").write_text("{}")
            env = FakeEnv(root / "layout.json")
            collect.record_episode(env, Plugin(), root, dict(env_cfg="cfg_600hz", fps=30, revision=None))
            quality = json.loads((root / "episode_000003.json").read_text())
            data = load_trajectory(str(root / "episode_000003.hdf5"), "RoboDojo", "v1.0")
        self.assertFalse(env.end_on_success)  # the recorder, not the task's check, ends the episode
        self.assertEqual((env.success, env.end_flag), ([True], [True]))
        self.assertTrue(quality["accepted"])
        self.assertEqual((quality["frames"], quality["stable_frames"]), (4, 2))
        self.assertEqual(quality["state_keys"], KEYS)  # left arm first, gripper after its arm
        self.assertEqual(quality["joint_names"]["left_arm_joint_state"][0], "left_j0")
        self.assertEqual(quality["planned_poses"], {"cup": [[0.1, -0.3, 0.7], [1, 0, 0, 0]]})
        self.assertEqual(data["instruction"], "pour it")
        self.assertEqual(data["additional_info"]["env_cfg"], "cfg_600hz")
        # frame t: observation before action t; the two expert actions, then the last one held
        np.testing.assert_array_equal(data["state"]["left_arm_joint_states"][:, 0], [0, 1, 2, 3])
        np.testing.assert_allclose(data["action"]["right_ee_joint_states"][:, 0], [0.1, 0.2, 0.2, 0.2])
        self.assertEqual(data["vision"]["cam_head"]["colors"].shape, (4, 8, 8, 3))
        self.assertEqual(int(data["vision"]["cam_left_wrist"]["colors"][2].mean().round()), 20)
        np.testing.assert_array_equal(data["privileged"]["steps"], np.arange(4))

    def test_export_takes_accepted_episodes_with_distinct_layouts(self):
        with tempfile.TemporaryDirectory() as directory:
            raw = Path(directory) / "seed100"
            raw.mkdir()
            base = dict(accepted=True, hold_frames=30, stable_frames=30, frames=90, task="t", plugin="p")
            base |= dict(acceptance_version="v", env_cfg="c", physics_dt=0.1, fps=30, cameras=[], state_keys=[])
            layouts = [[0.1, -0.3], [0.1, -0.3], [0.12, -0.3]]
            for i, (xy, accepted) in enumerate(zip(layouts, (True, True, False))):
                quality = dict(base, accepted=accepted, planned_poses={"cup": [[*xy, 0.7], [1, 0, 0, 0]]})
                (raw / f"episode_{i:06d}.json").write_text(json.dumps(quality))
            self.assertEqual([p.name for p, _ in export.select(raw.parent, 1)], ["episode_000000.hdf5"])
            with self.assertRaisesRegex(ValueError, "Only 1 accepted"):
                export.select(raw.parent, 2)
            (raw / "episode_000000.json").write_text(json.dumps(dict(base, stable_frames=12, planned_poses={})))
            with self.assertRaisesRegex(ValueError, "Inconsistent"):
                export.select(raw.parent, 1)

    def test_shards_are_marked_as_training_layouts(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "9").mkdir()
            files = [root / "9" / f"t_{i}.json" for i in range(5)]
            for i, path in enumerate(files):
                path.write_text(json.dumps({"i": i}))
            args = SimpleNamespace(env_cfg="cfg", candidates=9, task="t", shards="100-101", per_shard=2)
            args.output = root / "run"
            with (
                patch.object(run_batch, "layout_files", lambda cfg, seed, task: files),
                patch.object(run_batch, "layout_dir", lambda cfg, seed: root / str(seed)),
            ):
                self.assertEqual(run_batch.make_shards(args), files[:4])
            self.assertEqual(json.loads((root / "101/t_1.json").read_text()), {"i": 3})
            for seed in ("9", "100", "101"):
                self.assertEqual(json.loads((root / seed / "split.json").read_text())["split"], "train")
        self.assertEqual(run_batch.span("0-2,5"), [0, 1, 2, 5])

    def test_more_worker_slots_than_shards(self):
        slots = [(gpu, k) for k in range(3) for gpu in (0, 2)]
        queue = [100, 101, 102, 103]
        got = run_batch.assign(slots, {(0, 0): None}, queue)
        self.assertEqual(got, [((2, 0), 100), ((0, 1), 101), ((2, 1), 102), ((0, 2), 103)])
        self.assertEqual((queue, run_batch.assign(slots, {}, queue)), ([], []))


class ContractTests(unittest.TestCase):
    def obs(self, ee_state):
        manager = ObsManager(
            obs_config=dict(robot=dict(joint_states=True, world_ee_state=False, ee_state=ee_state)),
            num_envs=1,
            dt=1 / 300,
            task_name="t",
            description_cfg={},
        )
        robot = dex1("left")
        manager.camera_manager = manager.capture_manager = None
        manager.instruction = [""]
        manager.robot_manager = SimpleNamespace(
            robot_list=[robot],
            get_joint=lambda robot, env_idx_list: {0: np.zeros(7)},
            get_end_effector_real_val=lambda robot, env_idx_list: {0: np.array([0.0070, 0.0086])},
            restore_name=lambda key: key[: -len("_joint_state")],
            get_robot_by_gripper_name=lambda name: robot,
            control_manager=SimpleNamespace(prev_control=[{"left_ee_joint_state": {"position": [CLOSED, CLOSED]}}]),
        )
        return manager.get_obs(env_idx_list=[0])[0]

    def test_gripper_state_is_measured_when_configured(self):
        measured = self.obs("measured")
        # commanded closed (0), the fingers stopped on a ~4 cm object (mean joint 0.0078 m)
        np.testing.assert_allclose(measured["state"]["left_ee_joint_state"], [(CLOSED - 0.0078) / (CLOSED - OPEN)])
        self.assertEqual(measured["action"]["left_ee_joint_state"], [0.0])
        self.assertEqual(self.obs("command")["state"]["left_ee_joint_state"], [0.0])
        with self.assertRaisesRegex(ValueError, "ee_state"):
            self.obs("both")

    def test_eval_refuses_training_layouts_and_runs_selected_ids(self):
        with tempfile.TemporaryDirectory() as directory:
            folder = Path(directory, "Eval_Layout/RoboDojo/cfg/5")
            folder.mkdir(parents=True)
            for i in range(3):
                (folder / f"t_{i}.json").write_text("{}")
            (folder / "split.json").write_text('{"split": "train"}')
            config = dict(num_envs=1, task_name="t", config_name="cfg", seed=5)
            with patch.object(seed_manager, "ASSETS_PATH", directory):
                with self.assertRaisesRegex(ValueError, "training layouts"):
                    seed_manager.SeedManager(config).init_eval()
                manager = seed_manager.SeedManager(dict(config, allow_train_layouts=True, layout_ids=[2, 0]))
                manager.init_eval()
                self.assertEqual(manager.seed_list, [0, 2])
                with self.assertRaisesRegex(ValueError, r"\[7\]"):
                    seed_manager.SeedManager(dict(config, allow_train_layouts=True, layout_ids=[7])).init_eval()

    def test_measuring_cup_audit_recomputes_every_bean(self):
        from scripts.datagen.plugins import g1d_pour_beans_measuring_cups as plugin

        hold = plugin.HOLD
        cups = np.tile([[-0.15, 0, 0, 1, 0, 0, 0], [0.15, 0, 0, 1, 0, 0, 0]], (hold, 1, 1))
        beans = np.tile([[0.147, 0, 0], [0.147, 0.005, 0]], (hold, 1, 1))
        privileged = dict(
            initial_particles=np.array([[-0.153, 0, 0], [-0.153, 0.005, 0]]), particles=beans, cup_poses=cups
        )
        quality = dict(frames=hold, initial_counts=dict(total=2))
        self.assertEqual(plugin.audit(privileged, quality)["beans"], 2)
        beans[-1, 1] = [0.0, 0.0, 0.0]  # one bean on the table in the last frame
        with self.assertRaises(AssertionError):
            plugin.audit(privileged, quality)

    def test_cup_or_mug_audit_uses_the_target_cavity(self):
        from scripts.datagen.plugins import g1d_pour_beans_cup_or_mug as plugin
        from scripts.objects import measuring_cup_shape, plastic_mug_shape

        hold, n = plugin.HOLD, 360
        jitter = np.random.default_rng(0).uniform(-0.01, 0.01, (n, 3))
        # cavity centres in each cup's bbox frame (cups at x = -+0.15, upright)
        tall = np.array([0, 0, 0.06]) + measuring_cup_shape.origin_shift(0.17) + [-0.15, 0, 0]
        mug = np.array([0, 0, 0.05]) + plastic_mug_shape.origin_shift() + [0.15, 0, 0]
        cups = np.tile([[-0.15, 0, 0, 1, 0, 0, 0], [0.15, 0, 0, 1, 0, 0, 0]], (hold, 1, 1))
        beans = np.tile(mug + jitter, (hold, 1, 1))
        distractors = np.tile([[0.4, -0.1, 0, 1, 0, 0, 0], [-0.4, -0.1, 0, 1, 0, 0, 0]], (hold, 1, 1))
        privileged = dict(
            initial_particles=tall + jitter,
            particles=beans,
            cup_poses=cups,
            distractor_poses=distractors,
            distractor_poses_start=distractors[0].copy(),
        )
        cup_info = dict(
            source_cup=dict(kind="measuring_cup", height=0.17), target_cup=dict(kind="plastic_mug", height=0.1)
        )
        quality = dict(frames=hold, initial_counts=dict(total=n), cups=cup_info)
        report = plugin.audit(privileged, quality)
        self.assertEqual((report["beans"], report["target"]), (n, "plastic_mug"))

        def fails(**changes):
            with self.assertRaises(AssertionError):
                plugin.audit({k: changes.get(k, v) for k, v in privileged.items()}, quality)

        moved = distractors.copy()
        moved[3, 0, 0] += 0.03  # a distractor pushed 3 cm in one frame
        fails(distractor_poses=moved)
        touching = cups.copy()
        touching[3, 0, 0] = 0.07  # the tall cup's side inside the mug
        fails(cup_poses=touching)
        tilted = cups.copy()
        tilted[3, 1, 3:] = [np.cos(np.radians(10)), np.sin(np.radians(10)), 0, 0]  # target 20 deg over
        fails(cup_poses=tilted)
        stray = beans.copy()
        stray[-1, 0] = mug + [0.045, 0, 0]  # in the mug wall in the last frame
        fails(particles=stray)
        few = dict(privileged, initial_particles=privileged["initial_particles"][:250], particles=beans[:, :250])
        with self.assertRaises(AssertionError):
            plugin.audit(few, dict(quality, initial_counts=dict(total=250)))

    def test_registry_entries_are_complete(self):
        registry = json.loads(Path(__file__).with_name("registry.json").read_text())
        required = {
            "name",
            "status",
            "task",
            "env_cfg",
            "episodes",
            "format",
            "acceptance",
            "code",
            "layouts",
            "location",
        }
        names = [d["name"] for d in registry["datasets"]]
        self.assertEqual(len(names), len(set(names)))
        for dataset in registry["datasets"]:
            self.assertLessEqual(required, set(dataset), dataset["name"])
            self.assertIn(dataset["status"], ("released", "superseded", "incomplete"))


if __name__ == "__main__":
    unittest.main()
