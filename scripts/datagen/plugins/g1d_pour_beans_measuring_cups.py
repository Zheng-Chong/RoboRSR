"""scripts/datagen plugin for g1d_pour_beans_measuring_cups: the scripted bimanual pour of every red bean from the
tall measuring cup into the short one. An episode is accepted when every bean is inside the short cup's cavity, none
in the tall cup or outside, and both cups stand within UPRIGHT_DEG of upright for HOLD frames in a row (acceptance
all_beans_retained_v2, the strict check of scripts/g1d/collect_office_beans.py).
"""

import numpy as np

from scripts.g1d import scripted_demo as demo
from scripts.g1d.collect_office_beans import UPRIGHT_DEG, all_transferred, counts

TASK = "g1d_pour_beans_measuring_cups"
ACCEPTANCE = "all_beans_retained_v2"
STEP_LIM = 1200  # actions: pouring out every bean takes longer than the task's 600
TAIL = 60  # actions the last command is held after the script ends
HOLD = 30  # frames (1 s) the acceptance must hold
LABELS = ("source_cup", "target_cup")
COUNT_KEYS = ("total", "source", "target", "outside", "overlap")
ROUND_WALL, ROUND_BOTTOM = 0.002, 0.004  # m; a round target cup's wall and bottom (make_water_cup.py --gray)


def target_cup(lm):
    """None for the short measuring cup; else the round cavity (collect_office_beans.in_round_cavity) of the target
    cup: the layout entry's "cavity" when given (tapered cups), otherwise a straight cup of its layout size."""
    cfg = lm.get_scene_object(0, lm.get_instance_name(0, "target_cup")).instance_config
    if cfg["category"] == "measuring_cup":
        return None
    if cfg.get("cavity"):
        return dict(cfg["cavity"])
    diameter, _, height = cfg["physics"]["size"]
    return dict(height=height, radius=diameter / 2 - ROUND_WALL, bottom=ROUND_BOTTOM)


def upright(pose):
    return demo.from_wxyz(pose[3:]).as_matrix()[2, 2] > np.cos(np.deg2rad(UPRIGHT_DEG))


def pose_map(poses):
    """(2, 7) cup poses [x, y, z, qw, qx, qy, qz] -> {label: [position, wxyz]} for counts()."""
    return {label: [p[:3], p[3:]] for label, p in zip(LABELS, poses)}


class Episode:
    def __init__(self, env):
        self.env = env
        lm = self.lm = env.scene_manager.layout_manager
        self.fluid = lm.get_scene_object(0, lm.get_instance_name(0, "beans"))
        self.target = target_cup(lm)
        self.initial = self.particles()
        self.initial_counts = counts(self.initial, pose_map(self.poses()), self.target)
        self.final_counts = self.initial_counts
        ok = self.initial_counts["source"] == len(self.initial)
        self.reject = None if ok else "beans not all inside the source cup at the start"
        self.history = dict(particles=[], cup_poses=[], counts=[])

    def particles(self):
        points = np.asarray(self.fluid.get_particle_positions()[0], dtype=float)
        return demo.from_wxyz(self.fluid.init_ori).apply(points) + self.fluid.init_pos

    def poses(self):
        return np.array(
            [np.concatenate([demo.as_np(v) for v in self.lm.get_instance_pose(0, label=label)]) for label in LABELS]
        )

    def actions(self):
        return demo.cup_to_cup_actions(self.env, pour_share=1.0)

    def step(self):
        """After each action: record beans and cups; True while every bean is in the short cup, both upright."""
        points, poses = self.particles(), self.poses()
        self.final_counts = counts(points, pose_map(poses), self.target)
        self.history["particles"].append(points.astype(np.float32))
        self.history["cup_poses"].append(poses.astype(np.float32))
        self.history["counts"].append([self.final_counts[k] for k in COUNT_KEYS])
        return all_transferred(self.final_counts, len(self.initial)) and all(map(upright, poses))

    def privileged(self):
        """Per frame after its action: bean positions (T, N, 3), cup poses (T, 2, 7), counts (T, 5) in COUNT_KEYS."""
        return dict(
            initial_particles=self.initial.astype(np.float32), **{k: np.asarray(v) for k, v in self.history.items()}
        )

    def summary(self):
        return dict(
            initial_counts=self.initial_counts,
            final_counts=self.final_counts,
            upright_limit_degrees=UPRIGHT_DEG,
            **({"target_cup": self.target} if self.target else {}),
        )


def audit(privileged, quality):
    """Recompute the acceptance from the recorded beans and cups: every bean in the short cup and both cups upright
    in each of the last HOLD frames. Raises AssertionError on a violation; returns metrics for the report."""
    total = quality["initial_counts"]["total"]
    particles, poses = privileged["particles"], privileged["cup_poses"]
    assert particles.shape == (quality["frames"], total, 3) and poses.shape == (quality["frames"], 2, 7)
    target = quality.get("target_cup")
    start = counts(privileged["initial_particles"].astype(float), pose_map(poses[0]), target)
    assert start["source"] == total and start["outside"] == 0, start
    tilt = 0.0
    for points, frame in zip(particles[-HOLD:], poses[-HOLD:]):
        assert all_transferred(counts(points.astype(float), pose_map(frame), target), total)
        assert all(map(upright, frame))
        tilt = max(
            tilt, *(np.degrees(np.arccos(np.clip(demo.from_wxyz(p[3:]).as_matrix()[2, 2], -1, 1))) for p in frame)
        )
    return dict(beans=total, max_final_tilt_degrees=float(tilt))
