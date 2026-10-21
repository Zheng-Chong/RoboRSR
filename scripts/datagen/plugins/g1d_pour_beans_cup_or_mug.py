"""scripts/datagen plugin for g1d_pour_beans_cup_or_mug: the scripted bimanual pour of every red bean (100-300 per
layout) from the tall measuring cup into the short measuring cup or the white mug, among Clutter distractors. An
episode is accepted when every bean is inside the target's actual cavity, none in the tall cup or outside, and both
cups stand within UPRIGHT_DEG of upright for HOLD frames in a row (acceptance all_beans_retained_any_target_v1: the
measuring-cup plugin's all_beans_retained_v2 with the target's own cavity). Distractor poses are recorded for the
audit report only; a nudged distractor does not fail an episode.
"""

import numpy as np

from scripts.datagen.plugins import g1d_pour_beans_measuring_cups as base
from scripts.g1d import scripted_demo as demo
from scripts.objects import measuring_cup_shape, plastic_mug_shape

TASK = "g1d_pour_beans_cup_or_mug"
ACCEPTANCE = "all_beans_retained_any_target_v1"
STEP_LIM, TAIL, HOLD = base.STEP_LIM, base.TAIL, base.HOLD
UPRIGHT_DEG = base.UPRIGHT_DEG
LABELS, COUNT_KEYS = base.LABELS, base.COUNT_KEYS
BEANS = (100, 300)  # beans per layout
KINDS = ("measuring_cup", "plastic_mug")


def in_cup(points, pose, kind, height):
    """Points (P, 3) inside the cup's actual cavity; pose [x, y, z, qw, qx, qy, qz] of its bbox centre."""
    local = demo.from_wxyz(pose[3:]).inv().apply(np.asarray(points, float) - pose[:3])
    if kind == "measuring_cup":
        return measuring_cup_shape.in_cavity(local - measuring_cup_shape.origin_shift(height), height)
    if kind == "plastic_mug":
        return plastic_mug_shape.in_cavity(local - plastic_mug_shape.origin_shift())
    raise ValueError(f"No cavity for {kind}")


def counts(points, poses, cups):
    """Beans per place; cups = {label: (kind, height)} in LABELS order, poses (2, 7) in the same order."""
    points = np.asarray(points, float)
    if points.ndim != 2 or points.shape[1] != 3 or not len(points) or not np.isfinite(points).all():
        raise ValueError("Invalid or missing bean positions")
    source, target = (in_cup(points, pose, *cups[label]) for label, pose in zip(LABELS, poses))
    return dict(
        total=len(points),
        source=int(source.sum()),
        target=int(target.sum()),
        outside=int((~(source | target)).sum()),
        overlap=int((source & target).sum()),
    )


def all_transferred(result, total):
    return total > 0 and result == dict(total=total, source=0, target=total, outside=0, overlap=0)


class Episode(base.Episode):
    def __init__(self, env):
        self.env = env
        lm = self.lm = env.scene_manager.layout_manager
        self.fluid = lm.get_scene_object(0, lm.get_instance_name(0, "beans"))
        self.cups = {}
        for label in LABELS:
            name = lm.get_instance_name(0, label)
            kind = next((k for k in KINDS if name.startswith(k)), None)
            bbox = lm.get_instance_bbox_vertices(name, 0)
            self.cups[label] = (kind, float(bbox[:, 2].max() - bbox[:, 2].min()))
        self.distractors = sorted(lm.get_labels_by_prefix(0, "distractor"))
        self.initial = self.particles()
        total = len(self.initial)
        self.reject = None
        if self.cups["source_cup"][0] != "measuring_cup" or self.cups["target_cup"][0] not in KINDS:
            self.reject = f"unexpected cups {self.cups}"
        elif not BEANS[0] <= total <= BEANS[1]:
            self.reject = f"{total} beans, outside {BEANS}"
        self.initial_counts = self.final_counts = counts(self.initial, self.poses(), self.cups)
        if not self.reject and self.initial_counts["source"] != total:
            self.reject = "beans not all inside the source cup at the start"
        self.initial_distractors = self.distractor_poses()
        self.history = dict(particles=[], cup_poses=[], counts=[])

    def distractor_poses(self):
        poses = [self.lm.get_instance_pose(0, label=label) for label in self.distractors]
        return np.array([np.concatenate([demo.as_np(v) for v in pose]) for pose in poses]).reshape(-1, 7)

    def step(self):
        """After each action: record beans and cups; True while every bean is in the target cup, both upright."""
        points, poses = self.particles(), self.poses()
        self.final_counts = counts(points, poses, self.cups)
        self.history["particles"].append(points.astype(np.float32))
        self.history["cup_poses"].append(poses.astype(np.float32))
        self.history["counts"].append([self.final_counts[k] for k in COUNT_KEYS])
        return all_transferred(self.final_counts, len(self.initial)) and all(map(base.upright, poses))

    def privileged(self):
        """base.Episode's arrays plus distractor poses at the start and the end (D, 7)."""
        return dict(
            super().privileged(),
            distractor_poses_start=self.initial_distractors.astype(np.float32),
            distractor_poses_end=self.distractor_poses().astype(np.float32),
        )

    def summary(self):
        moved = np.linalg.norm(self.distractor_poses()[:, :3] - self.initial_distractors[:, :3], axis=1)
        return dict(
            initial_counts=self.initial_counts,
            final_counts=self.final_counts,
            upright_limit_degrees=UPRIGHT_DEG,
            cups={label: dict(kind=k, height=h) for label, (k, h) in self.cups.items()},
            beans=len(self.initial),
            distractors=self.distractors,
            distractor_max_shift_m=float(moved.max()) if len(moved) else 0.0,
        )


def audit(privileged, quality):
    """Recompute the acceptance from the recorded beans and cups with each cup's cavity: every bean in the target and
    both cups upright in each of the last HOLD frames. Raises AssertionError on a violation; returns metrics."""
    total = quality["initial_counts"]["total"]
    cups = {label: (c["kind"], c["height"]) for label, c in quality["cups"].items()}
    particles, poses = privileged["particles"], privileged["cup_poses"]
    assert BEANS[0] <= total <= BEANS[1], total
    assert particles.shape == (quality["frames"], total, 3) and poses.shape == (quality["frames"], 2, 7)
    start = counts(privileged["initial_particles"], poses[0], cups)
    assert start["source"] == total and start["outside"] == 0, start
    tilt = 0.0
    for points, frame in zip(particles[-HOLD:], poses[-HOLD:]):
        assert all_transferred(counts(points, frame, cups), total)
        assert all(map(base.upright, frame))
        tilt = max(
            tilt, *(np.degrees(np.arccos(np.clip(demo.from_wxyz(p[3:]).as_matrix()[2, 2], -1, 1))) for p in frame)
        )
    start_d, end_d = privileged["distractor_poses_start"], privileged["distractor_poses_end"]
    shift = np.linalg.norm(end_d[:, :3] - start_d[:, :3], axis=1)
    return dict(
        beans=total,
        target=cups["target_cup"][0],
        max_final_tilt_degrees=float(tilt),
        distractors=int(len(shift)),
        distractor_max_shift_m=float(shift.max()) if len(shift) else 0.0,
    )
