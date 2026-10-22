"""scripts/datagen plugin for g1d_pour_beans_cup_or_mug: the scripted bimanual pour of every red bean (300-600 per
layout) from the tall measuring cup into the short measuring cup or the white mug, among Clutter distractors.

Acceptance all_beans_retained_any_target_v2:
- the last HOLD frames: every bean inside the target's actual cavity, none in the tall cup or outside, both cups
  within UPRIGHT_DEG of upright (a cup that turned in the grip stays tilted after the pour);
- the whole episode: the two cups' bodies never touch (the tall cup's surface sampled against the target's solid and
  the mug handle), the target never tilts past TARGET_TILT_DEG (it is only carried, never turned), and no distractor
  moves more than DISTRACTOR_SHIFT_M (nothing bumps them).
The episode ends at the first whole-episode violation (env.success is cleared), with the reason in the quality file.
"""

import numpy as np

from scripts.datagen.plugins import g1d_pour_beans_measuring_cups as base
from scripts.g1d import scripted_demo as demo
from scripts.objects import measuring_cup_shape, plastic_mug_shape

TASK = "g1d_pour_beans_cup_or_mug"
ACCEPTANCE = "all_beans_retained_any_target_v2"
STEP_LIM, TAIL, HOLD = base.STEP_LIM, base.TAIL, base.HOLD
LABELS, COUNT_KEYS = base.LABELS, base.COUNT_KEYS
BEANS = (300, 600)  # beans per layout
KINDS = ("measuring_cup", "plastic_mug")
UPRIGHT_DEG = 15  # both cups at the end
TARGET_TILT_DEG = 15  # the target, all along
DISTRACTOR_SHIFT_M = 0.02
RADIUS = {"measuring_cup": 0.040, "plastic_mug": 0.042}  # body cylinders for the contact check (m)
_ANGLES = np.linspace(0, 2 * np.pi, 24, endpoint=False)


def tilt_deg(pose):
    return float(np.degrees(np.arccos(np.clip(demo.from_wxyz(pose[3:]).as_matrix()[2, 2], -1, 1))))


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


def cups_touch(poses, cups):
    """True when the tall cup's body (cylinder side and bottom, sampled) enters the target's body or mug handle."""
    (s_kind, s_h), (t_kind, t_h) = (cups[label] for label in LABELS)
    r = RADIUS[s_kind]
    side = [(r * np.cos(a), r * np.sin(a), z) for z in np.linspace(-s_h / 2, s_h / 2, 9) for a in _ANGLES]
    bottom = [(f * r * np.cos(a), f * r * np.sin(a), -s_h / 2) for f in (0.3, 0.7) for a in _ANGLES]
    source, target = poses
    world = demo.from_wxyz(source[3:]).apply(np.array(side + bottom)) + source[:3]
    p = demo.from_wxyz(target[3:]).inv().apply(world - target[:3])  # target bbox frame
    if t_kind == "measuring_cup":
        return bool(((np.hypot(p[:, 0], p[:, 1]) < RADIUS[t_kind]) & (np.abs(p[:, 2]) < t_h / 2)).any())
    p = p - plastic_mug_shape.origin_shift()  # mug frame: bottom centre at the origin, handle on +X
    g = plastic_mug_shape.MUG
    body = (np.hypot(p[:, 0], p[:, 1]) < RADIUS[t_kind]) & (p[:, 2] > 0) & (p[:, 2] < t_h)
    bar = (np.abs(p[:, 0] - g["R1"] - g["HANDLE_OUT"]) < 0.012) & (np.abs(p[:, 1]) < g["HANDLE_W"] / 2 + 0.003)
    return bool((body | bar & (p[:, 2] > g["HANDLE_BOT"]) & (p[:, 2] < g["HANDLE_TOP"])).any())


def violation(poses, cups, distractors, distractors_start):
    """Whole-episode check of one frame: the reason it fails, or None."""
    if cups_touch(poses, cups):
        return "cups touched"
    if tilt_deg(poses[1]) > TARGET_TILT_DEG:
        return f"target tilted past {TARGET_TILT_DEG} deg"
    if (
        len(distractors)
        and np.linalg.norm(distractors[:, :3] - distractors_start[:, :3], axis=1).max() > DISTRACTOR_SHIFT_M
    ):
        return f"a distractor moved more than {DISTRACTOR_SHIFT_M} m"
    return None


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
        self.violation = None
        self.history = dict(particles=[], cup_poses=[], counts=[], distractor_poses=[])

    def distractor_poses(self):
        poses = [self.lm.get_instance_pose(0, label=label) for label in self.distractors]
        return np.array([np.concatenate([demo.as_np(v) for v in pose]) for pose in poses]).reshape(-1, 7)

    def step(self):
        """After each action: record beans, cups and distractors; end the episode at a whole-episode violation; True
        while every bean is in the target cup and both cups are upright."""
        points, poses, distractors = self.particles(), self.poses(), self.distractor_poses()
        self.final_counts = counts(points, poses, self.cups)
        self.history["particles"].append(points.astype(np.float32))
        self.history["cup_poses"].append(poses.astype(np.float32))
        self.history["counts"].append([self.final_counts[k] for k in COUNT_KEYS])
        self.history["distractor_poses"].append(distractors.astype(np.float32))
        reason = violation(poses, self.cups, distractors, self.initial_distractors)
        if reason:
            self.violation = f"{reason} at frame {len(self.history['counts']) - 1}"
            self.env.success[0] = False  # collect.py stops and rejects the episode
            return False
        upright = all(tilt_deg(p) <= UPRIGHT_DEG for p in poses)
        return all_transferred(self.final_counts, len(self.initial)) and upright

    def privileged(self):
        """Per frame after its action: beans (T, N, 3), cup poses (T, 2, 7), counts (T, 5), distractor poses (T, D, 7);
        the initial beans and distractor poses."""
        arrays = {k: np.asarray(v) for k, v in self.history.items()}
        arrays["distractor_poses"] = arrays["distractor_poses"].reshape(-1, len(self.distractors), 7)
        return dict(
            initial_particles=self.initial.astype(np.float32),
            distractor_poses_start=self.initial_distractors.astype(np.float32),
            **arrays,
        )

    def summary(self):
        poses = np.asarray(self.history["cup_poses"]).reshape(-1, 2, 7)
        return dict(
            initial_counts=self.initial_counts,
            final_counts=self.final_counts,
            upright_limit_degrees=UPRIGHT_DEG,
            target_tilt_limit_degrees=TARGET_TILT_DEG,
            cups={label: dict(kind=k, height=h) for label, (k, h) in self.cups.items()},
            beans=len(self.initial),
            distractors=self.distractors,
            violation=self.violation,
            max_target_tilt_degrees=max((tilt_deg(p[1]) for p in poses), default=0.0),
        )


def audit(privileged, quality):
    """Recompute the acceptance from the recorded data: beans and uprightness in the last HOLD frames, and the
    whole-episode checks in every frame. Raises AssertionError on a violation; returns metrics."""
    total = quality["initial_counts"]["total"]
    cups = {label: (c["kind"], c["height"]) for label, c in quality["cups"].items()}
    particles, poses = privileged["particles"], privileged["cup_poses"]
    distractors, start_d = privileged["distractor_poses"], privileged["distractor_poses_start"]
    assert BEANS[0] <= total <= BEANS[1], total
    assert particles.shape == (quality["frames"], total, 3) and poses.shape == (quality["frames"], 2, 7)
    assert distractors.shape[:1] == (quality["frames"],), distractors.shape
    start = counts(privileged["initial_particles"], poses[0], cups)
    assert start["source"] == total and start["outside"] == 0, start
    for frame, moved in zip(poses, distractors):
        assert violation(frame, cups, moved, start_d) is None, violation(frame, cups, moved, start_d)
    for points, frame in zip(particles[-HOLD:], poses[-HOLD:]):
        assert all_transferred(counts(points, frame, cups), total)
        assert all(tilt_deg(p) <= UPRIGHT_DEG for p in frame)
    shift = np.linalg.norm(distractors[..., :3] - start_d[:, :3], axis=-1) if start_d.size else np.zeros(1)
    return dict(
        beans=total,
        target=cups["target_cup"][0],
        max_final_tilt_degrees=max(tilt_deg(p) for frame in poses[-HOLD:] for p in frame),
        max_target_tilt_degrees=max(tilt_deg(frame[1]) for frame in poses),
        distractors=int(start_d.shape[0]),
        distractor_max_shift_m=float(shift.max()),
    )
