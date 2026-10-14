"""Scripted demos for the G1-D tasks: no policy server, privileged object poses.

Runs src/eval_client/main.py in-process with the policy swapped for a Cartesian script, tracked by damped
least-squares IK on the simulator's Jacobian. Videos and _result.json land in eval_result/ like a normal eval,
under policy name "scripted_demo".
    g1d_pour_water_into_cup   grasp the pitcher handle from the robot side, lift, carry it beside the cup, roll the
                              wrist until a quarter of the coffee has left the jug, roll back upright
    g1d_general_pickup        top-down grasp across the target's narrow side, lift 15 cm
    g1d_stack_blocks          stack the two outer blocks on the middle one, each with the arm on its side; arms back

Usage (sim env, repo root):
    CUDA_VISIBLE_DEVICES=1 PYTHONPATH=.:XPolicyLab EVAL_NUM=1 python scripts/g1d/scripted_demo.py \
        --env_cfg_type g1d --task_name g1d_stack_blocks [--seed 0]
"""

import itertools
from pathlib import Path
import runpy
import sys
import types

import numpy as np
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[2]
POLICY = "scripted_demo"

# Calibration knobs: metres, radians, actions (30 Hz)
PAD_REACH = 0.12  # dex1_base_link origin -> grip point on the pads (fingertips 2.3 cm further)
GRASP_IN_PITCHER = np.array([0.0617, 0.0, 0.059])  # handle bar centre (metadata functional.handle)
OPEN = 0.15  # 19 mm between the pads: the inner finger fits in the 26 mm slot between handle and body
BEHIND = np.array([0.0, -0.09, 0.0])  # fingertips clear the pitcher's back face; descend here, then slide in
SPOUT_IN_PITCHER = np.array([-0.0663, 0.0, 0.1145])  # spout lip, the tilt pivot
BBOX_HALF = np.array([0.0663, 0.0445, 0.1225])
POUR_BACKOFF = 0.05  # lip short of the cup centre: the stream leaves with forward speed
SPOUT_CLEAR, BODY_CLEAR = 0.03, 0.02  # above the cup rim
TILT = np.deg2rad(115)
TILT_RATE, RETURN_RATE = np.deg2rad(1.25), np.deg2rad(2.5)  # per action
POUR_TARGET = 0.25  # of the half-full jug (~130 ml, 2/3 of a mug); the stream in flight tops it up
MAX_HOLD = 48  # actions at full tilt
FLOW = 0.0033  # of the coffee per action (~50 ml/s): hold the tilt above this
DAMPING = 0.05
GRASP_ROT = Rotation.from_euler("z", 90, degrees=True)  # fingers forward (+y), wrist camera up: the G1-D zero pose
# Pick and place
# fingers 60 deg below horizontal, closing along x: reaches ~8 cm further at table height than straight down
TOP_ROT = Rotation.from_euler("x", -60, degrees=True) * GRASP_ROT
PAD_CLOSED, PAD_TRAVEL = 0.006, 0.089  # pad gap (m): 6 mm at grip=0 .. 95 mm at grip=1 (2 * 44.5 mm finger stroke)
GRASP_CLEAR = 0.025  # open the pads this much wider than the grasped side so neighbours are not swept in
FINGER_TIPS = 0.023  # grip point -> fingertips
TABLE_CLEAR = 0.006  # fingertips above the table
HOVER, LIFT = 0.10, 0.15
PLACE_GAP = 0.004  # held object's bottom above the surface it is released on


def pad_open(width):
    return float(np.clip((width + GRASP_CLEAR - PAD_CLOSED) / PAD_TRAVEL, 0.0, 1.0))


def as_np(x):
    return np.asarray(x.cpu() if hasattr(x, "cpu") else x, dtype=float).reshape(-1)


def from_wxyz(q):
    return Rotation.from_quat([q[1], q[2], q[3], q[0]])


def waypoints(p, rot, keys):
    """Smoothstep-interpolated (grasp point, hand rotation, gripper) per action through `keys`."""
    for p1, rot1, grip, n in keys:
        slerp = Slerp([0, 1], Rotation.concatenate([rot, rot1]))
        for i in range(1, n + 1):
            s = (i / n) ** 2 * (3 - 2 * i / n)
            yield p + (p1 - p) * s, slerp(s), grip
        p, rot = p1, rot1


class Arm:
    def __init__(self, env, robot):
        rm = env.robot_manager
        self.robot, self.key = robot, rm.robot_key[rm.robot_list.index(robot)]
        self.body = self.key.body_names.index(robot.ee_link_name)
        self.cols = list(robot.arm_joint_indices)
        self.origin = as_np(env.scene_manager.env_origins[0])
        self.arm_key, self.grip_key = rm.process_name(robot.arm_name), rm.process_name(robot.gripper_name)

    def q(self):
        return as_np(self.key.data.joint_pos[0, self.cols])

    def pose(self):
        link = as_np(self.key.data.body_link_pose_w[0, self.body])
        return link[:3] - self.origin, from_wxyz(link[3:])

    def grasp_point(self):
        pos, rot = self.pose()
        return pos + rot.apply([PAD_REACH, 0, 0])

    def ik(self, point, rot_t):
        pos, rot = self.pose()
        err = np.r_[point - rot_t.apply([PAD_REACH, 0, 0]) - pos, (rot_t * rot.inv()).as_rotvec()]
        jac = as_np(self.key.root_physx_view.get_jacobians()[0, self.body - 1][:, self.cols]).reshape(6, -1)
        return self.q() + jac.T @ np.linalg.solve(jac @ jac.T + DAMPING**2 * np.eye(6), err)


def pour_actions(env):
    lm = env.scene_manager.layout_manager
    pos, quat = lm.get_instance_pose(0, label="bottle")
    pitcher_pos, pitcher_rot = as_np(pos), from_wxyz(as_np(quat))
    cup = as_np(lm.get_instance_pose(0, label="cup")[0])
    fluid = lm.get_scene_object(0, lm.get_instance_name(0, "wine"))
    cup_top = cup[2] + lm.get_instance_bbox_vertices(lm.get_instance_name(0, "cup"), 0)[:, 2].max()

    arms = [Arm(env, r) for r in env.robot_manager.robot_list if r.type == "target"]
    arm = min(arms, key=lambda a: abs(a.grasp_point()[0] - pitcher_pos[0]))
    idle = {a.arm_key: a.q() for a in arms if a is not arm}

    side = np.sign(cup[0] - pitcher_pos[0])  # the spout faces the cup (build_asset.py)
    grasp = pitcher_pos + pitcher_rot.apply(GRASP_IN_PITCHER)
    corners = np.array(list(itertools.product(*zip(-BBOX_HALF, BBOX_HALF)))) - GRASP_IN_PITCHER

    def pour(theta):
        """Grasp point and hand rotation tilting the pitcher about its lip, held over the cup."""
        tilt = Rotation.from_rotvec([0, side * theta, 0])
        lip = (tilt * pitcher_rot).apply(SPOUT_IN_PITCHER - GRASP_IN_PITCHER)
        low = (tilt * pitcher_rot).apply(corners)[:, 2].min() - lip[2]
        target = [cup[0] - side * POUR_BACKOFF, cup[1], cup_top + max(SPOUT_CLEAR, BODY_CLEAR - low)]
        return target - lip, tilt * GRASP_ROT

    keys = [
        (grasp + BEHIND + [0, 0, 0.1], GRASP_ROT, OPEN, 42),
        (grasp + BEHIND, GRASP_ROT, OPEN, 30),
        (grasp + BEHIND, GRASP_ROT, OPEN, 12),  # settle
        (grasp, GRASP_ROT, OPEN, 48),
        (grasp, GRASP_ROT, 0, 18),  # close
        (np.r_[grasp[:2], pour(0)[0][2]], GRASP_ROT, 0, 36),  # lift
        (*pour(0), 0, 42),
    ]
    check = env.pour_check()

    def left_jug():
        """Fraction of the coffee outside the jug's bounding box (in the cup, in flight or spilled)."""
        jug_pos, jug_quat = lm.get_instance_pose(0, label="bottle")
        local = from_wxyz(as_np(jug_quat)).inv().apply(coffee() - as_np(jug_pos))
        return np.mean(np.any(np.abs(local) > BBOX_HALF, axis=1))

    def coffee():
        return np.asarray(fluid.get_particle_positions()[0]).reshape(-1, 3) + fluid.init_pos

    def act(point, rot, grip):
        nonlocal step
        if step % 10 == 0:
            jug_pos, jug_quat = lm.get_instance_pose(0, label="bottle")
            tilt = np.degrees(np.arccos(from_wxyz(as_np(jug_quat)).apply([0, 0, 1])[2]))
            fingers = as_np(arm.key.data.joint_pos[0, list(arm.robot.gripper_joint_indices)])
            print(
                f"[scripted_demo] {step:3d} err {np.linalg.norm(arm.grasp_point() - point):.3f}"
                f" fingers {fingers.round(4)} jug {as_np(jug_pos).round(3)} tilt {tilt:.0f}"
                f" left jug {left_jug():.0%} check {env.reward_manager.call_func_parser(check, 0):.0f}",
                flush=True,
            )
        step += 1
        action = {a.grip_key: [1.0] for a in arms} | {arm.grip_key: [float(grip)]} | idle
        action[arm.arm_key] = arm.ik(point, rot)
        return action

    print(f"[scripted_demo] jug {pitcher_pos.round(3)} cup {cup.round(3)} rim {cup_top:.3f} arm {arm.arm_key}")
    step = 0
    for target in waypoints(arm.grasp_point(), arm.pose()[1], keys):
        yield act(*target)
    theta, held, prev = 0.0, 0, 0.0  # tilt until enough coffee has left the jug, then back upright
    while (now := left_jug()) < POUR_TARGET and held < MAX_HOLD:
        if now - prev < FLOW:  # tilt further only while the stream is weak: a surge overshoots the cup
            theta = min(theta + TILT_RATE, TILT)
        held += theta == TILT
        prev = now
        yield act(*pour(theta), 0)
    while theta > 0:
        theta = max(theta - RETURN_RATE, 0.0)
        yield act(*pour(theta), 0)
    for _ in range(18):
        yield act(*pour(0), 0)


class Robot:
    """Both arms: one moves to Cartesian targets, the other holds its last command."""

    def __init__(self, env):
        self.env, self.lm = env, env.scene_manager.layout_manager
        self.arms = [Arm(env, r) for r in env.robot_manager.robot_list if r.type == "target"]
        self.hold = {a.arm_key: a.q() for a in self.arms}
        self.grip = {a.grip_key: [1.0] for a in self.arms}
        self.home = {a.arm_key: (a.grasp_point(), a.pose()[1]) for a in self.arms}
        self.step = 0

    def arm_for(self, x):
        return min(self.arms, key=lambda a: abs(self.home[a.arm_key][0][0] - x))

    def move(self, arm, keys):
        """Follow `keys` [(grip point, hand rotation, gripper, actions)] with `arm`."""
        for point, rot, grip in waypoints(arm.grasp_point(), arm.pose()[1], keys):
            self.hold[arm.arm_key] = arm.ik(point, rot)
            self.grip[arm.grip_key] = [float(grip)]
            if self.step % 20 == 0:
                err = np.linalg.norm(arm.grasp_point() - point)
                print(f"[scripted_demo] {self.step:3d} {arm.arm_key} err {err:.3f}", flush=True)
            self.step += 1
            yield dict(self.hold) | self.grip

    def object(self, label):
        """World centre, rotation, local bbox extents and world z bounds of an object."""
        pos, quat = self.lm.get_instance_pose(0, label=label)
        rot = from_wxyz(as_np(quat))
        bbox = self.lm.get_instance_bbox_vertices(self.lm.get_instance_name(0, label), 0)
        lo, hi = bbox.min(0), bbox.max(0)
        z = (as_np(pos) + rot.apply(bbox))[:, 2]
        return as_np(pos) + rot.apply((lo + hi) / 2), rot, hi - lo, z.min(), z.max()

    def pick(self, label):
        """Top-down grasp across the narrow horizontal side, lifted; returns (arm, offset, grip_height, open_cmd)."""
        centre, rot, ext, bottom, top = self.object(label)
        axes = rot.as_matrix()  # columns are the object's local axes in world frame
        up = int(np.argmax(np.abs(axes[2])))
        narrow = min((a for a in range(3) if a != up), key=lambda a: ext[a])
        yaw = np.arctan2(axes[1, narrow], axes[0, narrow])  # close along the narrow horizontal local axis
        yaw = (yaw + np.pi / 2) % np.pi - np.pi / 2  # the jaw is symmetric: keep the wrist camera facing forward
        hand = Rotation.from_euler("z", yaw) * TOP_ROT
        open_cmd = pad_open(ext[narrow])
        grip = np.r_[centre[:2], max(top - 0.03, bottom + FINGER_TIPS + TABLE_CLEAR)]
        arm = self.arm_for(centre[0])
        print(
            f"[scripted_demo] pick {label} at {centre.round(3)} size {ext.round(3)}"
            f" width {ext[narrow]:.3f} open {open_cmd:.2f} with {arm.arm_key}"
        )
        yield from self.move(
            arm,
            [
                (grip + [0, 0, HOVER], hand, open_cmd, 54),
                (grip, hand, open_cmd, 30),
                (grip, hand, 0, 18),  # close
                (grip + [0, 0, LIFT], hand, 0, 30),
            ],
        )
        centre, _, _, _, _ = self.object(label)
        fingers = as_np(arm.key.data.joint_pos[0, list(arm.robot.gripper_joint_indices)])
        print(f"[scripted_demo] lifted {label} to {centre.round(3)} (grip {grip.round(3)}), fingers {fingers.round(4)}")
        return arm, centre - arm.grasp_point(), grip[2] - bottom, open_cmd

    def place(self, arm, offset, grip_height, open_cmd, xy, surface):
        """Lower the held object onto `surface` (z) at `xy`, release to `open_cmd` and back off upward."""
        hand = arm.pose()[1]
        point = np.r_[np.asarray(xy) - offset[:2], surface + grip_height + PLACE_GAP]
        print(f"[scripted_demo] place at {point.round(3)} on surface {surface:.3f}")
        yield from self.move(
            arm,
            [
                (point + [0, 0, HOVER], hand, 0, 48),
                (point, hand, 0, 30),
                (point, hand, open_cmd, 12),  # release
                (point + [0, 0, HOVER], hand, open_cmd, 24),
            ],
        )

    def go_home(self):
        for arm in self.arms:
            point, rot = self.home[arm.arm_key]
            yield from self.move(arm, [(point, rot, 1, 54), (point, rot, 1, 12)])


def pickup_actions(env):
    robot = Robot(env)
    arm, _, _, _ = yield from robot.pick("target")
    point, rot = arm.grasp_point(), arm.pose()[1]
    yield from robot.move(arm, [(point, rot, 0, 36)])  # hold it up


def stack_actions(env):
    robot = Robot(env)
    labels = ["block_0", "block_1", "block_2"]
    base, *others = sorted(labels, key=lambda label: abs(robot.object(label)[0][0]))
    stack = [base]
    for label in others:
        arm, offset, grip_height, open_cmd = yield from robot.pick(label)
        top = max(robot.object(b)[4] for b in stack)
        yield from robot.place(arm, offset, grip_height, open_cmd, robot.object(stack[-1])[0][:2], top)
        stack.append(label)
    yield from robot.go_home()


DEMOS = {
    "g1d_pour_water_into_cup": pour_actions,
    "g1d_general_pickup": pickup_actions,
    "g1d_stack_blocks": stack_actions,
}


def eval_one_episode(TASK_ENV, model_client):
    TASK_ENV.get_obs()  # initial frame (also triggers the pre-action reward check so is_lift records table height)
    for action in DEMOS[TASK_ENV.task_name](TASK_ENV):
        TASK_ENV.take_action(action)
        if TASK_ENV.is_episode_end():
            break
        TASK_ENV.get_obs()  # streams the camera frames into the episode videos


class NoServer:
    def __init__(self, *args, **kwargs):
        pass

    def call(self, func_name, **kwargs):
        return None


def main():
    import client_server.ws.model_client as ws

    ws.WsModelClient = NoServer
    package = types.ModuleType(f"XPolicyLab.policy.{POLICY}")
    deploy = types.ModuleType(f"XPolicyLab.policy.{POLICY}.deploy")
    deploy.eval_one_episode = eval_one_episode
    sys.modules |= {package.__name__: package, deploy.__name__: deploy}

    args = sys.argv[1:]
    defaults = {"--task_name": "g1d_pour_water_into_cup", "--seed": "0", "--device_id": "0"}
    args += [x for k, v in defaults.items() if k not in args for x in (k, v)]
    sys.argv = [str(ROOT / "src/eval_client/main.py"), *args, "--policy_name", POLICY, "--port", "0"]
    sys.argv += ["--additional_info", "scripted", "--headless", "--enable_cameras"]
    sys.argv += ["--kit_args", "--enable isaacsim.replicator.behavior --enable isaacsim.sensors.camera"]
    runpy.run_path(sys.argv[0], run_name="__main__")


if __name__ == "__main__":
    main()
