"""Scripted demos for the G1-D tasks: no policy server, privileged object poses.

Runs src/eval_client/main.py in-process with the policy swapped for a Cartesian script, tracked by damped
least-squares IK on the simulator's Jacobian. Videos and _result.json land in eval_result/ like a normal eval,
under policy name "scripted_demo".
    g1d_pour_water_into_cup   bimanual: one arm grasps the pitcher handle from the robot side, the other picks the cup
                              from the outside and holds it up between them; the jug rolls about its lip until a
                              quarter of the beans have left it, the actual spout steered over the cup; back upright
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
TILT = np.deg2rad(125)
TILT_RATE, RETURN_RATE = np.deg2rad(2.0), np.deg2rad(4.0)  # per action
TILT_DRY, DRY_RATE = np.deg2rad(80), np.deg2rad(4.0)  # nothing leaves the jug below this: tilt faster
JUG_SPEED = 0.0035  # m per action carrying the jug: faster swings it about the handle and it slips out
POUR_TARGET = 0.25  # of the half-full jug (~130 ml, 2/3 of a mug); the stream in flight tops it up
FLOW = 0.0033  # of the beans per action (~50 ml/s): hold the tilt above this
STEP_MARGIN = 12  # actions kept free after the jug is back upright, for the rising-edge success check
# Second arm holding the cup
CUP_LIFT = 0.04  # cup bottom above the table while held under the spout
CUP_MEET = 0.0  # the cup is carried this much toward the jug (the cup arm is near the edge of its reach)
SPOUT_GAIN, SPOUT_SLACK = 0.2, 0.05  # per action, share of the spout error (actual vs over the actual cup) fed back
CUP_GRIP_DOWN = 0.03  # pads this far below the rim
CUP_YAW = 60  # deg, wrist turned inward from fingers-forward
DAMPING = 0.05
GRASP_ROT = Rotation.from_euler("z", 90, degrees=True)  # fingers forward (+y), wrist camera up: the G1-D zero pose
# Pick and place
# fingers 60 deg below horizontal, closing along x: reaches ~8 cm further at table height than straight down
TOP_ROT = Rotation.from_euler("x", -60, degrees=True) * GRASP_ROT
PAD_CLOSED, PAD_TRAVEL = 0.006, 0.089  # pad gap (m): 6 mm at grip=0 .. 95 mm at grip=1 (2 * 44.5 mm finger stroke)
GRASP_CLEAR = 0.025  # open the pads this much wider than the grasped side so neighbours are not swept in
FINGER_TIPS = 0.023  # grip point -> fingertips
TABLE_CLEAR = 0.006  # fingertips above the table
HOVER, LIFT = 0.06, 0.15  # 6 cm hover keeps 3-block stacks below the wrist-pitch limit; pickup lifts 15 cm
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
        r = rot.apply([PAD_REACH, 0, 0])
        err = np.r_[point - (pos + r), 0.3 * (rot_t * rot.inv()).as_rotvec()]
        jac = as_np(self.key.root_physx_view.get_jacobians()[0, self.body - 1][:, self.cols]).reshape(6, -1)
        jac = np.r_[jac[:3] - np.cross(r, jac[3:].T).T, 0.3 * jac[3:]]
        q = self.q()
        lim = as_np(self.key.data.soft_joint_pos_limits[0, self.cols]).reshape(-1, 2) + [0.025, -0.025]
        dq = jac.T @ np.linalg.solve(jac @ jac.T + 0.02**2 * np.eye(6), err)
        jac[:, (q + dq <= lim[:, 0]) & (dq < 0) | (q + dq >= lim[:, 1]) & (dq > 0)] = 0.0
        return np.clip(q + jac.T @ np.linalg.solve(jac @ jac.T + 0.02**2 * np.eye(6), err), lim[:, 0], lim[:, 1])


def cup_grasp(lm, out):
    """Grasp of the cup body below the rim by the arm on side `out` (+-1 in x): fingers TOP_ROT steep, wrist turned
    CUP_YAW inward, pads closing across the body; the steep fingers pass above a mug handle.

    Returns the grasp point (on the body axis), hand rotation, rim height, body radius and gripper opening.
    """
    pos, quat = lm.get_instance_pose(0, label="cup")
    pos, rot = as_np(pos), from_wxyz(as_np(quat))
    meta = lm.get_instance_metadata(0, label="cup") or {}
    bbox = lm.get_instance_bbox_vertices(lm.get_instance_name(0, "cup"), 0)
    lo, hi = bbox.min(0), bbox.max(0)
    circle = meta.get("active", {}).get("place", {}).get("up", {}).get("contact_circle", {})
    axis = pos + rot.apply(np.r_[circle.get("center", (lo + hi) / 2)[:2], 0])
    rim, radius = pos[2] + hi[2], (hi[1] - lo[1]) / 2
    grip = np.r_[axis[:2], max(rim - CUP_GRIP_DOWN, pos[2] + lo[2] + FINGER_TIPS + TABLE_CLEAR)]
    hand = Rotation.from_euler("z", out * CUP_YAW, degrees=True) * TOP_ROT
    return grip, hand, rim, radius, pad_open(2 * radius)


def pour_actions(env):
    lm = env.scene_manager.layout_manager
    pos, quat = lm.get_instance_pose(0, label="bottle")
    pitcher_pos, pitcher_rot = as_np(pos), from_wxyz(as_np(quat))
    fluid = lm.get_scene_object(0, lm.get_instance_name(0, "wine"))

    arms = [Arm(env, r) for r in env.robot_manager.robot_list if r.type == "target"]
    arm = min(arms, key=lambda a: abs(a.grasp_point()[0] - pitcher_pos[0]))
    cup_arm = next(a for a in arms if a is not arm)
    cup0 = as_np(lm.get_instance_pose(0, label="cup")[0])
    side = np.sign(cup0[0] - pitcher_pos[0])  # the spout faces the cup (build_asset.py)
    cup_grip, cup_hand, rim0, cup_r, cup_open = cup_grasp(lm, side)  # the cup arm is beyond the cup
    held_grip = cup_grip + [-side * CUP_MEET, 0, CUP_LIFT]
    meet = cup_grip[:2] + [-side * CUP_MEET, 0]
    rim = rim0 + CUP_LIFT

    grasp = pitcher_pos + pitcher_rot.apply(GRASP_IN_PITCHER)
    corners = np.array(list(itertools.product(*zip(-BBOX_HALF, BBOX_HALF)))) - GRASP_IN_PITCHER

    def pour(theta, centre=meet, top=rim):
        """Grasp point and hand rotation tilting the pitcher about its lip, held over the cup; and the lip target."""
        tilt = Rotation.from_rotvec([0, side * theta, 0])
        lip = (tilt * pitcher_rot).apply(SPOUT_IN_PITCHER - GRASP_IN_PITCHER)
        low = (tilt * pitcher_rot).apply(corners)[:, 2].min() - lip[2]
        target = np.array([centre[0] - side * POUR_BACKOFF, centre[1], top + max(SPOUT_CLEAR, BODY_CLEAR - low)])
        return target - lip, tilt * GRASP_ROT, target

    start = pour(0)[0]
    mid = np.r_[grasp[:2], start[2]]  # straight up first: pulled sideways while standing, the jug tips over

    def carry(a, b):
        return max(int(np.ceil(np.linalg.norm(b - a) / JUG_SPEED)), 12)

    jug_keys = [
        (grasp + BEHIND + [0, 0, 0.1], GRASP_ROT, OPEN, 30),
        (grasp + BEHIND, GRASP_ROT, OPEN, 20),
        (grasp + BEHIND, GRASP_ROT, OPEN, 8),  # settle
        (grasp, GRASP_ROT, OPEN, 48),
        (grasp, GRASP_ROT, 0, 18),  # close
        (mid, GRASP_ROT, 0, carry(grasp, mid)),
        (start, GRASP_ROT, 0, carry(mid, start)),
    ]
    cup_keys = [
        (cup_grip + [0, 0, HOVER], cup_hand, cup_open, 54),
        (cup_grip, cup_hand, cup_open, 30),
        (cup_grip, cup_hand, 0, 15),  # close
        (cup_grip + [0, 0, CUP_LIFT], cup_hand, 0, 18),
        (held_grip, cup_hand, 0, 20),  # carry toward the jug
    ]
    check = env.pour_check()

    def beans():
        return np.asarray(fluid.get_particle_positions()[0]).reshape(-1, 3) + fluid.init_pos

    def jug():
        jug_pos, jug_quat = lm.get_instance_pose(0, label="bottle")
        return as_np(jug_pos), from_wxyz(as_np(jug_quat))

    def left_jug():
        """Fraction of the beans outside the jug's bounding box (in the cup, in flight or spilled)."""
        jug_pos, jug_rot = jug()
        return np.mean(np.any(np.abs(jug_rot.inv().apply(beans() - jug_pos)) > BBOX_HALF, axis=1))

    axis_off = from_wxyz(as_np(lm.get_instance_pose(0, label="cup")[1])).inv().apply(cup_grip - cup0) * [1, 1, 0]

    def in_cup():
        """Fraction of the beans inside the cup's rim cylinder, for the log."""
        pos, quat = lm.get_instance_pose(0, label="cup")
        pos, rot = as_np(pos), from_wxyz(as_np(quat))
        local = rot.inv().apply(beans() - pos) - axis_off
        return np.mean((np.hypot(local[:, 0], local[:, 1]) < cup_r) & (local[:, 2] < rim0 - cup0[2]))

    def act(jug_target, cup_target):
        nonlocal step
        if step % 10 == 0:
            jug_pos, jug_rot = jug()
            tilt = np.degrees(np.arccos(jug_rot.apply([0, 0, 1])[2]))
            print(
                f"[scripted_demo] {step:3d} err jug {np.linalg.norm(arm.grasp_point() - jug_target[0]):.3f}"
                f" cup {np.linalg.norm(cup_arm.grasp_point() - cup_target[0]):.3f} jug {jug_pos.round(3)}"
                f" tilt {tilt:.0f} left jug {left_jug():.0%} in cup {in_cup():.0%}"
                f" check {env.reward_manager.call_func_parser(check, 0):.0f}",
                flush=True,
            )
        step += 1
        return {
            arm.arm_key: arm.ik(*jug_target[:2]),
            arm.grip_key: [float(jug_target[2])],
            cup_arm.arm_key: cup_arm.ik(*cup_target[:2]),
            cup_arm.grip_key: [float(cup_target[2])],
        }

    print(
        f"[scripted_demo] jug {pitcher_pos.round(3)} cup {cup0.round(3)} rim {rim0:.3f} jug arm {arm.arm_key}"
        f" cup arm {cup_arm.arm_key} grip {cup_grip.round(3)} open {cup_open:.2f}"
    )
    step = 0
    jug_path = list(waypoints(arm.grasp_point(), arm.pose()[1], jug_keys))
    cup_path = list(waypoints(cup_arm.grasp_point(), cup_arm.pose()[1], cup_keys))
    for j, c in itertools.zip_longest(jug_path, cup_path):
        yield act(j or jug_path[-1], c or cup_path[-1])
    held = as_np(lm.get_instance_pose(0, label="cup")[0])
    print(f"[scripted_demo] cup held at {held.round(3)} (from {cup0.round(3)})")

    hold_cup = (held_grip, cup_hand, 0)
    fix = np.zeros(3)  # the jug slips in the handle grip: steer the actual spout over where the cup actually is

    def steer(theta):
        nonlocal fix
        pos, quat = lm.get_instance_pose(0, label="cup")
        pos, rot = as_np(pos), from_wxyz(as_np(quat))
        hand, hand_rot, lip_target = pour(theta, (pos + rot.apply(axis_off))[:2], pos[2] + rim0 - cup0[2])
        jug_pos, jug_rot = jug()
        fix = np.clip(
            fix + SPOUT_GAIN * (lip_target - jug_pos - jug_rot.apply(SPOUT_IN_PITCHER)), -SPOUT_SLACK, SPOUT_SLACK
        )
        return act((hand + fix, hand_rot, 0), hold_cup)

    returning = int(np.ceil(TILT / RETURN_RATE)) + STEP_MARGIN
    theta, prev = 0.0, 0.0  # tilt until enough beans have left the jug, then back upright
    while theta < TILT_DRY:
        theta = min(theta + DRY_RATE, TILT_DRY)
        yield steer(theta)
    while (now := left_jug()) < POUR_TARGET and step < env.step_lim - returning:
        if now - prev < FLOW:  # tilt further only while the stream is weak: a surge overshoots the cup
            theta = min(theta + TILT_RATE, TILT)
        prev = now
        yield steer(theta)
    while theta > 0:
        theta = max(theta - RETURN_RATE, 0.0)
        yield steer(theta)
    for _ in range(STEP_MARGIN):
        yield steer(0.0)


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

    def pick(self, label, lift=LIFT, close=False):
        """Top-down grasp across the narrow side, lifted; returns (arm, hand, offset, grip_z, open_cmd, close_cmd)."""
        centre, rot, ext, bottom, top = self.object(label)
        axes = rot.as_matrix()  # columns are the object's local axes in world frame
        up = int(np.argmax(np.abs(axes[2])))
        narrow = min((a for a in range(3) if a != up), key=lambda a: ext[a])
        yaw = np.arctan2(axes[1, narrow], axes[0, narrow])  # close along the narrow horizontal local axis
        yaw = (yaw + np.pi / 2) % np.pi - np.pi / 2  # the jaw is symmetric: keep the wrist camera facing forward
        open_cmd = pad_open(ext[narrow])
        grip = np.r_[centre[:2], max(top - 0.03, bottom + FINGER_TIPS + TABLE_CLEAR)]
        arm = self.arm_for(centre[0])
        if abs(yaw) > np.deg2rad(60):  # sideways grasp: point the wrist toward the shoulder so it stays in reach
            if (arm.arm_key.startswith("right") and yaw > 0) or (arm.arm_key.startswith("left") and yaw < 0):
                yaw += -np.pi if yaw > 0 else np.pi
        hand = Rotation.from_euler("z", yaw) * TOP_ROT
        print(
            f"[scripted_demo] pick {label} at {centre.round(3)} size {ext.round(3)}"
            f" width {ext[narrow]:.3f} open {open_cmd:.2f} with {arm.arm_key}"
        )
        yield from self.move(
            arm,
            [
                (grip + [0, 0, lift], hand, open_cmd, 54),
                (grip, hand, open_cmd, 30),
                (grip, hand, 0, 18),  # close on the table so both fingers stop against the object
            ],
        )
        fingers = as_np(arm.key.data.joint_pos[0, list(arm.robot.gripper_joint_indices)])
        close_cmd = 0.0 if close else float(np.clip((0.023 - fingers.mean()) / 0.0445, 0.0, 1.0))  # 1.5 mm linear PD
        yield from self.move(arm, [(grip + [0, 0, lift], hand, close_cmd, 30)])
        centre, _, _, _, _ = self.object(label)
        fingers = as_np(arm.key.data.joint_pos[0, list(arm.robot.gripper_joint_indices)])
        print(f"[scripted_demo] lifted {label} to {centre.round(3)} (grip {grip.round(3)}), fingers {fingers.round(4)}")
        return arm, hand, arm.pose()[1].inv().apply(centre - arm.grasp_point()), grip[2] - bottom, open_cmd, close_cmd

    def place(self, arm, hand, offset_local, grip_height, open_cmd, close_cmd, xy, surface):
        """Lower the held object onto `surface` (z) at `xy`, release to `open_cmd` and back off upward."""
        point = np.r_[np.asarray(xy) - hand.apply(offset_local)[:2], surface + grip_height + PLACE_GAP]
        print(f"[scripted_demo] place at {point.round(3)} on surface {surface:.3f}")
        yield from self.move(
            arm,
            [
                (point + [0, 0, HOVER], hand, close_cmd, 48),
                (point, hand, close_cmd, 30),
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
    arm, _, _, _, _, close_cmd = yield from robot.pick("target", close=True)
    point, rot = arm.grasp_point(), arm.pose()[1]
    yield from robot.move(arm, [(point, rot, close_cmd, 36)])  # hold it up


def stack_actions(env):
    robot = Robot(env)
    labels = ["block_0", "block_1", "block_2"]
    base, *others = sorted(labels, key=lambda label: abs(robot.object(label)[0][0]))
    stack = [base]
    for label in others:
        arm, hand, offset_local, grip_height, open_cmd, close_cmd = yield from robot.pick(label, lift=HOVER)
        top = max(robot.object(b)[4] for b in stack)
        yield from robot.place(
            arm, hand, offset_local, grip_height, open_cmd, close_cmd, robot.object(stack[-1])[0][:2], top
        )
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
    if not TASK_ENV.end_flag[0]:  # EvalEnv starts each episode with success=True: running out of script is a fail
        TASK_ENV.success[0] = False
        TASK_ENV.end_flag[0] = True


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
