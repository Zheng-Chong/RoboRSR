"""GPU smoke for the G1-D body: drive the waist, the lift and (g1d_mobile) the chassis through EvalEnv.take_action with
the arms held, and check the measured joints, the torso pose and the reset. Run two episodes (the second starts by
checking that the reset put the body back):

    CUDA_VISIBLE_DEVICES=0 PYTHONPATH=.:XPolicyLab EVAL_NUM=2 python scripts/g1d/body_probe.py \
        --env_cfg_type g1d_mobile --task_name g1d_general_pickup --seed 0

Each episode prints one `[body_probe] {...}` JSON line and passes (success) only if every check holds.
"""

import json

import numpy as np

from scripts.g1d.scripted_demo import main as run

WAIST, LIFT = [0.3, 0.2], 0.1  # rad (yaw, pitch); m above the start height
BACK, TURN = -0.3, 0.3  # m/s, rad/s for 2 s each; backwards, the table is ahead
episodes = []


def eval_one_episode(TASK_ENV, model_client):
    rm = TASK_ENV.robot_manager
    robot = next(r for r in rm.robot_list if hasattr(r, "body_dims"))

    def body():
        return rm.get_body_state(robot, env_idx_list=[0])[0]

    def torso():
        return rm.get_link_pose(robot, "torso_link", env_idx_list=[0], is_relative=True)[0]

    obs = TASK_ENV.get_obs()
    hold = {k: obs["state"][k] for k in ("left_arm_joint_state", "right_arm_joint_state")}
    hold.update({k: [1.0] for k in ("left_ee_joint_state", "right_ee_joint_state")})
    start, start_torso = body(), torso()
    checks = {
        "reset_lift_m": abs(start["lift_joint_state"][0] - (episodes[0] if episodes else start["lift_joint_state"][0]))
    }
    if robot.mobile:
        checks["reset_base_pose"] = float(np.abs(start["base_pose"]).max())
    episodes.append(start["lift_joint_state"][0])

    def act(steps, **body_cmd):
        for _ in range(steps):
            TASK_ENV.take_action({**hold, **body_cmd})
            TASK_ENV.get_obs()  # streams the camera frames into the episode videos

    lift = start["lift_joint_state"][0] + LIFT
    act(60, lift_joint_state=[lift])  # 0.1 m at 76.5 mm/s takes 1.3 s
    checks["lift_m"] = abs(body()["lift_joint_state"][0] - lift)
    checks["torso_rise_m"] = abs(torso()[2] - start_torso[2] - LIFT)
    act(30, waist_joint_state=WAIST)  # the lift holds its last command
    checks["waist_rad"] = float(np.abs(np.subtract(body()["waist_joint_state"], WAIST)).max())
    checks["lift_held_m"] = abs(body()["lift_joint_state"][0] - lift)
    if robot.mobile:
        before = torso()
        act(60, base_velocity=[BACK, 0.0])
        moved = body()["base_pose"]
        checks["back_m"] = abs(moved[0] - 2 * BACK)
        checks["torso_follows_m"] = abs(np.linalg.norm(torso()[:2] - before[:2]) - abs(moved[0]))
        act(60, base_velocity=[0.0, TURN])
        checks["turn_rad"] = abs(body()["base_pose"][2] - 2 * TURN)
        act(15)  # no base_velocity: the chassis stops
        checks["stopped_mps"] = float(np.abs(body()["base_velocity"]).max())
    arms = TASK_ENV.get_obs()["state"]
    checks["arm_drift_rad"] = float(max(np.abs(np.subtract(arms[k], hold[k])).max() for k in hold if "arm" in k))
    try:
        TASK_ENV.validate_action_dict({**hold, "base_velocity": [0.0, 0.0]})
        checks["base_key_accepted"] = robot.mobile
    except ValueError:
        checks["base_key_accepted"] = not robot.mobile
    # Tolerances: joint tracking 0.01 rad / 2 mm, chassis 3 cm and 0.05 rad after 2 s (acceleration lag), reset 1 mm.
    limits = dict(waist_rad=0.01, lift_m=0.002, lift_held_m=0.002, torso_rise_m=0.002, back_m=0.03, turn_rad=0.05)
    limits.update(torso_follows_m=0.01)
    limits.update(stopped_mps=0.01, arm_drift_rad=0.02, reset_lift_m=0.001, reset_base_pose=0.001)
    passed = all(v if isinstance(v, bool) else v <= limits[k] for k, v in checks.items())
    print("[body_probe] " + json.dumps(dict(passed=passed, **{k: round(float(v), 5) for k, v in checks.items()})))
    TASK_ENV.success[0], TASK_ENV.end_flag[0] = passed, True


if __name__ == "__main__":
    run(policy_module="scripts.g1d.body_probe")
