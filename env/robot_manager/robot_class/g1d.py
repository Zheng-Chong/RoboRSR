import numpy as np

from env.robot_manager.robot_class.x5 import X5

# Official G1-D interface (unitree_sdk2 example/g1/g1d, AgvClient): the waist is low-level motors 12 WaistYaw and
# 14 WaistPitch, position-controlled; the lift column is one height in m (rt/hispeed_state); the Flagship chassis
# takes Move(vx, vyaw). The URDF names the waist joints by the wrong axes: torso_Joint turns about z, Yaw_Joint about y.
WAIST_JOINTS = ["torso_Joint", "Yaw_Joint"]  # waist_joint_state: [yaw, pitch] rad
LIFT_JOINTS = ["LZ_mt_Joint", "LZ_it_Joint"]  # lift_joint_state: [height] m, 0..0.42, split equally over the stages
BASE_JOINTS = ["base_x_joint", "base_y_joint", "base_yaw_joint"]  # g1d_mobile_dex1.urdf, in the start-pose frame
BASE_LIMITS = np.array([1.5, 0.6])  # base_velocity: [vx m/s, vyaw rad/s], AgvClient.Move limits
# A blocked chassis stalls instead of winding up force: its target leads the measured pose by at most this (m, rad).
BASE_LEAD = np.array([0.01, 0.01, 0.02])


# G1-D is one articulation with two arms (coupled); each arm reads Assets/Robots/g1d/<config_key>_robot_config.yml.
class G1DLeft(X5):
    config_key = "left"
    asset_name = "g1d"

    def __init__(self, cfg: dict):
        super().__init__(cfg)
        # The body (waist, lift, chassis) is one per articulation and rides on the left arm's entry.
        self.mobile = self.robot_name == "g1d_mobile"
        self.body_joints_name = WAIST_JOINTS + LIFT_JOINTS + (BASE_JOINTS if self.mobile else [])
        self.body_dims = {"waist_joint_state": 2, "lift_joint_state": 1}
        if self.mobile:
            self.body_dims["base_velocity"] = 2

    def body_state(self, q, qd):
        """Body joint positions and velocities (body_joints_name order) -> the body_dims keys, plus base_pose
        ([x, y, yaw] from the start pose) on the mobile version; base_velocity is in the chassis frame."""
        state = {"waist_joint_state": list(q[:2]), "lift_joint_state": [q[2] + q[3]]}
        if self.mobile:
            yaw = q[6]
            state["base_velocity"] = [qd[4] * np.cos(yaw) + qd[5] * np.sin(yaw), qd[6]]
            state["base_pose"] = list(q[4:7])
        return state

    def body_targets(self, command, target, q, dt):
        """Joint position and velocity targets for one control step from the body_dims command; `target` holds the
        last position targets and `q` the measured positions."""
        lift = np.full(2, command["lift_joint_state"][0] / 2)
        pos = np.concatenate([command["waist_joint_state"], lift])
        if not self.mobile:
            return pos, np.zeros(4)
        vx, vyaw = np.clip(command["base_velocity"], -BASE_LIMITS, BASE_LIMITS)
        vel = np.array([vx * np.cos(target[6]), vx * np.sin(target[6]), vyaw])
        base = np.clip(target[4:] + vel * dt, q[4:] - BASE_LEAD, q[4:] + BASE_LEAD)
        return np.concatenate([pos, base]), np.concatenate([np.zeros(4), vel])


class G1DRight(X5):
    config_key = "right"
    asset_name = "g1d"
