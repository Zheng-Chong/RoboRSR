from copy import deepcopy
from typing import List

import numpy as np

from env.description_manager.desc_manager import DescManager


class ObsManager:
    ANNOTATORS_TO_COLLECT = {
        "rgb": "color",
        "depth": "depth",
        "distance_to_image_plane": "distance_to_image_plane",
    }

    def __init__(
        self,
        obs_config,
        num_envs,
        dt,
        task_name,
        description_cfg,
        seeds_per_env: List[int] = None,
    ):
        self.obs_config = obs_config
        self.num_envs = num_envs
        self.dt = dt
        self.task_name = task_name
        self.description_cfg = description_cfg
        self.desc_manager = DescManager(
            num_envs=self.num_envs,
            description_cfg=self.description_cfg,
            desc_type="seen",
            seeds_per_env=seeds_per_env,
        )
        self.robot_cfg = obs_config.get("robot", {})
        # Gripper opening in obs["state"]: the last command ("command", default) or the finger joints ("measured");
        # obs["action"] always holds the command.
        self.ee_state = self.robot_cfg.get("ee_state", "command")
        # body_states: true adds the robot's body (G1-D waist, lift, chassis) to obs["state"] / obs["action"].
        if self.ee_state not in ("command", "measured"):
            raise ValueError(f"observation.robot.ee_state must be 'command' or 'measured', got {self.ee_state!r}")
        self.collect_freq = self.obs_config.get("collect_freq", 0)
        # Frames the SDG dispatcher must deliver after the physics step before
        # images are read. One is exact with the zero-delay Kit settings that
        # the entrypoints enable; render_sync adds one more if they are absent.
        self.capture_render_passes = int(self.obs_config.get("capture_render_passes", 1))
        self.capture_render_max_passes = int(self.obs_config.get("capture_render_max_passes", 16))
        if self.collect_freq > 0:
            self.collect_interval = 1.0 / (self.dt * self.collect_freq)
        else:
            self.collect_interval = 0.0
        if not self.collect_interval.is_integer():
            raise ValueError("Collect Interval must be integer!")
        vision_cfg = self.obs_config.get("vision", {})
        self.collect_approximate_depth = vision_cfg.get("approximate_depth", True)
        self.collect_depth = vision_cfg.get("depth", False)
        self.collect_intrinsic_matrix = vision_cfg.get("intrinsic_matrix", False)
        self.collect_extrinsic_matrix = vision_cfg.get("extrinsic_matrix", False)
        self.collect_shape = vision_cfg.get("shape", True)

    def initialize(self, env):
        self.env = env
        if hasattr(env, "robot_manager"):
            self.robot_manager = env.robot_manager
        else:
            self.robot_manager = None
        if hasattr(env, "capture_manager"):
            self.capture_manager = env.capture_manager
        else:
            self.capture_manager = None
        if hasattr(env, "camera_manager"):
            self.camera_manager = env.camera_manager
        else:
            self.camera_manager = None

        self.initialize_instruction()

    def initialize_instruction(self):
        self.desc_manager.initialize(self.env)
        self.instruction = self.desc_manager.get_one_description()

    def reset(self):
        self.desc_manager.reset()
        self.instruction = self.desc_manager.get_one_description()

    def render_for_capture(self):
        """Render until the camera buffers hold the current physics state.

        Call this instead of a bare ``env.render()`` right before ``get_obs``.
        """
        from env.camera_manager.capture.render_sync import wait_for_latest_cameras

        wait_for_latest_cameras(
            self.env,
            capture_manager=self.capture_manager,
            min_passes=self.capture_render_passes,
            max_passes=self.capture_render_max_passes,
        )

    def get_obs(self, env_idx_list=None):  # batch
        if env_idx_list is None:
            env_idx_list = range(self.num_envs)
        obs = dict()
        for env_idx in env_idx_list:
            obs[env_idx] = dict()
            obs[env_idx]["vision"] = dict()
            obs[env_idx]["state"] = dict()
            obs[env_idx]["action"] = dict()
            obs[env_idx]["data_format_version"] = "v1.0"
            obs[env_idx]["additional_info"] = {
                "frequency": self.collect_freq,
            }
            obs[env_idx]["instruction"] = self.instruction[env_idx]
        if self.camera_manager is not None and self.capture_manager is not None:
            try:
                data = self.capture_manager.step(env_ids=env_idx_list)
                for ith in range(self.camera_manager.num_cams):
                    cam_data = data[ith]
                    for annotator_name, env_list in cam_data.items():
                        collect_name = self.ANNOTATORS_TO_COLLECT[annotator_name]
                        for idx, env_idx in enumerate(env_idx_list):
                            camera_name = self.camera_manager.camera_names[env_idx][ith]
                            if camera_name not in obs[env_idx]["vision"]:
                                obs[env_idx]["vision"][camera_name] = dict()
                            obs[env_idx]["vision"][camera_name][collect_name] = env_list[idx]["data"]

                            if annotator_name == "rgb":
                                obs[env_idx]["vision"][camera_name][collect_name] = obs[env_idx]["vision"][camera_name][
                                    collect_name
                                ][:, :, :3]
                                if self.collect_shape:
                                    obs[env_idx]["vision"][camera_name]["shape"] = obs[env_idx]["vision"][camera_name][
                                        collect_name
                                    ].shape

            except Exception as e:
                import traceback

                stack_trace = traceback.format_exc()
                print(stack_trace)
                print("[get_obs] Camera observation capture failed.")
                raise e

            for env_idx in env_idx_list:
                for cam_data in obs[env_idx]["vision"].values():
                    if "distance_to_image_plane" in cam_data:
                        raw = cam_data.pop("distance_to_image_plane")
                        if hasattr(raw, "ndim") and raw.ndim >= 1 and raw.shape[-1] == 1:
                            raw = raw.squeeze(-1)
                        if self.collect_approximate_depth:
                            cam_data["approximate_depth"] = np.clip(raw * 1000, 0, 65535).astype(np.uint16)
                        if self.collect_depth:
                            cam_data["depth"] = raw.astype(np.float32)

            if self.collect_intrinsic_matrix or self.collect_extrinsic_matrix:
                for ith in range(self.camera_manager.num_cams):
                    for idx, env_idx in enumerate(env_idx_list):
                        camera_name = self.camera_manager.camera_names[env_idx][ith]
                        if self.collect_intrinsic_matrix:
                            obs[env_idx]["vision"][camera_name]["intrinsic_matrix"] = (
                                self.camera_manager.get_camera_intrinsics(ith, env_idx)
                            )
                        if self.collect_extrinsic_matrix:
                            obs[env_idx]["vision"][camera_name]["extrinsic_matrix"] = (
                                self.camera_manager.get_camera_extrinsics(ith, env_idx)
                            )

        if self.robot_manager is not None:
            try:
                for robot in self.robot_manager.robot_list:
                    if robot.type != "target":
                        continue
                    if self.robot_cfg["joint_states"]:
                        if robot.robot_type == "arm":
                            joints = self.robot_manager.get_joint(robot=robot, env_idx_list=env_idx_list)
                        for env_idx in env_idx_list:
                            if robot.robot_type == "arm":
                                obs[env_idx]["state"][f"{robot.arm_name}_joint_state"] = joints[env_idx]
                                obs[env_idx]["action"][f"{robot.arm_name}_joint_state"] = joints[env_idx]

                    if self.robot_cfg["world_ee_state"]:
                        endpose = self.robot_manager.get_real_endpose(robot=robot, env_idx_list=env_idx_list)
                        for env_idx in env_idx_list:
                            if self.robot_manager.target_arm_nums == 1:
                                obs[env_idx]["state"]["ee_pose"] = endpose[env_idx]
                            else:
                                name = robot.arm_name.split("_")[0]
                                obs[env_idx]["state"][f"{name}_ee_pose"] = endpose[env_idx]

                    if self.robot_cfg.get("body_states", False) and hasattr(robot, "body_dims"):
                        body = self.robot_manager.get_body_state(robot=robot, env_idx_list=env_idx_list)
                        for env_idx in env_idx_list:
                            obs[env_idx]["state"].update(body[env_idx])
                            prev = self.robot_manager.control_manager.prev_control[env_idx]
                            for name in robot.body_dims:
                                if name in prev:
                                    obs[env_idx]["action"][name] = list(prev[name]["position"])

                if hasattr(self.robot_manager, "control_manager"):
                    measured = (
                        self._measured_grippers(env_idx_list)
                        if self.ee_state == "measured" and self.robot_cfg["joint_states"]
                        else {}
                    )
                    for env_idx in env_idx_list:
                        for (
                            key,
                            data,
                        ) in self.robot_manager.control_manager.prev_control[env_idx].items():
                            position = data["position"]
                            if key.endswith("ee_joint_state"):
                                end_effector_name = self.robot_manager.restore_name(deepcopy(key))
                                robot = self.robot_manager.get_robot_by_gripper_name(end_effector_name)
                                if robot.ee_type == "gripper":
                                    val = self._to_gripper_scale(robot, position[0])
                                    obs[env_idx]["action"][key] = [val]
                                    if self.robot_cfg["joint_states"]:
                                        obs[env_idx]["state"][key] = [
                                            measured.get(end_effector_name, {}).get(env_idx, val)
                                        ]
                                else:
                                    obs[env_idx]["action"][key] = position
                                    if self.robot_cfg["joint_states"]:
                                        obs[env_idx]["state"][key] = position

            except Exception as e:
                import traceback

                stack_trace = traceback.format_exc()
                print(stack_trace)
                print("[get_obs] Robot observation capture failed.")
                raise e
        return obs

    @staticmethod
    def _to_gripper_scale(robot, q):
        """Finger joint position -> the 0..1 scale of gripper commands (inverse of EvalEnv.take_action_batch)."""
        lo, hi = robot.gripper_scale
        return (q - lo) / (hi - lo) if robot.gripper_move["sign"] == 1 else (hi - q) / (hi - lo)

    def _measured_grippers(self, env_idx_list):
        """{gripper_name: {env_idx: opening}} from the finger joints on the command scale: the mean over the fingers,
        the mimic finger mapped back onto the base one (Dex1 fingers are separate joints and stop on an object apart)."""
        result = {}
        for robot in self.robot_manager.robot_list:
            if robot.type != "target" or robot.ee_type != "gripper":
                continue
            joints = self.robot_manager.get_end_effector_real_val(robot=robot, env_idx_list=env_idx_list)
            mimic, gain, offset = robot.gripper_move["mimic"]
            result[robot.gripper_name] = {}
            for env_idx in env_idx_list:
                base = [
                    (q - offset) / gain if n == mimic else q for n, q in zip(robot.gripper_joints_name, joints[env_idx])
                ]
                result[robot.gripper_name][env_idx] = float(self._to_gripper_scale(robot, np.mean(base)))
        return result
