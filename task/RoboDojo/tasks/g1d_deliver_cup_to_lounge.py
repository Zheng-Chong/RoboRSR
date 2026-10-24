import json
from pathlib import Path

import numpy as np

from env.environment.task_env import TaskEnv
from env.reward_manager.reward_manager import RewardManager


class DeliverCupToLoungeG1dCommon:
    """G1-D mobile manipulation in the counter scene (real_scene2): pick the cup from the countertop, drive to the
    lounge's round table and set it down there, upright and released. The chairs around the table are rigid bodies
    the chassis can push. Layouts and the table come from scripts/g1d/build_asset.py and the room's
    usd/build_params.json ("lounge")."""

    TABLE = "blue"  # the round table with the four blue chairs
    EDGE_MARGIN = 0.04  # m: the cup's centre this far inside the table's rim

    def __init__(self, config, app, **kwargs):
        super().__init__(config, app, **kwargs)
        self.reward_manager = RewardManager(self.num_envs)
        self.step_lim = 1500  # 50 s: pick ~6 s, drive ~4.5 m with turns ~25 s, place ~5 s, plus slack
        self.table_xy = [None] * self.num_envs
        self.table_r = [None] * self.num_envs
        self.table_z = [None] * self.num_envs

    def _post_setup_scene(self, sim):
        super()._post_setup_scene(sim)
        self.reward_manager.initialize(self)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.reward_manager.reset()
        for env_idx in range(self.num_envs):
            room = self.scene_manager.config[f"env{env_idx}"]["Room"]
            lounge = json.loads((Path(room["usd_path"]).parent / "usd" / "build_params.json").read_text())["lounge"]
            self.table_xy[env_idx] = list(np.add(lounge["tables"][self.TABLE], room["default_pos"][:2]))
            self.table_r[env_idx] = lounge["table_r"]
            self.table_z[env_idx] = lounge["table_top_z"] + room["default_pos"][2]

    def delivered_checks(self):
        rm = self.reward_manager
        return [
            rm.is_A_xy_distance_close_to_pos(
                "cup", pos=self.table_xy, dis_threshold=self.table_r[0] - self.EDGE_MARGIN
            ),
            # the cup starts on the 0.833 m countertop: ~8 cm lower on the 0.75 m table, ~83 cm lower on the floor
            rm.is_lift("cup", z_threshold=-0.15),
            rm.is_axis_up("cup", axis=[0, 0, 1], threshold=15),
            rm.is_all_gripper_open(open_threshold=0.6),
        ]

    def run_reward(self):
        self.reward_manager.check(self.delivered_checks())

    def gen_instruction(self, env_idx):
        return ["Take the cup from the counter to the round table in the lounge and put it down."]


class g1d_deliver_cup_to_lounge(DeliverCupToLoungeG1dCommon, TaskEnv):
    """Mobile G1-D (g1d_counter_mobile): carry the cup from the counter to the lounge table."""
