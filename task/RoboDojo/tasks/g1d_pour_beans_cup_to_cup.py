from env.environment.task_env import TaskEnv
from env.reward_manager.reward_manager import RewardManager

# The source cup is a sixth full of red beans (~55 ml); as in the real G1D-PourBeansPlus clips most of them go over:
POURED = 0.6  # at least 60% of the beans leave the source cup ...
IN_CUP = 0.9  # ... and 90% of what left it is in the target cup


class G1dPourBeansCupToCupCommon:
    """Unitree G1-D, both arms: each hand picks a clear plastic cup from the side, lifts it and the source cup's beans
    are poured into the target cup in mid-air (the real G1D-PourBeansPlus clips).

    Eval layouts come from scripts/g1d/build_asset.py (two_cups).
    """

    def __init__(self, config, app, **kwargs):
        super().__init__(config, app, **kwargs)
        self.reward_manager = RewardManager(self.num_envs)
        self.step_lim = 600  # ~200 actions to pick and lift both cups, the rest to pour and come back upright

    def _post_setup_scene(self, sim):
        super()._post_setup_scene(sim)
        self.reward_manager.initialize(self)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed, options=options)
        self.reward_manager.reset()

    def pour_check(self):
        return self.reward_manager.is_A_fluid_in_B(
            "beans",
            "target_cup",
            percentage_threshold=IN_CUP,
            label_C="source_cup",
            B_buffer=0.0,
            C_buffer=0.0,
            B_z_threshold=0.001,
            C_residual_threshold=1 - POURED,
            ignore_scattered=True,
            scatter_connect_radius=0.012,  # ~2 bean spacings (5.6 mm)
            scatter_min_component_size=7,
            max_ignore_ratio=0.02,
        )

    def run_reward(self):
        # checked each time the source cup comes back upright
        self.reward_manager.trigger_check(
            [self.reward_manager.is_axis_up(label="source_cup", axis=[0, 0, 1], threshold=30)],
            [self.pour_check()],
            trigger_mode="rising_edge",
        )

    def gen_instruction(self, env_idx):
        return ["Pick up both cups and pour the red beans from one cup into the other."]


class g1d_pour_beans_cup_to_cup(G1dPourBeansCupToCupCommon, TaskEnv):
    pass
