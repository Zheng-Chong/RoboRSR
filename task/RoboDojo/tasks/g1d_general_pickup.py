from env.environment.task_env import TaskEnv
from task.RoboDojo.tasks.general_pickup import GeneralPickupCommon


class GeneralPickupG1dCommon(GeneralPickupCommon):
    def __init__(self, config, app, **kwargs):
        super().__init__(config, app, **kwargs)
        self.step_lim = 240  # general_pickup's 200 actions at 25 Hz, at G1-D's 30 Hz


class g1d_general_pickup(GeneralPickupG1dCommon, TaskEnv):
    """general_pickup for the Unitree G1-D: objects fitting the Dex1-1 jaw, layouts within G1-D reach (build_asset.py)."""
