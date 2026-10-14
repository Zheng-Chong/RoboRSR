from env.environment.task_env import TaskEnv
from task.RoboDojo.tasks.stack_blocks import StackBlocksCommon


class StackBlocksG1dCommon(StackBlocksCommon):
    def __init__(self, config, app, **kwargs):
        super().__init__(config, app, **kwargs)
        self.step_lim = 660  # stack_blocks's 550 actions at 25 Hz, at G1-D's 30 Hz


class g1d_stack_blocks(StackBlocksG1dCommon, TaskEnv):
    """stack_blocks for the Unitree G1-D: same objects and success check, layouts within G1-D reach (build_asset.py)."""
