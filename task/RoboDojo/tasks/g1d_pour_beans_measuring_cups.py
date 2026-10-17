from env.environment.task_env import TaskEnv
from task.RoboDojo.tasks.g1d_pour_beans_cup_to_cup import G1dPourBeansCupToCupCommon


class G1dPourBeansMeasuringCupsCommon(G1dPourBeansCupToCupCommon):
    """g1d_pour_beans_cup_to_cup with the measuring cups of the desk photos: each hand picks a measuring cup from the
    side, and the red beans in the 500 ml one (~95 ml) are poured into the 400 ml one in mid-air; same success check
    (most beans out of the source cup, nearly all of those in the target when the source is back upright).

    Eval layouts come from scripts/g1d/build_asset.py (two_measuring_cups).
    """

    def gen_instruction(self, env_idx):
        return ["Pick up both measuring cups and pour the red beans from the larger one into the smaller one."]


class g1d_pour_beans_measuring_cups(G1dPourBeansMeasuringCupsCommon, TaskEnv):
    pass
