from env.environment.task_env import TaskEnv
from task.RoboDojo.tasks.g1d_pour_beans_cup_to_cup import G1dPourBeansCupToCupCommon

TARGETS = {
    "measuring_cup": "the short measuring cup",
    "plastic_mug": "the white mug",
}


class G1dPourBeansCupOrMugCommon(G1dPourBeansCupToCupCommon):
    """g1d_pour_beans_measuring_cups with the target varied: the tall measuring cup's red beans (300-600 per layout)
    go into either the short measuring cup or the white plastic mug with a handle, among unrelated distractors. Same
    success check as the cup-to-cup pour; the instruction names the target.

    Layouts come from scripts/g1d/prepare_bean_pour_layouts.py.
    """

    def gen_instruction(self, env_idx):
        name = self.scene_manager.layout_manager.get_instance_name(env_idx, "target_cup")
        target = next((text for kind, text in TARGETS.items() if name.startswith(kind)), "the other cup")
        return [f"Pick up the tall measuring cup and pour all the red beans into {target}."]


class g1d_pour_beans_cup_or_mug(G1dPourBeansCupOrMugCommon, TaskEnv):
    pass
