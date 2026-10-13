from env.robot_manager.robot_class.x5 import X5


# G1-D is one articulation with two arms (coupled); each arm reads Assets/Robots/g1d/<config_key>_robot_config.yml.
class G1DLeft(X5):
    config_key = "left"


class G1DRight(X5):
    config_key = "right"
