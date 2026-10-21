from env.robot_manager.robot_config.g1d import FINGER_PADS, get_robot_config as _g1d_config

__all__ = ["FINGER_PADS"]  # same grippers as g1d: RobotManager puts the pad ContactSensor on them


def get_robot_config():
    return _g1d_config(mobile=True)
