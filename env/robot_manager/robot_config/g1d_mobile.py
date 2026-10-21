from env.robot_manager.robot_config.g1d import get_robot_config as _g1d_config


def get_robot_config():
    return _g1d_config(mobile=True)
