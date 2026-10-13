from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
import isaaclab.sim as sim_utils

from env.global_configs import ROBOTS_PATH


def get_robot_config():
    # Unitree G1-D (wheeled base locked) with Dex1-1 grippers; URDF built by scripts/g1d/build_asset.py.
    return ArticulationCfg(
        spawn=sim_utils.UrdfFileCfg(
            asset_path=f"{ROBOTS_PATH}/g1d/g1d_dex1.urdf",
            fix_base=True,
            merge_fixed_joints=False,  # keep d435_link / *_dex1_base_link for cameras and the EE frame
            joint_drive=sim_utils.UrdfConverterCfg.JointDriveCfg(
                gains=sim_utils.UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=0.0, damping=0.0)
            ),
            rigid_props=sim_utils.RigidBodyPropertiesCfg(
                disable_gravity=True,
                max_depenetration_velocity=5.0,
            ),
            articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                enabled_self_collisions=True,
                solver_position_iteration_count=8,
                solver_velocity_iteration_count=0,
                fix_root_link=True,
            ),
        ),
        init_state=ArticulationCfg.InitialStateCfg(
            joint_pos={
                "LZ_mt_Joint": 0.0,  # lift column stages, 0..0.21 m each; raise for taller tables
                "LZ_it_Joint": 0.0,
                "Yaw_Joint": 0.0,  # waist pitch (forward lean) despite the Unitree name
                "torso_Joint": 0.0,  # waist yaw
                ".*_shoulder_.*": 0.0,
                ".*_elbow_joint": 0.0,  # G1 elbow 0 = forearm pointing forward
                ".*_wrist_.*": 0.0,
                ".*_dex1_finger_joint_.*": 0.0245,  # open
            },
        ),
        actuators={
            "body": ImplicitActuatorCfg(
                joint_names_expr=["LZ_.*_Joint", "Yaw_Joint", "torso_Joint"],
                effort_limit_sim=1e4,
                stiffness=1e6,
                damping=1e4,
            ),
            "arm": ImplicitActuatorCfg(
                joint_names_expr=[".*_shoulder_.*", ".*_elbow_joint", ".*_wrist_.*"],
                effort_limit_sim=100.0,
                velocity_limit_sim=5.0,
                stiffness=4400.0,
                damping=40.0,
                armature=0.01,
            ),
            "gripper": ImplicitActuatorCfg(
                joint_names_expr=[".*_dex1_finger_joint_.*"],
                effort_limit_sim=100.0,
                stiffness=2300,
                damping=100,
            ),
        },
    )
