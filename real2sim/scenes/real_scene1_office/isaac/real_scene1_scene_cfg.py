# Scene configuration for unitree_sim_isaaclab using the real_scene1 USD package.
# Drop this file into tasks/common_scene/ and make a task's ObjectTableSceneCfg inherit RealScene1SceneCfg
# instead of TableCylinderSceneCfg. The robot is NOT part of this package; keep the task's robot config.
"""
real_scene1 scene configuration (office room reconstructed from a phone video).

Scene USD frame: origin on the floor under the task-table center, +Y from the robot side toward the
sofa/window wall. The whole package is shifted by SCENE_OFFSET so that the default G1 pose
(init_pos=(-0.15, 0.0, 0.76), facing +Y) stands in front of the table's robot-side edge.
"""
import json
import math
import os
from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.assets import AssetBaseCfg, RigidObjectCfg
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.utils import configclass

from tasks.common_config import CameraBaseCfg  # isort: skip

USD_DIR = Path(os.environ.get("REAL_SCENE1_USD_DIR", Path(__file__).resolve().parents[1] / "usd"))

# Table center lands at x=-0.15 (robot x) and its robot-side edge (scene y=-0.31) at world y=+0.30.
# Starting point only: tune to the real stance. The real (wheeled) robot stood ~0.64 m from the edge.
SCENE_OFFSET = (-0.15, 0.61, 0.0)
# Table height and object poses come from the build output, so --scale-correction rebuilds stay consistent.
_PARAMS = json.loads((USD_DIR / "build_params.json").read_text(encoding="utf-8"))
TABLE_TOP_Z = _PARAMS["table"]["top_z"]
OBJECT_POSES = {n: (s["pos"][0], s["pos"][1], s["yaw"]) for n, s in _PARAMS["objects"].items()}
# Entity exposed as `object` for the task's rewards/terminations (SceneEntityCfg("object")).
TARGET_OBJECT = "blue_cup_a"


def _world(x, y, z=0.0):
    return (x + SCENE_OFFSET[0], y + SCENE_OFFSET[1], z + SCENE_OFFSET[2])


def _yaw_quat(deg):
    h = math.radians(deg) / 2.0
    return (math.cos(h), 0.0, 0.0, math.sin(h))


def _rigid(name, prim_name):
    x, y, yaw = OBJECT_POSES[name]
    return RigidObjectCfg(
        prim_path=f"/World/envs/env_.*/{prim_name}",
        init_state=RigidObjectCfg.InitialStateCfg(pos=_world(x, y, TABLE_TOP_Z), rot=_yaw_quat(yaw)),
        spawn=sim_utils.UsdFileCfg(usd_path=str(USD_DIR / "objects" / f"{name}.usda")),
    )


def _other(name, prim_name):
    return None if name == TARGET_OBJECT else _rigid(name, prim_name)


@configclass
class RealScene1SceneCfg(InteractiveSceneCfg):
    """Room + task table + five rigid task objects reconstructed from real_scene1."""

    # 1. room shell and coarse furniture (static colliders, no rigid bodies)
    room_walls = AssetBaseCfg(
        prim_path="/World/envs/env_.*/Room",
        init_state=AssetBaseCfg.InitialStateCfg(pos=SCENE_OFFSET, rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=sim_utils.UsdFileCfg(usd_path=str(USD_DIR / "background.usda")),
    )
    # 2. white lift table (static colliders)
    packing_table = AssetBaseCfg(
        prim_path="/World/envs/env_.*/PackingTable",
        init_state=AssetBaseCfg.InitialStateCfg(pos=SCENE_OFFSET, rot=(1.0, 0.0, 0.0, 0.0)),
        spawn=sim_utils.UsdFileCfg(usd_path=str(USD_DIR / "table.usda")),
    )
    # 3. task objects (each USD root carries RigidBodyAPI + MassAPI)
    #    The target is spawned as `object`; its own named slot is left as None (skipped by InteractiveScene).
    object = _rigid(TARGET_OBJECT, "Object")
    blue_cup_a = _other("blue_cup_a", "BlueCupA")
    blue_cup_b = _other("blue_cup_b", "BlueCupB")
    paper_cup = _other("paper_cup", "PaperCup")
    tissue_pack = _other("tissue_pack", "TissuePack")
    plush_toy = _other("plush_toy", "PlushToy")
    # 4. lights: sun + window fills + (disabled) ceiling panels + weak dome. Global, not per env.
    #    Do not also keep the stock DomeLight(intensity=3000) from the base scene.
    light = AssetBaseCfg(
        prim_path="/World/RealScene1Lights",
        spawn=sim_utils.UsdFileCfg(usd_path=str(USD_DIR / "lights.usda")),
        init_state=AssetBaseCfg.InitialStateCfg(pos=SCENE_OFFSET),
    )
    # 5. third-person camera from the sofa side looking back at the robot (same orientation as the stock cfg)
    world_camera = CameraBaseCfg.get_camera_config(
        prim_path="/World/PerspectiveCamera",
        pos_offset=_world(0.0, 1.00, 1.55),
        rot_offset=(-0.00617, 0.00617, 0.70708, -0.70708),
        focal_length=16.5,
    )
