import re

from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets.articulation import ArticulationCfg
import isaaclab.sim as sim_utils
from isaaclab.sim.spawners.from_files import from_files
import numpy as np
from pxr import Usd, UsdGeom, UsdShade, Vt

from env.global_configs import ROBOTS_PATH

# Real G1-D finish (product photo + head/wrist recordings). URDF colours are diffuse-only, so links matching
# these names get PBR materials; patterns are disjoint.
MATERIALS = {
    "metal": sim_utils.PreviewSurfaceCfg(diffuse_color=(0.62, 0.63, 0.65), metallic=1.0, roughness=0.5),  # matte
    "black": sim_utils.PreviewSurfaceCfg(diffuse_color=(0.02, 0.02, 0.02), roughness=0.8),
}
LINK_MATERIALS = {
    r"torso_link|AGV_link|LZ_.*_Link|.*_(shoulder_.*|elbow|wrist_.*)_link": "metal",
    r".*_dex1_.*|Yaw_Link|Pitching_Link|head_link|logo_link|.*_Wheel_Link": "black",
}
# Only the ribbed ring of the elbow link (3.5-7.5 cm along the forearm, link frame x) is black; the elbow bracket
# and the neck to the wrist roll stay metal.
RING_LINKS, RING_X = r".*_elbow_link", (0.035, 0.075)


def _paint_ring(link, material):
    """Bind `material` to the faces of the link's meshes whose centroid lies in RING_X."""
    xform_cache = UsdGeom.XformCache()
    for p in Usd.PrimRange(link.GetChild("visuals")):
        if not p.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(p)
        to_link = np.array(xform_cache.ComputeRelativeTransform(p, link)[0])
        pts = np.array(mesh.GetPointsAttr().Get())
        x = (pts @ to_link[:3, :3] + to_link[3, :3])[:, 0]  # Gf matrices act on row vectors
        counts = np.array(mesh.GetFaceVertexCountsAttr().Get())
        face_x = np.add.reduceat(x[np.array(mesh.GetFaceVertexIndicesAttr().Get())], np.r_[0, np.cumsum(counts)[:-1]])
        face_x /= counts
        faces = np.nonzero((face_x > RING_X[0]) & (face_x < RING_X[1]))[0].tolist()
        binding = UsdShade.MaterialBindingAPI.Apply(p)
        subset = binding.CreateMaterialBindSubset("ring", Vt.IntArray(faces), UsdGeom.Tokens.face)
        UsdShade.MaterialBindingAPI.Apply(subset.GetPrim()).Bind(material)


@sim_utils.clone
def _spawn_g1d(prim_path, cfg, *args, **kwargs):
    prim = from_files.spawn_from_urdf.__wrapped__(prim_path, cfg, *args, **kwargs)
    materials = {}
    for name, material_cfg in MATERIALS.items():
        material_cfg.func(f"{prim_path}/Looks/{name}", material_cfg)  # returns the shader, not the material
        materials[name] = UsdShade.Material(prim.GetStage().GetPrimAtPath(f"{prim_path}/Looks/{name}"))
    for link in prim.GetChildren():
        visuals = link.GetChild("visuals")
        name = next((m for pattern, m in LINK_MATERIALS.items() if re.fullmatch(pattern, link.GetName())), None)
        if not visuals or name is None:
            continue
        visuals.SetInstanceable(False)  # material bindings are skipped on instanced prims
        if re.fullmatch(RING_LINKS, link.GetName()):
            # bind per mesh, not stronger-than-descendants on visuals, so the ring subset can override it
            for p in Usd.PrimRange(visuals):
                if p.IsA(UsdGeom.Mesh):
                    UsdShade.MaterialBindingAPI.Apply(p).Bind(materials[name])
            _paint_ring(link, materials["black"])
        else:
            sim_utils.bind_visual_material(str(visuals.GetPath()), str(materials[name].GetPath()))
    # The head camera sits inside the head shell, as on the real robot; hide the shell so it does not block the view.
    UsdGeom.Imageable(prim.GetChild("head_link").GetChild("visuals")).MakeInvisible()
    return prim


def get_robot_config():
    # Unitree G1-D (wheeled base locked) with Dex1-1 grippers; URDF built by scripts/g1d/build_asset.py.
    return ArticulationCfg(
        spawn=sim_utils.UrdfFileCfg(
            func=_spawn_g1d,
            asset_path=f"{ROBOTS_PATH}/g1d/g1d_dex1.urdf",
            usd_dir=f"{ROBOTS_PATH}/g1d/usd",  # converted once, reused while the URDF/config hash matches
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
