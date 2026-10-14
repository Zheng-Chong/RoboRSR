"""Run with the server's Isaac Lab Python environment. Example:
python run_pour.py --device cpu --water-ml 50
The CPU tensor pipeline is deliberate; fluid dynamics still uses the NVIDIA GPU.
"""
from pathlib import Path
import argparse,json,sys
PACK=Path(__file__).resolve().parent
sys.path.insert(0,str(PACK/'motion'))
from isaaclab.app import AppLauncher
parser=argparse.ArgumentParser(description='Scene 3: actual G1 + Dex1, feedback-controlled pouring')
parser.add_argument('--water-ml',type=float,default=50.,help='Initial source water; use 0 for dry motion.')
parser.add_argument('--scene-only',action='store_true',help='Show physics scene without executing the pouring sequence.')
parser.add_argument('--spec',type=Path,default=PACK/'scene_spec.json')
parser.add_argument('--log-dir',type=Path,default=PACK/'runs')
AppLauncher.add_app_launcher_args(parser);parser.set_defaults(device='cpu')
args=parser.parse_args()
if args.water_ml<0:parser.error('--water-ml must be nonnegative')
if args.water_ml>0 and args.device!='cpu':parser.error('Water readback uses --device cpu (CPU tensor pipeline); NVIDIA GPU dynamics is enabled separately.')
app=AppLauncher(args).app

import datetime,math
import numpy as np,torch
import carb
from pxr import Gf,Sdf,UsdGeom,UsdPhysics,UsdLux,PhysxSchema
import isaaclab.sim as sim_utils
from isaaclab.assets import Articulation,ArticulationCfg,RigidObject,RigidObjectCfg
from isaaclab.actuators import ImplicitActuatorCfg
from kinematics import G1Kinematics,transform
from task_geometry import initial_targets
from water import Water
from controller import PourController

def add_reference(stage,path,file,position=(0,0,0),yaw=0):
    p=UsdGeom.Xform.Define(stage,path);p.GetPrim().GetReferences().AddReference(str(file.resolve()))
    p.AddTranslateOp().Set(Gf.Vec3d(*position));p.AddOrientOp().Set(Gf.Quatf(math.cos(math.radians(yaw)/2),Gf.Vec3f(0,0,math.sin(math.radians(yaw)/2))))
    return p.GetPrim()

def main():
    s=json.loads(args.spec.read_text(encoding='utf-8'));dt=1/240
    stamp=datetime.datetime.now().strftime('%Y%m%d_%H%M%S');out=args.log_dir/stamp;out.mkdir(parents=True,exist_ok=True)
    (out/'requested_spec.json').write_text(json.dumps(s,ensure_ascii=False,indent=2),encoding='utf-8')
    kin=G1Kinematics();base=transform(s['robot']['root_position']);ready={};residuals={}
    for side,target in initial_targets(s).items():
        q,info=kin.solve(side,target,base=base,offset=s['pour']['tcp_in_dex1_base_m'])
        if info['position_error_m']>.006 or info['rotation_error_deg']>4:raise RuntimeError(f'Initial {side} target unreachable for current root pose: {info}')
        ready.update(dict(zip(kin.names(side),map(float,q))));residuals[side]=info
    for j in kin.joints:
        if j['type']!='fixed' and j['name'] not in ready:ready[j['name']]=s['pour']['open_finger_m'] if 'finger' in j['name'] else 0.
    cfg=sim_utils.SimulationCfg(dt=dt,device=args.device,use_fabric=False,render_interval=8)
    cfg.physx.enable_ccd=True;cfg.physx.bounce_threshold_velocity=.01
    sim=sim_utils.SimulationContext(cfg);stage=sim.stage
    physics=PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(cfg.physics_prim_path))
    if args.water_ml>0:
        # GPU dynamics and tensor-pipeline location are distinct choices.
        sim.get_physics_context().enable_gpu_dynamics(True)
        physics.CreateEnableGPUDynamicsAttr(True);physics.CreateBroadphaseTypeAttr('GPU')
        if hasattr(physics,'CreateEnableDirectGPUAPIAttr'):physics.CreateEnableDirectGPUAPIAttr(False)
    settings=carb.settings.get_settings()
    settings.set_bool('/physics/updateToUsd',True);settings.set_bool('/physics/updateParticlesToUsd',True)
    settings.set_bool('/physics/suppressReadback',False)
    add_reference(stage,'/World/Table',PACK/'assets/table.usdc')
    objects={}
    for name,c in s['objects'].items():
        path='/World/'+name;file=PACK/'assets/objects'/f'{name}.usdc'
        if name.endswith('cup'):
            objects[name]=RigidObject(RigidObjectCfg(prim_path=path,spawn=sim_utils.UsdFileCfg(usd_path=str(file)),
                init_state=RigidObjectCfg.InitialStateCfg(pos=tuple(c['position']))))
        else:
            p=add_reference(stage,path,file,c['position'],c['yaw']);UsdPhysics.RigidBodyAPI(p).CreateKinematicEnabledAttr(True)
    arm_effort={j['name']:j['effort'] for j in kin.joints if any(n in j['name'] for n in ['shoulder','elbow','wrist'])}
    robot_cfg=ArticulationCfg(prim_path='/World/Robot',spawn=sim_utils.UsdFileCfg(usd_path=str(PACK/'robot/g1_dex1.usdc'),
        activate_contact_sensors=True,articulation_props=sim_utils.ArticulationRootPropertiesCfg(enabled_self_collisions=True,solver_position_iteration_count=32,solver_velocity_iteration_count=8)),
        init_state=ArticulationCfg.InitialStateCfg(pos=tuple(s['robot']['root_position']),rot=tuple(s['robot']['root_quaternion_wxyz']),joint_pos=ready,joint_vel={'.*':0.}),
        soft_joint_pos_limit_factor=1.,actuators={
            'arms':ImplicitActuatorCfg(joint_names_expr=['.*_shoulder_.*','.*_elbow_joint','.*_wrist_.*'],effort_limit=arm_effort,velocity_limit=.8,stiffness=100.,damping=30.,armature=.1),
            'legs_waist':ImplicitActuatorCfg(joint_names_expr=['.*_hip_.*','.*_knee_joint','.*_ankle_.*','waist_.*'],effort_limit=150.,velocity_limit=.5,stiffness=1500.,damping=70.),
            'fingers':ImplicitActuatorCfg(joint_names_expr=['.*_dex1_finger_joint_.*'],effort_limit=20.,velocity_limit=.03,stiffness=2500.,damping=35.)})
    robot=Articulation(robot_cfg)
    anchor=UsdPhysics.FixedJoint(stage.GetPrimAtPath('/World/Robot/Joints/root_fixed'))
    anchor.GetLocalPos0Attr().Set(Gf.Vec3f(*s['robot']['root_position']))
    q=s['robot']['root_quaternion_wxyz'];anchor.CreateLocalRot0Attr(Gf.Quatf(q[0],Gf.Vec3f(*q[1:])))
    dome=UsdLux.DomeLight.Define(stage,'/World/Lighting/Fill');dome.CreateIntensityAttr(800);dome.CreateColorAttr(Gf.Vec3f(.88,.94,1))
    for n,p,power in [('Key',(-.6,-.6,2.5),1500),('FillSoft',(.8,.7,2.1),850)]:
        light=UsdLux.RectLight.Define(stage,'/World/Lighting/'+n);light.CreateWidthAttr(1.2);light.CreateHeightAttr(1.0);light.CreateIntensityAttr(power);light.AddTranslateOp().Set(Gf.Vec3d(*p))
    water=Water(stage,s,args.water_ml,cfg.physics_prim_path) if args.water_ml>0 else None
    sim.set_camera_view([-1.75,-1.60,1.62],[-.07,0,.77])
    sim.reset();robot.write_joint_state_to_sim(robot.data.default_joint_pos,robot.data.default_joint_vel)
    robot.set_joint_position_target(robot.data.default_joint_pos);robot.set_joint_velocity_target(robot.data.default_joint_vel)
    for obj in objects.values():obj.write_root_pose_to_sim(obj.data.default_root_state[:,:7]);obj.write_root_velocity_to_sim(obj.data.default_root_state[:,7:])
    robot.reset()
    for obj in objects.values():obj.reset()
    robot.update(dt)
    for obj in objects.values():obj.update(dt)
    controller=None
    if not args.scene_only:controller=PourController(robot,objects['source_cup'],objects['receiver_cup'],water,s,dt*2,out/'trace.jsonl')
    print('Scene ready. G1 fixed pelvis:',s['robot']['root_position'],'Water:',args.water_ml,'mL. Log:',out,flush=True)
    frames=0;result={'status':'not_started','physics_validation_performed_before_delivery':False}
    try:
      while app.is_running():
        if controller and frames%2==0:
            controller.step()
            if controller.done:
                result={'status':'failed' if controller.failed else 'success' if water else 'dry_motion_complete','reason':controller.reason,**controller.measurement};break
        robot.write_data_to_sim()
        for obj in objects.values():obj.write_data_to_sim()
        sim.step();robot.update(dt)
        for obj in objects.values():obj.update(dt)
        frames+=1
    except Exception as e:
      result={'status':'exception','error':str(e)}
      raise
    finally:
      if controller:controller.close()
      (out/'result.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
      print('Result:',result,flush=True)

if __name__=='__main__':
    try:main()
    finally:app.close()
